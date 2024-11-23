#!/usr/bin/env python3
import os
import fcntl
import time
import threading
import math
from itertools import zip_longest
import numpy as np

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
    GripperCommandAction,
    GripperCommandResult,
    GripperCommandFeedback
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse, Empty, Trigger, TriggerResponse
from mycobot_communication.srv import GetAngles, SetAngles, SetAnglesRequest
from rospy import ServiceException

import tf

from pymycobot.mycobot import MyCobot

def acquire_lock(lock_file):
    """Acquire a file lock"""
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    fd = os.open(lock_file, open_mode)

    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except (IOError, OSError):
            time.sleep(0.01)
            current_time = time.time()

    os.close(fd)
    return None

def release_lock(fd):
    """Release a file lock"""
    if fd is not None:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)

class MycobotInterface:
    def __init__(self):
        # Initialize parameters
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = rospy.get_param("~baud", 115200)
        self.vel_rate = rospy.get_param("~vel_rate", 64.0)
        self.min_vel = rospy.get_param("~min_vel", 5)

        # Create lock file path
        self.lock_file = f"/tmp/mycobot_lock_{self.port.split('/')[-1]}"

        # Connect to MyCobot
        rospy.loginfo(f"Connecting to MyCobot on {self.port}, {self.baud}")
        self.mc = MyCobot(self.port, self.baud)
        self.mc.set_color(255, 0, 0)

        # Initialize state variables
        self.real_angles = None
        self._gripper_state = None # for publish
        self.gripper_state = None # for command
        self.servo_on = True
        self.gripper_is_moving = False
        self.gripper_value = None
        self.gripper_velocity = 100

        self.get_angles = rospy.get_param("~get_angles", True)
        self.get_gripper = rospy.get_param("~get_gripper", False)
        self.set_angles = rospy.get_param("~set_angles", True)
        self.set_gripper = rospy.get_param("~set_gripper", False)
        self.get_atom_button = rospy.get_param("~get_atom_button", False)

        # Setup publishers and subscribers
        self._setup_ros_interface()

        # Initialize MyCobot
        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                self.mc.set_fresh_mode(1)
                self.mc.set_color(255, 0, 255)
            finally:
                release_lock(lock_fd)

    def _setup_ros_interface(self):
        """Setup all ROS publishers, subscribers, services and action servers"""
        # Publishers
        self.joint_angle_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.atom_button_pub = rospy.Publisher("atom_button", Bool, queue_size=1)
        self.gripper_state_pub = rospy.Publisher("gripper_state", JointState, queue_size=1)

        if rospy.get_param("~pub_end_coord", False):
            self.end_coord_pub = rospy.Publisher("end_coord", PoseStamped, queue_size=1)

        # Subscribers
        self.joint_command_sub = rospy.Subscriber("joint_command", JointState, self.joint_command_cb, queue_size=1)
        self.gripper_command_sub = rospy.Subscriber("gripper_command", JointState, self.gripper_command_cb, queue_size=1)

        # Services
        self.set_servo_srv = rospy.Service("set_servo", SetBool, self.set_servo_cb)
        self.get_servo_srv = rospy.Service("get_servo", Trigger, self.get_servo_cb)
        self.gripper_servo_srv = rospy.Service("set_gripper_servo", SetBool, self.gripper_servo_cb)
        self.open_gripper_srv = rospy.Service("open_gripper", Empty, self.open_gripper_cb)
        self.close_gripper_srv = rospy.Service("close_gripper", Empty, self.close_gripper_cb)
        # waiting util server `get_joint_angles` enable.等待'get_joint_angles'服务启用

        rospy.loginfo("wait service")
        rospy.wait_for_service("get_joint_angles")
        self.get_angles_srv = rospy.ServiceProxy("get_joint_angles", GetAngles)

        # Action servers
        self.joint_as = actionlib.SimpleActionServer(
            "arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.joint_as_cb
        )
        self.joint_as.start()

        self.gripper_as = actionlib.SimpleActionServer(
            "gripper_controller/gripper_command",
            GripperCommandAction,
            execute_cb=self.gripper_as_cb
        )
        self.gripper_as.start()

    def _validate_trajectory(self, goal):
        """Validate the trajectory goal"""
        if not self.real_angles:
            self._abort_goal("Real joint angles are empty!", FollowJointTrajectoryResult.INVALID_JOINTS)
            return False

        if len(self.real_angles) != len(goal.trajectory.joint_names):
            self._abort_goal(
                "Incoming trajectory joints do not match the joints of the controller",
                FollowJointTrajectoryResult.INVALID_JOINTS
            )
            return False

        if not goal.trajectory.points:
            self._abort_goal("Incoming trajectory is empty", FollowJointTrajectoryResult.INVALID_GOAL)
            return False

        if not goal.trajectory.points[0].positions:
            self._abort_goal("First point of trajectory has no positions", FollowJointTrajectoryResult.INVALID_GOAL)
            return False

        return True

    def _abort_goal(self, message, error_code):
        """Abort the current goal with the given message and error code"""
        rospy.logerr(message)
        result = FollowJointTrajectoryResult()
        result.error_code = error_code
        self.joint_as.set_aborted(result, message)

    def _process_trajectory(self, goal):
        """Process the trajectory into segments"""
        points = goal.trajectory.points
        trajectory = []

        # Calculate durations
        durations = [points[0].time_from_start]
        for i in range(1, len(points)):
            durations.append(points[i].time_from_start - points[i-1].time_from_start)

        # Calculate start time
        if goal.trajectory.header.stamp == rospy.Time():
            start_time = rospy.Time.now() + rospy.Duration(0.01)
        else:
            start_time = goal.trajectory.header.stamp

        # Process each point
        for i, point in enumerate(points):
            segment = {
                'start_time': start_time + point.time_from_start - durations[i],
                'end_time': start_time + point.time_from_start,
                'positions': point.positions
            }

            if point.velocities:
                if len(point.velocities) != len(goal.trajectory.joint_names):
                    self._abort_goal(
                        f"Command point {i+1} has wrong amount of velocities",
                        FollowJointTrajectoryResult.INVALID_GOAL
                    )
                    return None
                segment['velocities'] = point.velocities

            if len(point.positions) != len(goal.trajectory.joint_names):
                self._abort_goal(
                    f"Command point {i+1} has wrong amount of positions",
                    FollowJointTrajectoryResult.INVALID_GOAL
                )
                return None

            trajectory.append(segment)

        return trajectory

    def _execute_trajectory_segment(self, segment, goal):
        """Execute a single trajectory segment"""
        target_angles = np.array(segment['positions']) * 180 / np.pi
        actual_angles = np.array(self.real_angles)

        if 'velocities' in segment:
            vel = int(np.max(np.abs(segment['velocities'])) * self.vel_rate)
            vel = max(vel, self.min_vel)
        else:
            duration = (segment['end_time'] - segment['start_time']).to_sec()
            vel = int(np.max(np.abs(target_angles - actual_angles)) / duration)

        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                self.mc.send_angles(target_angles.tolist(), vel)
                time.sleep(0.08)
            finally:
                release_lock(lock_fd)

        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = goal.trajectory.joint_names

        rate = rospy.Rate(5)
        while rospy.Time.now() < segment['end_time']:
            if self.joint_as.is_preempt_requested():
                self.joint_as.set_preempted()
                return False

            actual_angles = np.array(self.real_angles)
            feedback.header.stamp = rospy.Time.now()
            feedback.desired.positions = (target_angles / 180 * np.pi).tolist()
            feedback.actual.positions = (actual_angles / 180 * np.pi).tolist()
            feedback.error.positions = ((target_angles - actual_angles) / 180 * np.pi).tolist()
            self.joint_as.publish_feedback(feedback)

            rate.sleep()

        return True

    def _check_goal_constraints(self, goal):
        """Check if the final position meets the goal constraints"""
        target_angles = np.array(goal.trajectory.points[-1].positions) * 180 / np.pi
        actual_angles = np.array(self.real_angles)

        for tolerance in goal.goal_tolerance:
            index = goal.trajectory.joint_names.index(tolerance.name)
            pos_error = abs(target_angles[index] - actual_angles[index]) * np.pi / 180

            if tolerance.position > 0 and pos_error > tolerance.position:
                self._abort_goal(
                    f"Joint {tolerance.name} ended outside goal constraints: {pos_error} > {tolerance.position}",
                    FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                )
                return False

        return True

    def run(self):
        rate = rospy.Rate(rospy.get_param("~joint_state_rate", 5))

        while not rospy.is_shutdown():
            if self.get_angles:
                res = self.get_angles_srv()
                if res.joint_1 == res.joint_2 == res.joint_3 == 0.0:
                    continue
                self.real_angles = [
                    res.joint_1 * (math.pi / 180),
                    res.joint_2 * (math.pi / 180),
                    res.joint_3 * (math.pi / 180),
                    res.joint_4 * (math.pi / 180),
                    res.joint_5 * (math.pi / 180),
                    res.joint_6 * (math.pi / 180),
                ]
                self._publish_joint_states()

            if self.get_gripper:
                lock_fd = acquire_lock(self.lock_file)
                if lock_fd is not None:
                    try:
                        gripper_state = [self.mc.get_gripper_value()]
                    finally:
                        release_lock(lock_fd)

                if gripper_state[0] > 0:
                    self._gripper_state = gripper_state
                    self._publish_gripper_state()

            if self.get_atom_button:
                lock_fd = acquire_lock(self.lock_file)
                if lock_fd is not None:
                    try:
                        button_state = Bool(data=not self.mc.get_digital_input(39))
                    finally:
                        release_lock(lock_fd)

                    self.atom_button_pub.publish(button_state)

            rate.sleep()

    def _publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.name = [f'joint{i+1}' for i in range(6)]
        msg.position = self.real_angles
        self.joint_angle_pub.publish(msg)

    def _publish_gripper_state(self):
        """Publish current gripper state"""
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.position = self._gripper_state
        self.gripper_state_pub.publish(msg)

    def gripper_command_cb(self, msg):
        gripper_state = msg.position
        if gripper_state:
            self.gripper_state = gripper_state
            lock_fd = acquire_lock(self.lock_file)
            if lock_fd is not None:
                try:
                    # rospy.loginfo(f"gripper_state:{self.gripper_state}")
                    # rospy.loginfo(f"_gripper_state:{self._gripper_state}")
                    self.mc.set_gripper_value(int(self.gripper_state[0]), self.gripper_velocity)
                finally:
                    release_lock(lock_fd)

    def joint_command_cb(self, msg):
        """Handle joint command messages"""
        angles = [0]*6
        vel = 100  # deg/s
        for n, p in zip(msg.name, msg.position):
            if 'joint' in n:
                idx = int(n[-1]) - 1
                if 0 <= idx < 6 and abs(p) < (170.0 * math.pi / 180):
                    angles[idx] = round(p * 180 / math.pi, 3)
                else:
                    rospy.logwarn(f"{n} exceeds the limit: {p}")
        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                self.mc.send_angles(angles, vel)
                rospy.loginfo(f"angles: {angles}")
            finally:
                release_lock(lock_fd)


    def joint_as_cb(self, goal):
        """Handle joint trajectory action goals"""
        if not self._validate_trajectory(goal):
            return

        trajectory = self._process_trajectory(goal)
        if trajectory is None:
            return

        # Wait for start time
        start_time = goal.trajectory.header.stamp
        if start_time != rospy.Time():
            wait_duration = (start_time - rospy.Time.now()).to_sec()
            if wait_duration > 0:
                rospy.sleep(wait_duration)

        for segment in trajectory:
            if not self._execute_trajectory_segment(segment, goal):
                return

        if not self._check_goal_constraints(goal):
            return

        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.joint_as.set_succeeded(result, "Trajectory execution completed successfully")

    def gripper_as_cb(self, goal):
        """Handle gripper command action goals"""
        goal_state = int(goal.command.position)

        if goal_state not in (0, 1):
            self.gripper_as.set_aborted(
                GripperCommandResult(
                    position=self.gripper_value,
                    stalled=True,
                    reached_goal=False
                ),
                "Only support 0 (open) or 1 (close)"
            )
            return

        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                self.mc.set_gripper_value(98 if goal_state == 0 else 35, 50)
            finally:
                release_lock(lock_fd)

        while not rospy.is_shutdown() and self.mc.is_gripper_moving():
            if self.gripper_as.is_preempt_requested():
                self.gripper_as.set_preempted()
                return

            lock_fd = acquire_lock(self.lock_file)
            if lock_fd is not None:
                try:
                    position = self.mc.get_gripper_value()
                finally:
                    release_lock(lock_fd)

                self.gripper_as.publish_feedback(
                    GripperCommandFeedback(
                        position=position,
                        stalled=False
                    )
                )

        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                final_position = self.mc.get_gripper_value()
            finally:
                release_lock(lock_fd)

        self.gripper_as.set_succeeded(
            GripperCommandResult(
                position=final_position,
                stalled=True,
                reached_goal=True
            ),
            "Gripper action completed"
        )


    def set_servo_cb(self, req):
        """Handle servo state service calls"""
        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                if req.data:
                    self.mc.send_angles(self.real_angles, 0)
                    self.servo_on = True
                else:
                    self.mc.release_all_servos()
                    self.servo_on = False
            finally:
                release_lock(lock_fd)
        return SetBoolResponse(True, "")


    def get_servo_cb(self, req):
        """Handle servo state query service calls"""
        return TriggerResponse(True, str(self.servo_on))

    def gripper_servo_cb(self, req):
        """Handle gripper servo state service calls"""
        try:
            if req.data:
                if self.gripper_state is not None:
                    lock_fd = acquire_lock(self.lock_file)
                    if lock_fd is not None:
                        try:
                            if self.gripper_state:
                                self.mc.set_gripper_value(self.gripper_state[0], self.gripper_velocity)
                            else:
                                self.gripper_state = [self.mc.get_gripper_value()]
                                self.mc.set_gripper_value(self.gripper_state[0], self.gripper_velocity)
                        finally:
                            release_lock(lock_fd)

                self.gripper_servo_on = True
                rospy.loginfo("Gripper servo turned ON")
                return SetBoolResponse(True, "Gripper servo turned ON")
            else:
                lock_fd = acquire_lock(self.lock_file)
                if lock_fd is not None:
                    try:
                        self.mc.release_servo(7)
                    finally:
                        release_lock(lock_fd)
                self.gripper_servo_on = False
                rospy.loginfo("Gripper servo turned OFF")
                return SetBoolResponse(True, "Gripper servo turned OFF")
        except Exception as e:
            rospy.logerr(f"Error in gripper servo control: {e}")
            return SetBoolResponse(False, str(e))

    def open_gripper_cb(self, _):
        """Handle open gripper service calls"""
        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                self.mc.set_gripper_value(98, 50)
            finally:
                release_lock(lock_fd)
        return Empty()

    def close_gripper_cb(self, _):
        """Handle close gripper service calls"""
        lock_fd = acquire_lock(self.lock_file)
        if lock_fd is not None:
            try:
                self.mc.set_gripper_value(35, 50)
            finally:
                release_lock(lock_fd)
        return Empty()


if __name__ == "__main__":
    rospy.init_node("mycobot_topics")
    interface = MycobotInterface()
    interface.run()
