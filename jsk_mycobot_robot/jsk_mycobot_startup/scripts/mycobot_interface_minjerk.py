#!/usr/bin/env python3
import time
import os
import sys
import signal
import threading
import math
from itertools import zip_longest
import numpy as np

from pymycobot.mycobot import MyCobot

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback, GripperCommandAction, GripperCommandResult, GripperCommandFeedback
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse, Empty, Trigger, TriggerResponse
import tf

def minjerk_interpolate(start, end, duration, num_points):
    """
    Perform minimum jerk interpolation between start and end points.

    :param start: Start position
    :param end: End position
    :param duration: Total duration of the movement
    :param num_points: Number of points to interpolate
    :return: Array of interpolated points
    """
    trajectory = []
    for i in range(num_points):
        t = i / (num_points - 1)
        if isinstance(start, (int, float)):
            point = start + (end - start) * (10 * t**3 - 15 * t**4 + 6 * t**5)
        else:
            point = [s + (e - s) * (10 * t**3 - 15 * t**4 + 6 * t**5) for s, e in zip(start, end)]
        trajectory.append(point)
    return trajectory

class MycobotInterface(object):

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        self.model = rospy.get_param("~model", '280')
        self.vel_rate = rospy.get_param("~vel_rate", 10.0) # bit/rad
        self.min_vel = rospy.get_param("~min_vel", 1) # bit, for the bad velocity tracking of mycobot.
        # self.vel_rate = rospy.get_param("~vel_rate", 32.0) # bit/rad
        # self.min_vel = rospy.get_param("~min_vel", 10) # bit, for the bad velocity tracking of mycobot.

        rospy.loginfo("Connect mycobot on %s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.lock = threading.Lock()

        self.joint_angle_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
        self.real_angles = None
        self.joint_command_sub = rospy.Subscriber("joint_command", JointState, self.joint_command_cb)

        self.pub_end_coord = rospy.get_param("~pub_end_coord", False)
        if self.pub_end_coord:
            self.end_coord_pub = rospy.Publisher("end_coord", PoseStamped, queue_size=5)

        self.atom_button_pub = rospy.Publisher("atom_button", Bool, queue_size=5)

        self.set_servo_srv = rospy.Service("set_servo", SetBool, self.set_servo_cb)
        self.get_servo_srv = rospy.Service("get_servo", Trigger, self.get_servo_cb)

        self.open_gripper_srv = rospy.Service("open_gripper", Empty, self.open_gripper_cb)
        self.close_gripper_srv = rospy.Service("close_gripper", Empty, self.close_gripper_cb)
        self.gripper_is_moving = False
        self.gripper_value = None
        self.get_gripper_state = False

        # action server for joint and gripper
        self.joint_as = actionlib.SimpleActionServer("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.joint_as_cb)
        self.joint_as.start()

        self.get_joint_state = True

        self.gripper_as = actionlib.SimpleActionServer("gripper_controller/gripper_command", GripperCommandAction, execute_cb=self.gripper_as_cb)
        self.gripper_as.start()

        self.get_atom_button = False

        self.servo_on = True
        self.current_goal = None
        self.new_goal_received = False

    def run(self):

        r = rospy.Rate(rospy.get_param("~joint_state_rate", 20)) # hz

        while not rospy.is_shutdown():

            # get real joint from MyCobot
            if self.get_joint_state:
                # real_angles = self.mc.get_angles()

                # the duration is over the sending loop, not good
                self.lock.acquire()
                real_angles = []
                _start = time.time()
                while len(real_angles) != 6:
                    real_angles = self.mc.get_angles()
                    _end = time.time()
                    if (_end - _start) > 1:
                        break
                self.lock.release()

                if len(real_angles) != 6: # we assume mycobot only have 6 DoF of joints
                    rospy.logwarn("empty joint angles!!!")
                else:
                    self.real_angles = real_angles

            if self.real_angles:
                rospy.logdebug_throttle(1.0, "get real angles from mycobot")

                msg = JointState()
                msg.header.stamp = rospy.get_rostime()

                for i, ang in enumerate(self.real_angles):
                   msg.name.append('joint' + str(i+1))
                   msg.position.append(ang / 180.0 * math.pi)
                self.joint_angle_pub.publish(msg)

            if self.get_atom_button:
                msg = Bool()
                msg.data = False if self.mc.get_digital_input(39) else True
                self.atom_button_pub.publish(msg)
            # get gripper state
            # Note: we only retreive the gripper state when doing the grasp action.
            # We find following polling function will cause the failure of get_angles() for Mycobot Pro 320.
            # This makes the publish of joint state decrease from 20Hz to 2Hz for MyCobot Pro 320.
            if self.get_gripper_state:
                self.gripper_is_moving = self.mc.is_gripper_moving()
                self.gripper_value = self.mc.get_gripper_value()


            # get end-effector if necessary (use tf by ros in default)
            if self.pub_end_coord:
                coords = self.mc.get_coords()
                if coords:
                    msg = PoseStamped
                    msg.header.stamp = rospy.get_rostime()
                    msg.pose.position.x = coords[0]
                    msg.pose.position.y = coords[1]
                    msg.pose.position.z = coords[2]
                    q = tf.transformations.quaternion_from_euler(coords[3], coords[4], coords[5])
                    msg.poseq.quaternion.x = q[0]
                    msg.poseq.quaternion.y = q[1]
                    msg.poseq.quaternion.z = q[2]
                    msg.poseq.quaternion.w = q[3]
                    self.end_coord_pub.publish(msg)

            r.sleep()

    def joint_command_cb(self, msg):
        angles = self.real_angles
        vel = 50 # deg/s, hard-coding
        # vel = 30 # deg/s, hard-coding

        for n, p, v in zip_longest(msg.name, msg.position, msg.velocity):
            id = int(n[-1]) - 1
            if 'joint' in n and id >= 0 and id < len(angles):
                if math.fabs(p) < 170.0 / 180 * math.pi: # 190 should be  retrieved from API
                    angles[id] = p * 180 / math.pi
                else:
                    rospy.logwarn("%s exceeds the limit, %f", n, p)
            if v:
                v = v * 180 / math.pi
                if v < vel:
                    vel = v

        self.lock.acquire()
        self.mc.send_angles(angles, vel)
        self.lock.release()

    def set_servo_cb(self, req):
        if req.data:
            self.lock.acquire()
            self.mc.send_angles(self.real_angles, 0)
            self.lock.release()
            self.servo_on = True
            rospy.loginfo("servo on")
        else:
            self.lock.acquire()
            self.mc.release_all_servos()
            self.lock.release()
            self.servo_on = False
            rospy.loginfo("servo off")
        return SetBoolResponse(True, "")

    def get_servo_cb(self, req):
        return TriggerResponse(True, str(self.servo_on))

    def open_gripper_cb(self, req):
        self.lock.acquire()
        # self.mc.set_gripper_state(0, 0)
        self.mc.set_gripper_value(98, 0) # first arg is the flag 0 - open, 1 - close; second arg is the speed
        self.lock.release()
        rospy.loginfo("open gripper")

    def close_gripper_cb(self, req):
        self.lock.acquire()
        # self.mc.set_gripper_state(1, 0)
        self.mc.set_gripper_value(15, 0) # first arg is the flag 0 - open, 1 - close; second arg is the speed
        self.lock.release()
        rospy.loginfo("close gripper")

    def joint_as_cb(self, goal):
        if '320' in self.model:
            self.get_joint_state = False

        if self.current_goal is not None:
            # If there's an ongoing execution, preempt it
            self.new_goal_received = True
            self.current_goal = goal
            return

        self.current_goal = goal
        self.new_goal_received = False

        while self.current_goal is not None:
            result = self.execute_trajectory(self.current_goal)
            if result is not None:
                self.joint_as.set_succeeded(result)

            if self.new_goal_received:
                self.new_goal_received = False
            else:
                self.current_goal = None

        self.get_joint_state = True

    def execute_trajectory(self, goal):
        if not self.real_angles:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            msg = "Real joint angles are empty!"
            rospy.logerr(msg)
            return res

        if len(self.real_angles) != len(goal.trajectory.joint_names):
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            msg = "Incoming trajectory joints do not match the joints of the controller"
            rospy.logerr(msg)
            return res

        if len(goal.trajectory.points) == 0:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            msg = "Incoming trajectory is empty"
            rospy.logerr(msg)
            return res

        current_point = JointTrajectoryPoint()
        current_point.positions = [angle * np.pi / 180 for angle in self.real_angles]
        current_point.time_from_start = rospy.Duration(0.0)

        points = [current_point] + goal.trajectory.points

        if not points[0].positions:
            msg = "First point of trajectory has no positions"
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            rospy.logerr(msg)
            return res

        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)

        for i in range(len(points) - 1):
            start_time = time + points[i].time_from_start
            end_time = time + points[i+1].time_from_start
            duration = (end_time - start_time).to_sec()

            start_positions = np.array(points[i].positions) * 180 / np.pi
            end_positions = np.array(points[i+1].positions) * 180 / np.pi

            #num_interpolation_points = max(int(duration * 10), 2)  # At least 10 points per second
            num_interpolation_points = 2
            interpolated_trajectory = minjerk_interpolate(start_positions, end_positions, duration, num_interpolation_points)

            for j, pos in enumerate(interpolated_trajectory):
                seg = {
                    'start_time': start_time + rospy.Duration(j * duration / (num_interpolation_points - 1)),
                    'end_time': start_time + rospy.Duration((j + 1) * duration / (num_interpolation_points - 1)),
                    'positions': pos
                }
                trajectory.append(seg)

        rospy.loginfo("Trajectory start requested at %.3lf, waiting...", goal.trajectory.header.stamp.to_sec())
        r = rospy.Rate(20)
        while (goal.trajectory.header.stamp - time).to_sec() > 0:
            time = rospy.Time.now()
            r.sleep()

        total_duration = (trajectory[-1]['end_time'] - trajectory[0]['start_time']).to_sec()
        rospy.loginfo("Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf",
                      time.to_sec(), time.to_sec() + total_duration, total_duration)

        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = goal.trajectory.joint_names
        feedback.header.stamp = time

        for i, seg in enumerate(trajectory):
            if self.new_goal_received:
                return None  # Signal that we need to switch to the new goal

            target_angles = np.array(seg['positions']).round(decimals=2)
            actual_angles = np.array(self.real_angles)

            rospy.loginfo(f"sec:{(seg['end_time'] - seg['start_time']).to_sec()}")
            vel = int(np.max(np.abs(target_angles - actual_angles)) / (seg['end_time'] - seg['start_time']).to_sec() * self.vel_rate / len(trajectory))

            if vel < self.min_vel:
                vel = self.min_vel

            self.lock.acquire()
            self.mc.send_angles(target_angles.tolist(), vel)
            self.lock.release()

            if not self.get_joint_state:
                self.real_angles = target_angles.tolist()

            while time.to_sec() < seg["end_time"].to_sec():
                if self.new_goal_received:
                    return None  # Signal that we need to switch to the new goal

                if (time - feedback.header.stamp).to_sec() > 0.1:
                    feedback.header.stamp = time
                    feedback.desired.positions = (target_angles / 180 * np.pi).tolist()
                    feedback.actual.positions = (actual_angles / 180 * np.pi).tolist()
                    feedback.error.positions = ((target_angles - actual_angles) / 180 * np.pi).tolist()
                    self.joint_as.publish_feedback(feedback)

                r.sleep()
                time = rospy.Time.now()

            for tol in goal.path_tolerance:
                index = goal.trajectory.joint_names.index(tol.name)
                pos_err = np.fabs(target_angles - actual_angles)[index] / 180 * np.pi

                if tol.position > 0 and pos_err > tol.position:
                    msg = f"Unsatisfied position tolerance for {tol.name}, trajectory point {i+1}, {pos_err} is larger than {tol.position}"
                    rospy.logwarn(msg)
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    return res

        if not self.get_joint_state:
            actual_angles = np.array(self.mc.get_angles())
        for tol in goal.goal_tolerance:
            index = goal.trajectory.joint_names.index(tol.name)
            pos_err = np.fabs(target_angles - actual_angles)[index] / 180 * np.pi
            if tol.position > 0 and pos_err > tol.position:
                msg = f"Aborting because {tol.name} wound up outside the goal constraints, {pos_err} is larger than {tol.position}"
                rospy.logwarn(msg)
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                return res

        msg = "Trajectory execution successfully completed"
        rospy.loginfo(msg)
        res = FollowJointTrajectoryResult()
        res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        return res



    def gripper_as_cb(self, goal):

        goal_state = (int)(goal.command.position)
        if not (goal_state == 0 or goal_state == 1):
            res = GripperCommandResult()
            res.position = self.gripper_value
            res.stalled = True
            res.reached_goal = False
            msg = "We only support 1 (totally close) or 0 (totally open) for gripper action"
            rospy.logerr(msg);
            self.gripper_as.set_aborted(res, msg)
            return

        feedback = GripperCommandFeedback()

        self.get_gripper_state = True # start retrive the grasping data
        if '320' in self.model: # workaround for mycobot pro 320
            self.get_joint_state = False # stop polling joint angles
            time.sleep(0.1) # wait for the finish of last joint angles polling

        self.lock.acquire()
        # self.mc.set_gripper_state(goal_state, 0) # first arg is the flag 0 - open, 1 - close; second arg is the speed
        if goal_state == 0:
            self.mc.set_gripper_value(98, 0) # first arg is the flag 0 - open, 1 - close; second arg is the speed
        else:
            self.mc.set_gripper_value(15, 0) # first arg is the flag 0 - open, 1 - close; second arg is the speed

        self.lock.release()

        self.get_joint_state = True # resume polling joint state if necessary

        t = rospy.Time(0)
        rospy.sleep(0.3) # wait for the gripper to start moving

        r = rospy.Rate(5) # 20 Hz

        while not rospy.is_shutdown():

            rospy.logdebug("Current gripper value is  %d state is %d", self.gripper_value, self.gripper_is_moving);

            if self.gripper_as.is_preempt_requested():

                self.gripper_as.set_preempted()
                self.get_gripper_state = False
                return;

            if (rospy.Time.now() - t).to_sec() > 0.1: # 10 Hz
                feedback.position = self.gripper_value
                feedback.stalled = False
                feedback.stalled = False
                self.gripper_as.publish_feedback(feedback);
                t = rospy.Time.now()

            if self.gripper_is_moving == 0: # not moving
                self.get_gripper_state = False
                msg = "Gripper stops moving"
                rospy.loginfo(msg)
                res = GripperCommandResult()
                res.position = self.gripper_value
                res.stalled = True
                res.reached_goal = True
                self.gripper_as.set_succeeded(res, msg);
                break


            r.sleep();

if __name__ == "__main__":
    rospy.init_node("mycobot_topics")
    mc_inteface = MycobotInterface()
    mc_inteface.run()
    pass
