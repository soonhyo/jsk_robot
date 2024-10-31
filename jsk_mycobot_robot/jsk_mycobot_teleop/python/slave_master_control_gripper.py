#!/usr/bin/env python3
import time

import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
    GripperCommandResult,
    GripperCommandFeedback
)

# Define joint angle limits in radians
JOINT_LIMITS = {
    1: (-150 * np.pi/180, 150 * np.pi/180),  # Joint 1 angle range
    2: (-120 * np.pi/180, 120 * np.pi/180),  # Joint 2 angle range
    3: (-150 * np.pi/180, 150 * np.pi/180),  # Joint 3 angle range
    4: (-120 * np.pi/180, 120 * np.pi/180),  # Joint 4 angle range
    5: (-150 * np.pi/180, 150 * np.pi/180),  # Joint 5 angle range
    6: (-150 * np.pi/180, 150 * np.pi/180),  # Joint 6 angle range
}

class MyCobotRelay:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('mycobot_relay', anonymous=True)

        # Get parameters
        self.control_rate = rospy.get_param('~control_rate', 15)  # Hz

        # Initialize state
        self.last_gripper_position = None
        self.gripper_action_state = None
        # Initialize left arm servo off
        self.initialize_left_arm()

        # Setup ROS interface
        self.setup_ros_interface()


        rospy.loginfo("MyCobot Relay node initialized")

    def setup_ros_interface(self):
        """Setup ROS publishers, subscribers and action clients"""
        # Publisher for right arm commands
        self.joint_cmd_pub = rospy.Publisher(
            '/slave/joint_command',
            JointState,
            queue_size=10
        )

        # Subscriber for left arm joint states
        self.joint_state_sub = rospy.Subscriber(
            '/master/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=10
        )

        # Subscriber for left arm gripper state
        self.left_gripper_sub = rospy.Subscriber(
            '/master/gripper_state',
            JointState,
            self.left_gripper_state_cb
        )

        # Publisher for right arm gripper command
        self.right_gripper_pub = rospy.Publisher(
            '/slave/gripper_command',
            JointState,
            queue_size=5)

    def initialize_left_arm(self):
        """Initialize left arm with servo off"""
        try:
            rospy.wait_for_service('/master/set_servo')
            set_servo = rospy.ServiceProxy('/master/set_servo', SetBool)
            rospy.wait_for_service('/master/set_gripper_servo')
            set_gripper_servo = rospy.ServiceProxy('/master/set_gripper_servo', SetBool)

            req = SetBoolRequest(data=False)
            response1 = set_servo(req)
            time.sleep(1)
            response2 = set_gripper_servo(req)
            time.sleep(1)

            if response1.success:
                rospy.loginfo("Successfully turned off left arm servo")
            else:
                rospy.logwarn("Failed to turn off left arm servo")

            if response2.success:
                rospy.loginfo("Successfully turned off left gripper servo")
            else:
                rospy.logwarn("Failed to turn off left gripper servo")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def gripper_done_cb(self, state, result):
        """Callback for when right gripper action is complete"""
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Right gripper action succeeded")
            self.gripper_action_state = 'succeeded'
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Right gripper action preempted")
            self.gripper_action_state = 'preempted'
        else:
            rospy.logwarn(f"Right gripper action failed with state: {state}")
            self.gripper_action_state = 'failed'

    def gripper_feedback_cb(self, feedback):
        """Callback for right gripper action feedback"""
        rospy.logdebug(f"Right gripper position: {feedback.position}, stalled: {feedback.stalled}")

    def left_gripper_state_cb(self, msg):
        """Handle incoming left gripper state"""
        try:
            current_position = msg.position

            # Only send command if the position has changed significantly
            if (self.last_gripper_position is None or
                abs(current_position[0] - self.last_gripper_position[0]) > 0.1):

                msg = JointState()
                msg.position = current_position
                # Send goal with callbacks
                self.right_gripper_pub.publish(msg)

                rospy.loginfo(f"Mirroring gripper position: {current_position}")
                self.last_gripper_position = current_position

        except Exception as e:
            rospy.logerr(f"Error in left gripper callback: {e}")

    def clamp(self, value, min_value, max_value):
        """Clamp value between min and max"""
        return max(min_value, min(value, max_value))

    def limit_joint_angles(self, angles):
        """Apply joint limits to angles"""
        limited_angles = []
        for i, angle in enumerate(angles, start=1):
            if i in JOINT_LIMITS:
                min_angle, max_angle = JOINT_LIMITS[i]
                limited_angle = self.clamp(angle, min_angle, max_angle)
                limited_angles.append(limited_angle)
            else:
                limited_angles.append(angle)
        return limited_angles

    def joint_state_callback(self, msg):
        """Handle incoming joint states from left arm and relay to right arm"""
        try:
            limited_angles = self.limit_joint_angles(msg.position)

            cmd_msg = JointState()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.name = [f"joint{i+1}" for i in range(len(limited_angles))]
            cmd_msg.position = limited_angles

            if msg.velocity and len(msg.velocity) == len(limited_angles):
                cmd_msg.velocity = msg.velocity

            if msg.effort and len(msg.effort) == len(limited_angles):
                cmd_msg.effort = msg.effort

            self.joint_cmd_pub.publish(cmd_msg)

        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {e}")

    def run(self):
        """Main control loop"""
        rate = rospy.Rate(self.control_rate)

        rospy.loginfo("Starting MyCobot relay node...")
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    try:
        relay = MyCobotRelay()
        relay.run()
    except rospy.ROSInterruptException:
        pass
