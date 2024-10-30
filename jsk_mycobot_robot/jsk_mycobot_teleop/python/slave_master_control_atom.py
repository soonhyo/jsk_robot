#!/usr/bin/env python3
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
        self.control_rate = rospy.get_param('~control_rate', 10)  # Hz

        # Initialize state
        self.gripper_state = False  # False: open, True: closed
        self.last_button_state = False
        self.gripper_action_state = None

        # Setup ROS interface
        self.setup_ros_interface()

        # Initialize left arm servo off
        self.initialize_left_arm()

        rospy.loginfo("MyCobot Relay node initialized")

    def setup_ros_interface(self):
        """Setup ROS publishers, subscribers and service clients"""
        # Publisher for right arm commands
        self.joint_cmd_pub = rospy.Publisher(
            '/rarm/joint_command',
            JointState,
            queue_size=10
        )

        # Subscriber for left arm joint states
        self.joint_state_sub = rospy.Subscriber(
            '/larm/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=10
        )

        # Subscriber for atom button
        self.atom_button_sub = rospy.Subscriber(
            '/larm/atom_button',
            Bool,
            self.atom_button_callback,
            queue_size=5
        )

        # Setup gripper action client
        self.gripper_client = actionlib.SimpleActionClient(
            'rarm/gripper_controller/gripper_command',
            GripperCommandAction
        )

        # Wait for gripper action server
        rospy.loginfo("Waiting for gripper action server...")
        if self.gripper_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.loginfo("Gripper action server connected")
        else:
            rospy.logwarn("Failed to connect to gripper action server")

    def initialize_left_arm(self):
        """Initialize left arm with servo off"""
        try:
            # Wait for the service to become available
            rospy.wait_for_service('/larm/set_servo')

            # Create service proxy
            set_servo = rospy.ServiceProxy('/larm/set_servo', SetBool)

            # Call service to turn servo off
            req = SetBoolRequest(data=False)
            response = set_servo(req)

            if response.success:
                rospy.loginfo("Successfully turned off left arm servo")
            else:
                rospy.logwarn("Failed to turn off left arm servo")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def gripper_done_cb(self, state, result):
        """Callback for when gripper action is complete"""
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Gripper action succeeded")
            self.gripper_action_state = 'succeeded'
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Gripper action preempted")
            self.gripper_action_state = 'preempted'
        else:
            rospy.logwarn(f"Gripper action failed with state: {state}")
            self.gripper_action_state = 'failed'

    def gripper_feedback_cb(self, feedback):
        """Callback for gripper action feedback"""
        rospy.logdebug(f"Gripper position: {feedback.position}, stalled: {feedback.stalled}")

    def send_gripper_command(self, close_gripper):
        """Send gripper command using action client"""
        try:
            # Cancel any existing goals
            self.gripper_client.cancel_all_goals()

            # Create and send new goal
            goal = GripperCommandGoal()
            goal.command.position = 1.0 if close_gripper else 0.0
            goal.command.max_effort = -1.0  # Use default effort

            # Send goal with callbacks
            self.gripper_action_state = 'sending'
            self.gripper_client.send_goal(
                goal,
                done_cb=self.gripper_done_cb,
                feedback_cb=self.gripper_feedback_cb
            )

            rospy.loginfo(f"Sent gripper {'close' if close_gripper else 'open'} command")

        except Exception as e:
            rospy.logerr(f"Failed to send gripper command: {e}")

    def atom_button_callback(self, msg):
        """Handle atom button state changes"""
        try:
            current_button_state = msg.data

            # Button state changed from False to True (button press)
            if current_button_state and not self.last_button_state:
                # Toggle gripper state
                self.gripper_state = not self.gripper_state

                # Send gripper command
                self.send_gripper_command(self.gripper_state)

            self.last_button_state = current_button_state

        except Exception as e:
            rospy.logerr(f"Error in atom button callback: {e}")

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
            # Apply joint limits
            limited_angles = self.limit_joint_angles(msg.position)

            # Create command message for right arm
            cmd_msg = JointState()
            cmd_msg.header.stamp = rospy.Time.now()

            cmd_msg.name = [f"joint{i+1}" for i in range(len(limited_angles))]

            cmd_msg.position = limited_angles

            # Copy velocities if they exist in the original message
            if msg.velocity and len(msg.velocity) == len(limited_angles):
                cmd_msg.velocity = msg.velocity

            # Copy efforts if they exist in the original message
            if msg.effort and len(msg.effort) == len(limited_angles):
                cmd_msg.effort = msg.effort

            # Publish command
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
