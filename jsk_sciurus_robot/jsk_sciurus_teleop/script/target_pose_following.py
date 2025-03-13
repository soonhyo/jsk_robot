#!/usr/bin/env python
import rospy
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import threading

class Sciurus17FastIK:
    def __init__(self):
        self.group = MoveGroupCommander("r_arm_group")
        self.robot = RobotCommander()
        self.pub = rospy.Publisher("/sciurus17/controller1/right_arm_controller/command", JointTrajectory, queue_size=1)
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        rospy.Subscriber("/target_pose", PoseStamped, self.target_pose_callback, queue_size=1)
        self.current_joint_state = None
        rospy.Subscriber("/sciurus17/controller1/joint_states", JointState, self.joint_state_callback, queue_size=1)
        self.joint_names = self.group.get_active_joints()
        self.lock = threading.Lock()
        self.last_solution = None
        rospy.loginfo("Sciurus17 Fast IK Solver 초기화 완료")

    def joint_state_callback(self, joint_state_msg):
        self.current_joint_state = joint_state_msg

    def target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_target_pose, args=(pose_msg.pose,)).start()

    def process_target_pose(self, target_pose):
        start_time = rospy.Time.now()
        joint_positions = self.solve_ik_fast(target_pose)
        if joint_positions:
            traj = self.create_simple_trajectory(joint_positions)
            self.send_trajectory(traj)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"IK 처리 시간: {process_time:.4f}초")

    def solve_ik_fast(self, target_pose):
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "r_arm_group"
        ik_request.ik_request.pose_stamped.header.frame_id = self.robot.get_planning_frame()
        ik_request.ik_request.pose_stamped.pose = target_pose
        ik_request.ik_request.timeout = rospy.Duration(0.005)

        if self.current_joint_state:
            robot_state = self.current_joint_state
            if self.last_solution:
                robot_state.position = self.last_solution
            ik_request.ik_request.robot_state.joint_state = robot_state

        try:
            response = self.ik_service(ik_request)
            if response.error_code.val == 1:
                solution = [response.solution.joint_state.position[
                    response.solution.joint_state.name.index(name)] for name in self.joint_names]
                self.last_solution = solution
                return solution
            return None
        except:
            return None

    def create_simple_trajectory(self, target_joint_positions):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = target_joint_positions
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        return traj

    def send_trajectory(self, traj):
        with self.lock:
            traj.header.stamp = rospy.Time.now()
            self.pub.publish(traj)

if __name__ == "__main__":
    rospy.init_node("sciurus17_fast_ik_node")
    ik_solver = Sciurus17FastIK()
    rospy.loginfo("Sciurus17 Fast IK Solver 실행 중...")
    rospy.spin()
