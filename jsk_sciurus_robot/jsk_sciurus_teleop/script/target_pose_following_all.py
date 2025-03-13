#!/usr/bin/env python
import rospy
import numpy as np
import time
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)
import threading
import tf.transformations as tf_trans  # Quaternion 변환을 위해
import actionlib

CONTROL_R = 0
CONTROL_L = 1

MAX_GRIPPER_ANGLE = 0.8
MIN_GRIPPER_ANGLE = 0.0

class Sciurus17FastIK:
    def __init__(self):
        # 팔 그룹 초기화 (IK 사용)
        self.r_arm_group = MoveGroupCommander("r_arm_group")
        self.l_arm_group = MoveGroupCommander("l_arm_group")
        # 그리퍼 액션 클라이언트 초기화
        self._clientR = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
        self._clientL = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)

        self._goalR = GripperCommandGoal()
        self._goalL = GripperCommandGoal()

        # 액션 서버 대기
        self._clientR.wait_for_server(rospy.Duration(5.0))
        if not self._clientR.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper R Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

        self._clientL.wait_for_server(rospy.Duration(5.0))
        if not self._clientL.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper L Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

        self.robot = RobotCommander()

        # 퍼블리셔 설정
        self.r_arm_pub = rospy.Publisher(
            "/sciurus17/controller1/right_arm_controller/command",
            JointTrajectory,
            queue_size=1
        )
        self.l_arm_pub = rospy.Publisher(
            "/sciurus17/controller2/left_arm_controller/command",
            JointTrajectory,
            queue_size=1
        )
        self.neck_pub = rospy.Publisher(
            "/sciurus17/controller3/neck_controller/command",
            JointTrajectory,
            queue_size=1
        )
        self.waist_yaw_pub = rospy.Publisher(
            "/sciurus17/controller3/waist_yaw_controller/command",
            JointTrajectory,
            queue_size=1
        )

        # IK 서비스 (팔에만 사용)
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        # 타겟 포즈 구독
        rospy.Subscriber("/right_wrist_target_pose", PoseStamped, self.r_arm_target_pose_callback, queue_size=1)
        rospy.Subscriber("/left_wrist_target_pose", PoseStamped, self.l_arm_target_pose_callback, queue_size=1)
        rospy.Subscriber("/right_hand", Float32, self.r_hand_target_value_callback, queue_size=1)
        rospy.Subscriber("/left_hand", Float32, self.l_hand_target_value_callback, queue_size=1)
        rospy.Subscriber("/head_target_pose", PoseStamped, self.neck_target_pose_callback, queue_size=1)
        rospy.Subscriber("/waist_target_pose", PoseStamped, self.waist_yaw_target_pose_callback, queue_size=1)
        rospy.Subscriber("/command_pose", String, self.command_pose_callback, queue_size=1)

        # 조인트 상태 구독
        self.current_joint_state = None
        self.joint_state_dict = {}  # 이름-위치 쌍을 저장하는 딕셔너리
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1)

        # 조인트 이름 설정 (MoveIt 그룹이 없으므로 임시로 주석 처리된 상태 유지)
        self.r_arm_joint_names = self.r_arm_group.get_active_joints()
        self.l_arm_joint_names = self.l_arm_group.get_active_joints()
        # self.r_hand_joint_names = self.r_hand_group.get_active_joints()
        # self.l_hand_joint_names = self.l_hand_group.get_active_joints()

        self.neck_joint_names = ["neck_pitch_joint", "neck_yaw_joint"]
        self.waist_yaw_joint_names = ["waist_yaw_joint"]

        self.lock = threading.Lock()
        self.last_solutions = {
            "r_arm": None,
            "l_arm": None
        }
        rospy.loginfo("Sciurus17 Fast IK Solver 초기화 완료")

    def joint_state_callback(self, joint_state_msg):
        self.current_joint_state = joint_state_msg
        self.joint_state_dict = dict(zip(joint_state_msg.name, joint_state_msg.position))

    # 팔 타겟 포즈 콜백
    def r_arm_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_arm_target_pose, args=("r_arm", self.r_arm_group, pose_msg.pose, self.r_arm_pub, self.r_arm_joint_names)).start()

    def l_arm_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_arm_target_pose, args=("l_arm", self.l_arm_group, pose_msg.pose, self.l_arm_pub, self.l_arm_joint_names)).start()

    # 그리퍼 타겟 값 콜백 (스레드로 비동기 처리)
    def r_hand_target_value_callback(self, value_msg):
        threading.Thread(target=self.process_hand_target_value, args=("r_hand", value_msg.data, CONTROL_R)).start()

    def l_hand_target_value_callback(self, value_msg):
        threading.Thread(target=self.process_hand_target_value, args=("l_hand", value_msg.data, CONTROL_L)).start()

    # 네크 타겟 포즈 콜백
    def neck_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_neck_target_pose, args=(pose_msg.pose,)).start()

    # 웨이스트 요 타겟 포즈 콜백
    def waist_yaw_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_waist_yaw_target_pose, args=(pose_msg.pose,)).start()

    def command_pose_callback(self, command_msg):
        if command_msg.data == "init_pose":
            self.process_init_pose()

    # 팔 IK 처리
    def process_arm_target_pose(self, part, group, target_pose, publisher, joint_names):
        start_time = rospy.Time.now()
        joint_positions = self.solve_ik_fast(part, group, target_pose, joint_names)
        if joint_positions:
            traj = self.create_simple_trajectory(joint_positions, joint_names)
            self.send_trajectory(traj, publisher)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"{part} IK 처리 시간: {process_time:.4f}초")

    # 그리퍼 처리 (actionlib 기반, 스레드 내에서 실행)
    def process_hand_target_value(self, part, target_value, control_type):
        start_time = rospy.Time.now()
        _inv_target_value = 1 - target_value
        _inv_target_value = np.interp(_inv_target_value, (0.0, 1.0), (MIN_GRIPPER_ANGLE, MAX_GRIPPER_ANGLE))

        if part == "r_hand":
            self.command(_inv_target_value, effort=0.01, type=control_type)
        elif part == "l_hand": # left hand is negative range
            self.command(-1 * _inv_target_value, effort=0.01, type=control_type)

        # 비동기 실행이므로 wait는 호출하지 않음 (필요 시 별도 호출 가능)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"{part} 처리 시간: {process_time:.4f}초")

    # 네크 처리
    def process_neck_target_pose(self, target_pose):
        start_time = rospy.Time.now()
        quat = target_pose.orientation
        euler = tf_trans.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        pitch = -euler[1]
        yaw = euler[2]
        joint_positions = [pitch, yaw]
        traj = self.create_simple_trajectory(joint_positions, self.neck_joint_names)
        self.send_trajectory(traj, self.neck_pub)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"neck 처리 시간: {process_time:.4f}초")

    # 웨이스트 요 처리
    def process_waist_yaw_target_pose(self, target_pose):
        start_time = rospy.Time.now()
        quat = target_pose.orientation
        euler = tf_trans.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = euler[2]
        joint_positions = [yaw]
        traj = self.create_simple_trajectory(joint_positions, self.waist_yaw_joint_names)
        self.send_trajectory(traj, self.waist_yaw_pub)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"waist_yaw 처리 시간: {process_time:.4f}초")

    def process_init_pose(self):
        # go right arm to init pose
        self.r_arm_group.set_named_target("r_arm_init_pose")
        self.r_arm_group.go(wait=True)
        self.r_arm_group.stop()

        # go right gripper to init pose
        self.command(MAX_GRIPPER_ANGLE, effort=0.01, type=CONTROL_R)
        self.wait(CONTROL_R, timeout=0.5)

        # go left arm to init pose
        self.l_arm_group.set_named_target("l_arm_init_pose")
        self.l_arm_group.go(wait=True)
        self.l_arm_group.stop()

        # go left gripper to init pose
        self.command(-1 * MAX_GRIPPER_ANGLE, effort=0.01, type=CONTROL_L)
        self.wait(CONTROL_L, timeout=0.5)

        # waist
        _joint_positions = [0.0]
        _traj = self.create_simple_trajectory(_joint_positions, self.waist_yaw_joint_names)
        self.send_trajectory(_traj, self.waist_yaw_pub)

        # head
        _joint_positions = [0.0, 0.0]
        _traj = self.create_simple_trajectory(_joint_positions, self.neck_joint_names)
        self.send_trajectory(_traj, self.neck_pub)

        # rospy.loginfo("wait 5 seconds ...")
        # time.sleep(5)
        rospy.loginfo("Moved to init_pose")

    # actionlib를 통한 그리퍼 제어
    def command(self, position, effort, type):
        if type == CONTROL_R:
            self._goalR.command.position = position
            self._goalR.command.max_effort = effort
            self._clientR.send_goal(self._goalR, feedback_cb=self.feedbackR)
        elif type == CONTROL_L:
            self._goalL.command.position = position
            self._goalL.command.max_effort = effort
            self._clientL.send_goal(self._goalL, feedback_cb=self.feedbackL)

    def feedbackR(self, msg):
        rospy.loginfo("Right Gripper Feedback: %s" % msg)

    def feedbackL(self, msg):
        rospy.loginfo("Left Gripper Feedback: %s" % msg)

    def stop(self):
        self._clientR.cancel_goal()
        self._clientL.cancel_goal()

    def wait(self, type, timeout=0.02):
        if type == CONTROL_R:
            self._clientR.wait_for_result(timeout=rospy.Duration(timeout))
            return self._clientR.get_result()
        elif type == CONTROL_L:
            self._clientL.wait_for_result(timeout=rospy.Duration(timeout))
            return self._clientL.get_result()

    def clear(self):
        self._goalR = GripperCommandGoal()
        self._goalL = GripperCommandGoal()

    # IK 풀이 (팔에만 적용)
    def solve_ik_fast(self, part, group, target_pose, joint_names):
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = f"{part}_group"
        ik_request.ik_request.pose_stamped.header.frame_id = self.robot.get_planning_frame()
        ik_request.ik_request.pose_stamped.pose = target_pose
        ik_request.ik_request.timeout = rospy.Duration(0.001)

        if self.current_joint_state:
            robot_state = self.current_joint_state
            if self.last_solutions.get(part):
                robot_state.position = list(robot_state.position)
                for i, name in enumerate(robot_state.name):
                    if name in joint_names and name in [n for n, p in zip(robot_state.name, self.last_solutions[part])]:
                        robot_state.position[i] = self.last_solutions[part][joint_names.index(name)]
            ik_request.ik_request.robot_state.joint_state = robot_state

        try:
            response = self.ik_service(ik_request)
            if response.error_code.val == 1:
                solution = [response.solution.joint_state.position[
                    response.solution.joint_state.name.index(name)] for name in joint_names]
                self.last_solutions[part] = solution
                return solution
            return None
        except:
            return None

    def create_simple_trajectory(self, target_joint_positions, joint_names):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_joint_positions
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        return traj

    def send_trajectory(self, traj, publisher):
        with self.lock:
            traj.header.stamp = rospy.Time.now()
            publisher.publish(traj)

if __name__ == "__main__":
    rospy.init_node("sciurus17_fast_ik_node")
    ik_solver = Sciurus17FastIK()
    rospy.loginfo("Sciurus17 Fast IK Solver 실행 중...")
    rospy.spin()
