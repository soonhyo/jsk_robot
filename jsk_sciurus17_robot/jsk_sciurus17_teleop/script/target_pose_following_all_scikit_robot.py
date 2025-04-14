#!/usr/bin/env python3

import rospy
import numpy as np
import time
from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import threading
import tf.transformations as tf_trans  # Quaternion 변환을 위해
import actionlib
import skrobot
from copy import deepcopy

CONTROL_R = 0
CONTROL_L = 1
MAX_GRIPPER_ANGLE = 0.8
MIN_GRIPPER_ANGLE = 0.0
class Sciurus17FastIK:
    def __init__(self):
        # scikit-robot 로봇 모델 초기화
        urdf_path = rospy.get_param("~urdf_path", "/home/s-kim-lab/baxter_ws/src/jsk_robot/jsk_sciurus17_robot/sciurus17eus/sciurus17.urdf")  # URDF 경로 설정
        self.robot_model = skrobot.models.urdf.RobotModelFromURDF(urdf_file=urdf_path)
        self.robot_model_copy = deepcopy(self.robot_model)

        self.r_arm_link_list, _ = self.get_link_list(self.robot_model)
        _, self.l_arm_link_list = self.get_link_list(self.robot_model_copy)
        
        # 오른팔과 왼팔의 엔드 이펙터 정의 (URDF에 따라 이름 수정 필요)
        self.r_arm_end_effector = self.robot_model.r_link7  # 예: "r_wrist_link"
        self.l_arm_end_effector = self.robot_model_copy.l_link7  # 예: "l_wrist_link"
        self.r_arm_joints = self.robot_model.joint_list_from_link_list(self.r_arm_link_list)
        self.l_arm_joints = self.robot_model_copy.joint_list_from_link_list(self.l_arm_link_list)
        self.r_arm_joint_names = [j.name for j in self.r_arm_joints]
        self.l_arm_joint_names = [j.name for j in self.l_arm_joints]

        # 그리퍼 액션 클라이언트 초기화 (기존 코드 유지)
        self._clientR = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
        self._clientL = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
        self._goalR = GripperCommandGoal()
        self._goalL = GripperCommandGoal()
        self._clientR.wait_for_server(rospy.Duration(5.0))
        self._clientL.wait_for_server(rospy.Duration(5.0))
        self.clear()

        # 퍼블리셔 설정 (기존 코드 유지)
        self.r_arm_pub = rospy.Publisher("/sciurus17/controller1/right_arm_controller/command", JointTrajectory, queue_size=1)
        self.l_arm_pub = rospy.Publisher("/sciurus17/controller2/left_arm_controller/command", JointTrajectory, queue_size=1)
        self.neck_pub = rospy.Publisher("/sciurus17/controller3/neck_controller/command", JointTrajectory, queue_size=1)
        self.waist_yaw_pub = rospy.Publisher("/sciurus17/controller3/waist_yaw_controller/command", JointTrajectory, queue_size=1)

        # 구독자 설정 (기존 코드 유지)
        rospy.Subscriber("/right_wrist_target_pose", PoseStamped, self.r_arm_target_pose_callback, queue_size=1)
        rospy.Subscriber("/left_wrist_target_pose", PoseStamped, self.l_arm_target_pose_callback, queue_size=1)
        rospy.Subscriber("/right_hand", Float32, self.r_hand_target_value_callback, queue_size=1)
        rospy.Subscriber("/left_hand", Float32, self.l_hand_target_value_callback, queue_size=1)
        rospy.Subscriber("/head_target_pose", PoseStamped, self.neck_target_pose_callback, queue_size=1)
        rospy.Subscriber("/waist_target_pose", PoseStamped, self.waist_yaw_target_pose_callback, queue_size=1)
        rospy.Subscriber("/command_pose", String, self.command_pose_callback, queue_size=1)

        # 조인트 상태 구독
        self.current_joint_state = None
        self.joint_state_dict = {}
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1)

        self.neck_joint_names = ["neck_pitch_joint", "neck_yaw_joint"]
        self.waist_yaw_joint_names = ["waist_yaw_joint"]

        self.lock = threading.Lock()
        self.last_solutions = {"r_arm": None, "l_arm": None}
        rospy.loginfo("Sciurus17 Fast IK Solver 초기화 완료 (scikit-robot)")
    
    def get_link_list(self, robot_model):
        r_arm_link_list = [robot_model.r_link1,
                            robot_model.r_link2,
                            robot_model.r_link3,
                            robot_model.r_link4,
                            robot_model.r_link5,
                            robot_model.r_link6,
                            robot_model.r_link7]

        l_arm_link_list = [robot_model.l_link1,
                            robot_model.l_link2,
                            robot_model.l_link3,
                            robot_model.l_link4,
                            robot_model.l_link5,
                            robot_model.l_link6,
                            robot_model.l_link7]
        return r_arm_link_list, l_arm_link_list

    def joint_state_callback(self, joint_state_msg):
        self.current_joint_state = joint_state_msg
        self.joint_state_dict = dict(zip(joint_state_msg.name, joint_state_msg.position))

    # 팔 타겟 포즈 콜백
    def r_arm_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_arm_target_pose, args=("r_arm", pose_msg.pose, self.r_arm_pub, self.r_arm_joint_names)).start()

    def l_arm_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_arm_target_pose, args=("l_arm", pose_msg.pose, self.l_arm_pub, self.l_arm_joint_names)).start()

    # 그리퍼 타겟 값 콜백 (기존 코드 유지)
    def r_hand_target_value_callback(self, value_msg):
        threading.Thread(target=self.process_hand_target_value, args=("r_hand", value_msg.data, CONTROL_R)).start()

    def l_hand_target_value_callback(self, value_msg):
        threading.Thread(target=self.process_hand_target_value, args=("l_hand", value_msg.data, CONTROL_L)).start()

    def neck_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_neck_target_pose, args=(pose_msg.pose,)).start()

    def waist_yaw_target_pose_callback(self, pose_msg):
        threading.Thread(target=self.process_waist_yaw_target_pose, args=(pose_msg.pose,)).start()

    def command_pose_callback(self, command_msg):
        if command_msg.data == "init_pose":
            self.process_init_pose()

    # 팔 IK 처리 (scikit-robot 사용)
    def process_arm_target_pose(self, part, target_pose, publisher, joint_names):
        start_time = rospy.Time.now()
        joint_positions = self.solve_ik_fast(part, target_pose, joint_names)
        if joint_positions:
            traj = self.create_simple_trajectory(joint_positions, joint_names)
            self.send_trajectory(traj, publisher)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"{part} IK 처리 시간: {process_time:.4f}초")

    # scikit-robot을 사용한 IK 계산
    def solve_ik_fast(self, part, target_pose, joint_names):
        # 목표 포즈를 scikit-robot의 Coordinates 객체로 변환
        pos = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        quat = [target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z]
        target_coords = skrobot.coordinates.Coordinates(pos=pos, rot=quat)

        # IK 설정
        if part == "r_arm":
            model = self.robot_model
            end_effector = self.r_arm_end_effector
            joints = self.r_arm_joints
            link_list = self.r_arm_link_list
        elif part == "l_arm":
            model = self.robot_model_copy
            end_effector = self.l_arm_end_effector
            joints = self.l_arm_joints
            link_list = self.l_arm_link_list
        else:
            return None

        # 현재 조인트 상태 설정
        if self.joint_state_dict:
            for joint_name, angle in self.joint_state_dict.items():
                if joint_name in joint_names:
                    model.__dict__[joint_name].joint_angle(angle)
        else:
            rospy.logwarn(f"[{part}] Joint state not available, using default angles.")

        # IK 계산
        success = model.inverse_kinematics(target_coords,
                                            stop=1,
                                            link_list=link_list,
                                            move_target=end_effector,
                                            revert_if_fail=False,
                                           )

        print(f"[{part}] IK success:", success)

        if success is not None:
            solution = [model.__dict__[jname].joint_angle() for jname in joint_names]
            self.last_solutions[part] = solution
            return solution
        else:
            rospy.logwarn(f"[{part}] IK solving failed.")
            return None

    # 트라젝트리 생성 및 전송 (기존 코드 유지)
    def create_simple_trajectory(self, target_joint_positions, joint_names):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_joint_positions
        point.time_from_start = rospy.Duration(0.01)
        traj.points = [point]
        return traj

    def send_trajectory(self, traj, publisher):
        traj.header.stamp = rospy.Time.now()
        publisher.publish(traj)

    # 나머지 메서드 (그리퍼, 네크, 웨이스트 등)는 기존 코드 유지
    def process_hand_target_value(self, part, target_value, control_type):
        start_time = rospy.Time.now()
        _inv_target_value = 1 - target_value
        _inv_target_value = np.interp(_inv_target_value, (0.0, 1.0), (MIN_GRIPPER_ANGLE, MAX_GRIPPER_ANGLE))
        if part == "r_hand":
            self.command(_inv_target_value, effort=0.01, type=control_type)
        elif part == "l_hand":
            self.command(-1 * _inv_target_value, effort=0.01, type=control_type)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo(f"{part} 처리 시간: {process_time:.4f}초")

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
        # 초기 포즈 설정 (scikit-robot으로 조정 가능)
        init_r_arm = [0.0] * len(self.r_arm_joint_names)  # 예시 값, URDF에 맞게 조정
        init_l_arm = [0.0] * len(self.l_arm_joint_names)
        self.send_trajectory(self.create_simple_trajectory(init_r_arm, self.r_arm_joint_names), self.r_arm_pub)
        self.send_trajectory(self.create_simple_trajectory(init_l_arm, self.l_arm_joint_names), self.l_arm_pub)
        self.command(MAX_GRIPPER_ANGLE, effort=0.01, type=CONTROL_R)
        self.command(-1 * MAX_GRIPPER_ANGLE, effort=0.01, type=CONTROL_L)
        _joint_positions = [0.0]
        _traj = self.create_simple_trajectory(_joint_positions, self.waist_yaw_joint_names)
        self.send_trajectory(_traj, self.waist_yaw_pub)
        _joint_positions = [0.0, 0.0]
        _traj = self.create_simple_trajectory(_joint_positions, self.neck_joint_names)
        self.send_trajectory(_traj, self.neck_pub)
        rospy.loginfo("Moved to init_pose")

    # 그리퍼 제어 메서드 (기존 코드 유지)
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

if __name__ == "__main__":
    rospy.init_node("sciurus17_fast_ik_node")
    ik_solver = Sciurus17FastIK()
    rospy.loginfo("Sciurus17 Fast IK Solver 실행 중 (scikit-robot)...")
    rospy.spin()
