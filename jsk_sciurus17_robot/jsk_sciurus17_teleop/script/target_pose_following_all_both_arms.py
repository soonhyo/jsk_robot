#!/usr/bin/env python
import rospy
import numpy as np
import time
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import Constraints, JointConstraint # 제약 조건 사용 위해 추가
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
        self.two_arm_group = MoveGroupCommander("two_arm_group")

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
        self.two_arm_joint_names = self.two_arm_group.get_active_joints()

        self.two_arm_link_names = self.robot.get_link_names("two_arm_group")

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
    # def r_arm_target_pose_callback(self, pose_msg):
    #     threading.Thread(target=self.process_arm_target_pose, args=("r_arm", self.r_arm_group, pose_msg.pose, self.r_arm_pub, self.r_arm_joint_names)).start()

    # def l_arm_target_pose_callback(self, pose_msg):
    #     threading.Thread(target=self.process_arm_target_pose, args=("l_arm", self.l_arm_group, pose_msg.pose, self.l_arm_pub, self.l_arm_joint_names)).start()

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

    def r_arm_target_pose_callback(self, pose_msg):
        # 오른팔만 처리할 경우
        # threading.Thread(target=self.process_both_arms_target_pose, args=(pose_msg.pose, None)).start()
        self.r_arm_target_pose = pose_msg
        threading.Thread(target=self.process_both_arms_target_pose, args=(self.r_arm_target_pose, self.l_arm_target_pose)).start()

    def l_arm_target_pose_callback(self, pose_msg):
        # 왼팔만 처리할 경우
        # threading.Thread(target=self.process_both_arms_target_pose, args=(None, pose_msg.pose)).start()
        self.l_arm_target_pose = pose_msg
        threading.Thread(target=self.process_both_arms_target_pose, args=(self.r_arm_target_pos, self.l_arm_target_pose)).start()

    def process_both_arms_target_pose(self, r_target_pose=None, l_target_pose=None):
        with self.lock:  # 동기화 보장
            start_time = rospy.Time.now()

            # 현재 로봇 상태 가져오기
            current_robot_state = self.robot.get_current_state()

            # 오른팔 IK 계산
            both_arm_joint_positions = None
            both_arm_joint_positions = self.solve_ik_fast(
                "two_arm", self.two_arm_group, [r_target_pose, l_target_pose],
                self.two_arm_joint_names
            )

            # 트라젝트리 생성 및 전송
            if both_arm_joint_positions:
                print("both_arm_joint_positions:", both_arm_joint_positions)
            #     r_traj = self.create_simple_trajectory(r_joint_positions, self.r_arm_joint_names)
            #     self.send_trajectory(r_traj, self.r_arm_pub)
            # if l_joint_positions:
            #     l_traj = self.create_simple_trajectory(l_joint_positions, self.l_arm_joint_names)
            #     self.send_trajectory(l_traj, self.l_arm_pub)

            process_time = (rospy.Time.now() - start_time).to_sec()
            rospy.loginfo(f"Both arms IK 처리 시간: {process_time:.4f}초")

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

    # IK 풀이 (팔에만 적용) - 끝 관절로 갈수록 허용 오차 증가
    def solve_ik_fast(self, part, group, target_pose, joint_names):
        # IK 요청 객체 먼저 생성
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = f"{part}_group"
        for i in range(len(target_pose)):
            pose_stamped = PoseStamped()
            pose_stamped.pose = target_pose[i]
            pose_stamped.header.frame_id = self.robot.get_planning_frame()
            ik_request.ik_request.pose_stamped_vector.append(pose_stamped)
        ik_request.ik_request.timeout = rospy.Duration(0.01) # 타임아웃 짧게 유지 (Fast IK 목적)
        # 초기 로봇 상태 설정 (아래에서 업데이트될 수 있음)
        ik_request.ik_request.robot_state = self.robot.get_current_state()
        ik_request.ik_request.ik_link_names = self.two_arm_link_names

        # --- 제약 조건 설정 시작 (가변 허용 오차 적용) ---
        constraints = Constraints()
        constraints.name = f"{part}_ik_joint_limit_variable"

        # 최소/최대 허용 오차 설정 (라디안)
        min_tolerance_rad = 0.3 # 베이스에 가까운 조인트의 허용 오차 (예: 약 2.8도)
        max_tolerance_rad = 0.5   # 끝(end-effector)에 가까운 조인트의 허용 오차 (예: 약 28.6도)

        # if not self.joint_state_dict:
        #     rospy.logwarn(f"[{part}] Cannot create constraints for IK: joint_state_dict is empty.")
        # else:
        #     valid_joints = True
        #     current_joint_values_for_constraints = {}
        #     for joint_name in joint_names:
        #         if joint_name in self.joint_state_dict:
        #             current_joint_values_for_constraints[joint_name] = self.joint_state_dict[joint_name]
        #         else:
        #             rospy.logwarn(f"[{part}] Cannot find current value for joint '{joint_name}' in joint_state_dict for IK constraint.")
        #             valid_joints = False
        #             break

        #     if valid_joints:
        #         num_joints = len(joint_names)
        #         # 각 조인트에 대해 순서(index)에 따라 다른 허용 오차 적용
        #         for i, joint_name in enumerate(joint_names):
        #             # 현재 조인트의 허용 오차 계산 (선형 보간)
        #             if num_joints > 1:
        #                 # i=0 (베이스 쪽)일 때 min, i=num_joints-1 (끝 쪽)일 때 max가 되도록 보간
        #                 fraction = float(i) / (num_joints - 1)
        #                 current_tolerance = min_tolerance_rad + (max_tolerance_rad - min_tolerance_rad) * fraction
        #             else:
        #                 # 조인트가 하나뿐인 경우 (이론상 가능) 평균 또는 기본값 사용
        #                 current_tolerance = (min_tolerance_rad + max_tolerance_rad) / 2.0

        #             jc = JointConstraint()
        #             jc.joint_name = joint_name
        #             jc.position = current_joint_values_for_constraints[joint_name] # 현재 각도 기준
        #             jc.tolerance_above = current_tolerance # 계산된 허용 오차 적용
        #             jc.tolerance_below = current_tolerance # 계산된 허용 오차 적용 (값 자체는 양수)
        #             jc.weight = 1.0 # IK 솔버가 가중치를 해석할 경우 의미 있음
        #             constraints.joint_constraints.append(jc)

        #         # 생성된 제약 조건을 IK 요청에 추가
        #         ik_request.ik_request.constraints = constraints
        #         rospy.loginfo(f"[{part}] Added variable constraints (min:{min_tolerance_rad:.3f}, max:{max_tolerance_rad:.3f}) to IK request.")
        #     else:
        #         rospy.logwarn(f"[{part}] Could not add constraints to IK request due to missing joint values.")
        # # --- 제약 조건 설정 끝 ---

        # # --- IK 시드 상태 설정 (기존 로직 유지) ---
        # # 제약조건과 함께 시드 상태는 IK 해 탐색에 영향을 줌
        # if self.current_joint_state:
        #     # 주의: robot_state를 직접 수정하는 것은 잠재적 위험이 있음
        #     # current_state 복사 후 수정하는 것이 더 안전할 수 있음
        #     robot_state_for_ik = self.robot.get_current_state().joint_state # 복사본 사용 고려

        #     # last_solutions 를 시드 상태로 사용하려는 로직 (원래 코드 유지)
        #     # 이 로직은 제약 조건과 상호작용할 수 있음
        #     last_sol = self.last_solutions.get(part)
        #     if last_sol:
        #          # last_sol이 joint_names 순서와 일치한다고 가정
        #          if len(last_sol) == len(joint_names):
        #              temp_joint_state = JointState()
        #              temp_joint_state.header = robot_state_for_ik.header # 헤더 복사
        #              temp_joint_state.name = list(robot_state_for_ik.name) # 이름 복사
        #              temp_joint_state.position = list(robot_state_for_ik.position) # 위치 복사

        #              name_to_last_sol_map = dict(zip(joint_names, last_sol))

        #              updated_count = 0
        #              for idx, name in enumerate(temp_joint_state.name):
        #                   if name in name_to_last_sol_map:
        #                       temp_joint_state.position[idx] = name_to_last_sol_map[name]
        #                       updated_count += 1

        #              if updated_count > 0:
        #                   ik_request.ik_request.robot_state.joint_state = temp_joint_state
        #                   # rospy.loginfo(f"[{part}] Using last solution as seed state for IK.")
        #          else:
        #               rospy.logwarn(f"[{part}] Mismatch length between last_sol ({len(last_sol)}) and joint_names ({len(joint_names)}). Not using last solution as seed.")


        # --- IK 서비스 호출 ---
        # try:
        response = self.ik_service(ik_request)
        if response.error_code.val == response.error_code.SUCCESS: # SUCCESS (1) 확인
            # joint_names 순서에 맞게 결과 추출
            print("reponse.solution.joint_state;", reponse.solution.joint_state)
            # solution_dict = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            # solution = [solution_dict[name] for name in joint_names if name in solution_dict]

            # # 모든 joint 이름이 결과에 있는지 확인 (중요)
            # if len(solution) == len(joint_names):
            #     self.last_solutions[part] = solution # 성공 시 last_solution 업데이트
            #     return solution
            # else:
            #     rospy.logerr(f"[{part}] IK solution found, but mismatch in expected joints. Found {len(solution)}, expected {len(joint_names)}.")
            #     return None
        else:
            # 에러 코드 출력 (디버깅에 유용)
            rospy.logwarn(f"[{part}] IK Failed. Error code: {response.error_code.val}")
            return None
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"[{part}] IK service call failed: {e}")
        #     return None
        # except Exception as e: # 일반 예외 처리
        #     rospy.logerr(f"[{part}] An unexpected error occurred during IK solving: {e}")
        #     return None

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

if __name__ == "__main__":
    rospy.init_node("sciurus17_fast_ik_node")
    ik_solver = Sciurus17FastIK()
    rospy.loginfo("Sciurus17 Fast IK Solver 실행 중...")
    rospy.spin()
