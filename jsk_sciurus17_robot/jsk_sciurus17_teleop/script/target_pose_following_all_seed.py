#!/usr/bin/env python3
# -*- coding: utf-8 -*- # UTF-8 인코딩 추가

import rospy
import numpy as np
import time
import sys
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, MoveItErrorCodes # RobotState 와 에러 코드 추가
from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import PoseStamped, Pose, Quaternion # Pose 추가
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)
import threading
import tf.transformations as tf_trans # Quaternion 변환을 위해
import actionlib

CONTROL_R = 0
CONTROL_L = 1

MAX_GRIPPER_ANGLE = 0.8
MIN_GRIPPER_ANGLE = 0.0

class Sciurus17FastIK:
    def __init__(self):
        # 팔 그룹 초기화 (IK 사용)
        # MoveGroupCommander 는 플래닝/실행 인터페이스에 더 적합합니다.
        # IK 계산 자체에는 그룹 이름만 필요하므로, 여기서는 이름을 저장해 둡니다.
        self.r_arm_group_name = "r_arm_group"
        self.l_arm_group_name = "l_arm_group"
        self.r_arm_group = MoveGroupCommander(self.r_arm_group_name) # init pose 등에 사용
        self.l_arm_group = MoveGroupCommander(self.l_arm_group_name) # init pose 등에 사용

        # 그리퍼 액션 클라이언트 초기화
        self._clientR = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
        self._clientL = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)

        self._goalR = GripperCommandGoal()
        self._goalL = GripperCommandGoal()

        # 액션 서버 대기
        if not self._clientR.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper R Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear(CONTROL_R) # 초기화 명확화

        if not self._clientL.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper L Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear(CONTROL_L) # 초기화 명확화

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
        rospy.loginfo("Waiting for /compute_ik service...")
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        rospy.loginfo("/compute_ik service connected.")

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
        self.joint_state_lock = threading.Lock() # joint_state 접근 보호용 락
        self.joint_state_dict = {} # 이름-위치 쌍을 저장하는 딕셔너리
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1)

        # 조인트 이름 설정
        # MoveGroupCommander에서 가져오는 것이 일반적입니다.
        self.r_arm_joint_names = self.r_arm_group.get_active_joints()
        self.l_arm_joint_names = self.l_arm_group.get_active_joints()
        # 그리퍼 조인트는 Action Controller를 사용하므로 MoveIt 그룹은 필요 없을 수 있습니다.
        # self.r_hand_joint_names = ["right_hand_j"] # 예시 이름, 실제 이름 확인 필요
        # self.l_hand_joint_names = ["left_hand_j"] # 예시 이름, 실제 이름 확인 필요

        self.neck_joint_names = ["neck_pitch_joint", "neck_yaw_joint"]
        self.waist_yaw_joint_names = ["waist_yaw_joint"]

        self.trajectory_lock = threading.Lock() # 궤적 전송 보호용 락

        # last_solutions 는 디버깅이나 특정 로직에 필요할 수 있으므로 유지
        self.last_solutions = {
            "r_arm": None,
            "l_arm": None
        }
        rospy.loginfo("Sciurus17 Fast IK Solver 초기화 완료")

    def joint_state_callback(self, joint_state_msg):
        # joint_state_dict 업데이트 시 락 사용
        with self.joint_state_lock:
            self.current_joint_state = joint_state_msg
            # 모든 조인트 상태를 딕셔너리에 저장하여 빠르게 접근 가능하도록 함
            self.joint_state_dict = dict(zip(joint_state_msg.name, joint_state_msg.position))

    # 팔 타겟 포즈 콜백
    def r_arm_target_pose_callback(self, pose_msg):
        # 스레드를 사용하여 비동기 처리
        threading.Thread(target=self.process_arm_target_pose, args=(
            "r_arm", self.r_arm_group_name, pose_msg.pose, self.r_arm_pub, self.r_arm_joint_names
        )).start()

    def l_arm_target_pose_callback(self, pose_msg):
        # 스레드를 사용하여 비동기 처리
        threading.Thread(target=self.process_arm_target_pose, args=(
            "l_arm", self.l_arm_group_name, pose_msg.pose, self.l_arm_pub, self.l_arm_joint_names
        )).start()

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
            # init_pose는 순차적으로 실행되어야 할 수 있으므로 스레드 대신 직접 호출 고려
            self.process_init_pose()

    # 팔 IK 처리
    def process_arm_target_pose(self, part, group_name, target_pose, publisher, joint_names):
        start_time = rospy.Time.now()
        # solve_ik_fast 호출 시 group_name 전달
        joint_positions = self.solve_ik_fast(part, group_name, target_pose, joint_names)

        if joint_positions:
            # 궤적 생성 및 전송
            traj = self.create_simple_trajectory(joint_positions, joint_names)
            self.send_trajectory(traj, publisher)
            # 성공 시 마지막 해 저장 (선택적)
            self.last_solutions[part] = joint_positions
        else:
            rospy.logwarn(f"{part} IK solution not found.")
            # 실패 시 마지막 해를 None으로 설정하거나 유지 (선택)
            # self.last_solutions[part] = None

        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.logdebug(f"{part} IK 처리 시간: {process_time:.4f}초") # Debug 레벨로 변경

    # 그리퍼 처리 (actionlib 기반, 스레드 내에서 실행)
    def process_hand_target_value(self, part, target_value, control_type):
        start_time = rospy.Time.now()
        # 값 변환: 0(open) ~ 1(close) -> MIN_GRIPPER_ANGLE ~ MAX_GRIPPER_ANGLE
        # Sciurus17 Gripper: 양수 값이 닫는 것일 수 있음 (확인 필요)
        # 예시: 0 -> MAX, 1 -> MIN (만약 닫는게 작은 각도라면)
        # _target_angle = np.interp(target_value, (0.0, 1.0), (MAX_GRIPPER_ANGLE, MIN_GRIPPER_ANGLE))

        # 코드는 1-target_value를 사용했으므로, 0(open) -> 1 -> MIN, 1(close) -> 0 -> MAX 로 매핑
        _inv_target_value = 1.0 - target_value
        _target_angle = np.interp(_inv_target_value, (0.0, 1.0), (MIN_GRIPPER_ANGLE, MAX_GRIPPER_ANGLE))

        # Sciurus17 특정: 왼쪽 그리퍼는 음수 값 사용
        if control_type == CONTROL_L:
            _target_angle *= -1

        self.command(_target_angle, effort=0.01, type=control_type)
        # wait는 필요 시 호출 (예: 순차 작업)

        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.logdebug(f"{part} 처리 시간: {process_time:.4f}초")

    # 네크 처리
    def process_neck_target_pose(self, target_pose):
        start_time = rospy.Time.now()
        quat = target_pose.orientation
        # ZYX 오일러 각도 (roll, pitch, yaw 순서)
        euler = tf_trans.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # Sciurus17 목 조인트 순서 및 방향 확인 필요
        # 예: pitch = -euler[1], yaw = euler[2] (코드와 동일)
        pitch = -euler[1] # Y축 회전 (오일러 각도 부호 반대)
        yaw = euler[2]    # Z축 회전
        joint_positions = [pitch, yaw] # 조인트 이름 순서와 일치해야 함
        traj = self.create_simple_trajectory(joint_positions, self.neck_joint_names)
        self.send_trajectory(traj, self.neck_pub)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.logdebug(f"neck 처리 시간: {process_time:.4f}초")

    # 웨이스트 요 처리
    def process_waist_yaw_target_pose(self, target_pose):
        start_time = rospy.Time.now()
        quat = target_pose.orientation
        euler = tf_trans.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = euler[2] # Z축 회전
        joint_positions = [yaw] # 조인트 이름 순서와 일치해야 함
        traj = self.create_simple_trajectory(joint_positions, self.waist_yaw_joint_names)
        self.send_trajectory(traj, self.waist_yaw_pub)
        process_time = (rospy.Time.now() - start_time).to_sec()
        rospy.logdebug(f"waist_yaw 처리 시간: {process_time:.4f}초")

    def process_init_pose(self):
        rospy.loginfo("Moving to init_pose...")

        # MoveGroupCommander 사용 (이름 지정된 포즈)
        rospy.loginfo("Moving right arm to init pose...")
        self.r_arm_group.set_named_target("r_arm_init_pose")
        if not self.r_arm_group.go(wait=True):
             rospy.logerr("Failed to move right arm to init pose")
             return # 실패 시 중단
        self.r_arm_group.stop() # 실행 후 정리

        rospy.loginfo("Opening right gripper...")
        self.command(MAX_GRIPPER_ANGLE, effort=0.01, type=CONTROL_R) # 열린 상태 (최대 각도)
        self.wait(CONTROL_R, timeout=1.0) # 완료 대기 시간 증가

        rospy.loginfo("Moving left arm to init pose...")
        self.l_arm_group.set_named_target("l_arm_init_pose")
        if not self.l_arm_group.go(wait=True):
            rospy.logerr("Failed to move left arm to init pose")
            return # 실패 시 중단
        self.l_arm_group.stop()

        rospy.loginfo("Opening left gripper...")
        self.command(-1 * MAX_GRIPPER_ANGLE, effort=0.01, type=CONTROL_L) # 열린 상태 (음수 최대 각도)
        self.wait(CONTROL_L, timeout=1.0) # 완료 대기 시간 증가

        # 허리 및 목은 직접 궤적 전송
        rospy.loginfo("Moving waist to init pose...")
        _waist_traj = self.create_simple_trajectory([0.0], self.waist_yaw_joint_names)
        self.send_trajectory(_waist_traj, self.waist_yaw_pub)
        time.sleep(0.5) # 간단한 대기

        rospy.loginfo("Moving neck to init pose...")
        _neck_traj = self.create_simple_trajectory([0.0, 0.0], self.neck_joint_names)
        self.send_trajectory(_neck_traj, self.neck_pub)
        time.sleep(0.5) # 간단한 대기

        rospy.loginfo("Moved to init_pose successfully.")

    # actionlib를 통한 그리퍼 제어
    def command(self, position, effort, type):
        client = self._clientR if type == CONTROL_R else self._clientL
        goal = self._goalR if type == CONTROL_R else self._goalL

        goal.command.position = position
        goal.command.max_effort = effort
        # feedback 콜백 제거 또는 필요한 경우 유지
        client.send_goal(goal) # feedback_cb 제거 또는 유지
        # rospy.logdebug(f"Sent gripper goal: pos={position}, effort={effort}, type={type}")

    def feedbackR(self, msg):
        # 필요 시 피드백 처리
        # rospy.logdebug("Right Gripper Feedback: %s" % msg)
        pass

    def feedbackL(self, msg):
        # 필요 시 피드백 처리
        # rospy.logdebug("Left Gripper Feedback: %s" % msg)
        pass

    def stop(self, type=None):
        """지정된 그리퍼 또는 모든 그리퍼의 동작을 중지합니다."""
        if type is None or type == CONTROL_R:
            self._clientR.cancel_goal()
        if type is None or type == CONTROL_L:
            self._clientL.cancel_goal()
        # rospy.loginfo(f"Stopped gripper goal (type: {type if type is not None else 'All'})")

    def wait(self, type, timeout=0.5): # 기본 타임아웃 증가
        """지정된 그리퍼의 액션 완료를 기다립니다."""
        client = self._clientR if type == CONTROL_R else self._clientL
        if client.wait_for_result(rospy.Duration(timeout)):
            return client.get_result()
        else:
            rospy.logwarn(f"Gripper type {type} action did not complete within {timeout}s.")
            return None

    def clear(self, type=None):
        """지정된 그리퍼 또는 모든 그리퍼의 목표를 초기화합니다."""
        if type is None or type == CONTROL_R:
            self._goalR = GripperCommandGoal()
        if type is None or type == CONTROL_L:
            self._goalL = GripperCommandGoal()

    # IK 풀이 (팔에만 적용) - 현재 상태를 시드로 사용하도록 수정
    def solve_ik_fast(self, part, group_name, target_pose, joint_names):
        """
        지정된 팔 그룹에 대해 IK를 계산합니다. 현재 조인트 상태를 시드로 사용합니다.
        """
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = group_name # 전달된 group_name 사용
        ik_request.ik_request.pose_stamped.header.frame_id = self.robot.get_planning_frame()
        ik_request.ik_request.pose_stamped.pose = target_pose
        # 타임아웃을 약간 늘려 해를 찾을 가능성 높임 (예: 0.05초)
        ik_request.ik_request.timeout = rospy.Duration(0.001)
        # Avoid collisions (기본값은 False 이지만 명시적으로 설정 가능)
        ik_request.ik_request.avoid_collisions = False

        # --- 현재 조인트 상태를 IK 시드로 설정 ---
        seed_state = RobotState()
        valid_seed = False
        with self.joint_state_lock: # 락 안에서 joint_state_dict 접근
            if self.current_joint_state and self.joint_state_dict:
                seed_state.joint_state.header = self.current_joint_state.header
                current_positions = []
                joint_names_in_seed = []
                # group_name 에 해당하는 조인트들의 현재 값만 추출
                for name in joint_names:
                    if name in self.joint_state_dict:
                        joint_names_in_seed.append(name)
                        current_positions.append(self.joint_state_dict[name])
                    else:
                        rospy.logwarn(f"Joint '{name}' not found in current joint_state_dict for seeding IK for {part}. Skipping seed for this joint.")
                        # 여기서 break 하지 않고 계속 진행하여 부분적인 시드라도 사용 시도 가능
                        # 하지만 모든 조인트가 있어야 시드가 유효하므로, 여기서는 실패 처리
                        valid_seed = False
                        break

                # 모든 필요한 조인트 값을 찾았는지 확인
                if len(joint_names_in_seed) == len(joint_names):
                    seed_state.joint_state.name = joint_names_in_seed
                    seed_state.joint_state.position = current_positions
                    valid_seed = True

        if valid_seed:
            ik_request.ik_request.robot_state = seed_state
            # rospy.logdebug(f"Using current joint state as seed for {part} IK.")
        else:
            rospy.logwarn(f"Could not construct a valid seed state from current joint state for {part}. Using default IK solver seed.")
            # 시드 없이 요청하거나, 이전에 성공한 해(self.last_solutions)를 사용하는 등 대체 전략 고려 가능
            # 여기서는 시드 없이 진행 (MoveIt 기본 시드 사용)
        # ---------------------------------------

        try:
            response = self.ik_service(ik_request)
            # 성공 코드 확인 (MoveItErrorCodes 사용)
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                # 결과에서 해당 그룹의 조인트 값만 추출 (순서 보장)
                solution_dict = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
                solution = []
                for name in joint_names:
                    if name in solution_dict:
                        solution.append(solution_dict[name])
                    else:
                        # 이런 경우는 거의 없어야 함
                        rospy.logerr(f"Joint '{name}' not found in IK solution for {part}!")
                        return None # 실패 처리
                # rospy.logdebug(f"IK solution found for {part}.")
                return solution
            else:
                # 실패 시 에러 코드 출력 (더 많은 정보 제공)
                rospy.logwarn(f"IK failed for {part} with error code: {self.error_code_to_string(response.error_code.val)}")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"IK service call failed for {part}: {e}")
            return None
        except Exception as e: # 일반적인 예외 처리 추가
             rospy.logerr(f"An unexpected error occurred during IK solving for {part}: {e}")
             return None

    def create_simple_trajectory(self, target_joint_positions, joint_names):
        """단일 포인트를 가진 JointTrajectory 메시지를 생성합니다."""
        if not joint_names:
             rospy.logwarn("Cannot create trajectory with empty joint names.")
             return None
        if len(target_joint_positions) != len(joint_names):
            rospy.logwarn(f"Mismatch between joint positions ({len(target_joint_positions)}) and names ({len(joint_names)}). Cannot create trajectory.")
            return None

        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_joint_positions
        # time_from_start 를 약간 늘려 부드러운 움직임 유도 (예: 0.1초)
        # 너무 짧으면 컨트롤러가 따라가지 못할 수 있음
        point.time_from_start = rospy.Duration(0.01)
        # 속도/가속도 정보는 비워둠 (Simple Controller는 위치만 사용)
        traj.points = [point]
        return traj

    def send_trajectory(self, traj, publisher):
        """생성된 궤적 메시지를 해당 퍼블리셔로 전송합니다."""
        if traj is None:
            rospy.logwarn("Attempted to send a null trajectory.")
            return

        # 궤적 전송 시 락 사용 (동시 접근 방지)
        with self.trajectory_lock:
            try:
                traj.header.stamp = rospy.Time.now() + rospy.Duration(0.01) # 약간의 지연 시간 부여
                publisher.publish(traj)
                # rospy.logdebug(f"Sent trajectory to {publisher.name}")
            except Exception as e:
                rospy.logerr(f"Failed to publish trajectory to {publisher.name}: {e}")

    def error_code_to_string(self, error_code):
        """MoveIt 에러 코드를 문자열로 변환합니다."""
        # MoveItErrorCodes 메시지에 정의된 상수 사용
        error_dict = {v: k for k, v in MoveItErrorCodes.__dict__.items() if k.isupper() and isinstance(v, int)}
        return error_dict.get(error_code, f"UNKNOWN_ERROR_CODE_{error_code}")


if __name__ == "__main__":
    # 로그 레벨 설정 (DEBUG, INFO, WARN, ERROR, FATAL)
    # rospy.init_node("sciurus17_fast_ik_node", log_level=rospy.DEBUG)
    rospy.init_node("sciurus17_fast_ik_node") # 기본 INFO 레벨
    try:
        ik_solver = Sciurus17FastIK()
        rospy.loginfo("Sciurus17 Fast IK Solver 실행 중...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node shutdown requested.")
    except Exception as e:
        rospy.logfatal(f"Unhandled exception in main: {e}")
    finally:
        rospy.loginfo("Sciurus17 Fast IK Solver 종료.")
