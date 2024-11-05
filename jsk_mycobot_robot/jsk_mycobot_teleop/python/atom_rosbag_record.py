#!/usr/bin/env python
import rospy
import yaml
import subprocess
import signal
import os
from std_msgs.msg import Bool
from datetime import datetime

class RosbagRecorder:
    def __init__(self):
        rospy.init_node('rosbag_recorder', anonymous=True)

        # YAML 설정 파일 로드
        config_path = rospy.get_param('~config_path', 'config/record_config.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # 설정값 가져오기
        self.topics = self.config['topics']
        self.save_path = self.config['save_path']

        # rosbag 프로세스 저장 변수
        self.recording_process = None
        self.is_recording = False
        self.prev_button_state = False  # 이전 버튼 상태를 저장

        # 버튼 구독
        rospy.Subscriber('/master/atom_button', Bool, self.button_callback)
        rospy.loginfo("Ready to record. Waiting for button press...")

    def button_callback(self, msg):
        # Rising edge 감지 (버튼이 눌렸다가 떼어질 때)
        if not self.prev_button_state and msg.data:
            if not self.is_recording:
                self.start_recording()
            else:
                self.stop_recording()

        # 현재 버튼 상태 저장
        self.prev_button_state = msg.data

    def start_recording(self):
        if self.is_recording:
            return

        # 저장 경로가 없으면 생성
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        # 현재 시간으로 파일명 생성
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = os.path.join(self.save_path, f'recording_{timestamp}.bag')

        # rosbag record 명령어 생성
        command = ['rosbag', 'record', '-O', filename]
        command.extend(self.topics)

        # 녹화 시작
        self.recording_process = subprocess.Popen(command)
        self.is_recording = True
        rospy.loginfo(f"Started recording to {filename}")

    def stop_recording(self):
        if not self.is_recording:
            return

        if self.recording_process is not None:
            # SIGINT 시그널 보내서 rosbag 종료
            self.recording_process.send_signal(signal.SIGINT)
            self.recording_process.wait()
            self.recording_process = None
            self.is_recording = False
            rospy.loginfo("Stopped recording")

    def shutdown(self):
        self.stop_recording()

if __name__ == '__main__':
    try:
        recorder = RosbagRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if recorder:
            recorder.shutdown()
