#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import psutil
import time
import os
import signal
import yaml
from jsk_sciurus17_teleop.srv import StartRosbag, StartRosbagResponse, CancelRosbag, CancelRosbagResponse
from datetime import datetime

rosbag_proc = None
last_bag_path = None

def is_process_alive(proc):
    return proc is not None and proc.poll() is None

def load_topics():
    config_path = rospy.get_param("~topics_yaml", "topics.yaml")
    if not os.path.isfile(config_path):
        rospy.logerr(f"[rosbag_service] Cannot find topics.yaml at: {config_path}")
        return []

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
        topics = data.get('topics', [])
        rospy.loginfo(f"[rosbag_service] Loaded topics: {topics}")
        return topics

def start_bag(req):
    global rosbag_proc, last_bag_path

    if is_process_alive(rosbag_proc):
        return StartRosbagResponse(success=False, message="rosbag already recording")

    topics = load_topics()
    if not topics:
        return StartRosbagResponse(success=False, message="No topics to record")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = req.bag_name
    base_dir = os.path.expanduser("~/rosbags")  # 원하는 기본 경로
    bag_dir = os.path.join(base_dir, bag_name)
    os.makedirs(bag_dir, exist_ok=True)

    full_bag_path = os.path.join(bag_dir, f"{timestamp}.bag")
    last_bag_path = full_bag_path  # 삭제용으로 기록

    rosbag_proc = subprocess.Popen(['rosbag', 'record', '-O', full_bag_path] + topics)
    return StartRosbagResponse(success=True, message=f"Started recording to {full_bag_path}")

# def start_bag(req):
#     global rosbag_proc
#     if is_process_alive(rosbag_proc):
#         return TriggerResponse(success=False, message="rosbag already recording")

#     topics = load_topics()
#     if not topics:
#         return TriggerResponse(success=False, message="No topics to record")

#     rosbag_proc = subprocess.Popen(['rosbag', 'record', '-O', 'teleop_record.bag'] + topics)
#     return TriggerResponse(success=True, message="Started recording")
def cancel_bag(req):
    global last_bag_path
    try:
        if last_bag_path and os.path.exists(last_bag_path):
            os.remove(last_bag_path)
            rospy.loginfo(f"Deleted rosbag: {last_bag_path}")
            return CancelRosbagResponse(success=True, message="Last rosbag deleted")
        else:
            return CancelRosbagResponse(success=False, message="No rosbag to delete")
    except Exception as e:
        return CancelRosbagResponse(success=False, message=str(e))
    
def stop_bag(req):
    global rosbag_proc
    rospy.loginfo("Stop rosbag called")

    if not is_process_alive(rosbag_proc):
        rospy.logwarn("rosbag not running")
        return TriggerResponse(success=False, message="rosbag not running")

    try:
        parent = psutil.Process(rosbag_proc.pid)
        children = parent.children(recursive=True)

        for child in children:
            rospy.loginfo(f"Sending SIGINT to child: {child.pid}")
            child.send_signal(signal.SIGINT)

        rospy.loginfo("Waiting for rosbag to exit...")
        gone, alive = psutil.wait_procs([parent] + children, timeout=5)

        if alive:
            rospy.logwarn("Some processes still alive, sending SIGTERM")
            for p in alive:
                p.terminate()
            gone, alive = psutil.wait_procs(alive, timeout=3)

        if alive:
            rospy.logwarn("Force killing remaining processes")
            for p in alive:
                p.kill()

        rosbag_proc = None
        return TriggerResponse(success=True, message="Stopped recording")

    except Exception as e:
        rospy.logerr(f"Failed to stop rosbag: {e}")
        return TriggerResponse(success=False, message="Error stopping rosbag")

if __name__ == "__main__":
    rospy.init_node('rosbag_service_node')
    rospy.Service('start_rosbag', Trigger, start_bag)
    rospy.Service('stop_rosbag', Trigger, stop_bag)
    rospy.spin()
