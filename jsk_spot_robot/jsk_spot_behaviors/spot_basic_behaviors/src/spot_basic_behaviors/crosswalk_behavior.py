# -*- coding: utf-8 -*-

from spot_behavior_manager.base_behavior import BaseBehavior

import roslaunch
import rospkg
import rospy

import threading

from std_msgs.msg import Bool


class CrosswalkBehavior(BaseBehavior):

    def callback_car_visible(self, msg):

        if self.car_state_visible != msg.data:
            self.car_starttime_visibility = rospy.Time.now()
            self.car_duration_visibility = rospy.Duration()
            self.car_state_visible = msg.data
        else:
            self.car_duration_visibility = rospy.Time.now() - self.car_starttime_visibility

    def run_initial(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_initial() called')

        self.silent_mode = rospy.get_param('~silent_mode', True)

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        roslaunch_path = rospkg.RosPack().get_path('spot_basic_behaviors') +\
            '/launch/crosswalk_detection.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        self.roslaunch_parent.start()

        # value for car checker
        self.subscriber_car_visible = None
        self.car_state_visible = False
        self.car_starttime_visibility = rospy.Time.now()
        self.car_duration_visibility = rospy.Duration(10)

        # start subscribers
        try:
            self.subscriber_car_visible = rospy.Subscriber(
                '/crosswalk_detection_car_tracker/visible',
                Bool,
                self.callback_car_visible
            )
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

        return True

    def run_main(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_main() called')

        graph_name = edge.properties['graph']
        start_id = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['id']
        end_id = filter(
            lambda x: x['graph'] == graph_name,
            end_node.properties['waypoints_on_graph']
        )[0]['id']
        localization_method = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['localization_method']

        # graph uploading and localization
        if pre_edge is not None and \
                graph_name == pre_edge.properties['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            ret = self.spot_client.upload_graph(graph_name)
            if ret[0]:
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
            else:
                rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                return False
            # Localization
            if localization_method == 'fiducial':
                ret = self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                ret = self.spot_client.set_localization_waypoint(start_id)
            else:
                ret = (False, 'Unknown localization method')
            if ret[0]:
                rospy.loginfo('robot is localized on the graph.')
            else:
                rospy.logwarn('Localization failed: {}'.format(ret[1]))
                return False

        # checking if there is a moving car visible or not
        if not self.silent_mode:
            self.sound_client.say('車が通るかみています', blocking=True)
        safety_count = 0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('safety_count: {}'.format(safety_count))
            if safety_count > 10:
                break
            try:
                rospy.loginfo('car visible: {}'.format(self.car_state_visible))
                if self.car_state_visible == True:
                    safety_count = 0
                    if not self.silent_mode:
                        self.sound_client.say('車が通ります', blocking=True)
                else:
                    safety_count += 1
            except Exception as e:
                rospy.logwarn('{}'.format(e))
                safety_count = 0

        # start leading
        success = False
        rate = rospy.Rate(10)
        if not self.silent_mode:
            self.sound_client.say('移動します', blocking=True)
        self.spot_client.navigate_to(end_id, blocking=False)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                result = self.spot_client.get_navigate_to_result()
                success = result.success
                rospy.loginfo('result: {}'.format(result))
                break

        # recovery on failure
        if not success:
            if not self.silent_mode:
                self.sound_client.say('失敗したので元に戻ります', blocking=True)
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return success

    def run_final(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_finalize() called')

        self.spot_client.cancel_navigate_to()

        if self.subscriber_car_visible != None:
            self.subscriber_car_visible.unregister()
            self.subscriber_car_visible = None

        self.roslaunch_parent.shutdown()
