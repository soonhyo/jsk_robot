#!/usr/bin/env python3

import rospy
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose, Point, Quaternion

class SciurusInteractiveMarker:
    def __init__(self):
        rospy.init_node('sciurus_interactive_marker', anonymous=True)
        self.server = InteractiveMarkerServer("sciurus_marker")
        self.tf_listener = tf.TransformListener()
        self.target_pose_pub = rospy.Publisher('/sciurus17/target_pose', Pose, queue_size=10)
        self.init_marker()
        rospy.spin()

    def init_marker(self):
        # Wait for TF transform from base_link to r_link7
        try:
            self.tf_listener.waitForTransform('/base_link', '/r_link7', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/r_link7', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF lookup failed: %s", e)
            trans = [0.0, 0.0, 0.5]  # Fallback position
            rot = [0.0, 0.0, 0.0, 1.0]  # Fallback quaternion (identity)

        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "sciurus_end_effector"
        int_marker.description = "6-DoF Control for Sciurus17 Right Arm"
        int_marker.pose = Pose(
            position=Point(*trans),
            orientation=Quaternion(*rot)
        )

        # Create a sphere marker for visualization
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        sphere_marker.color.r = 1.0
        sphere_marker.color.a = 1.0

        # Add 6-DoF controls (3 for position, 3 for orientation)
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(sphere_marker)
        int_marker.controls.append(control)

        # Translation controls (X, Y, Z)
        for axis, name in [(1, "move_x"), (-1, "move_x"), (2, "move_y"), (-2, "move_y"), (3, "move_z"), (-3, "move_z")]:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0 if abs(axis) == 1 else 0.0
            control.orientation.y = 1.0 if abs(axis) == 2 else 0.0
            control.orientation.z = 1.0 if abs(axis) == 3 else 0.0
            control.name = name
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        # Rotation controls (X, Y, Z)
        for axis, name in [(1, "rotate_x"), (-1, "rotate_x"), (2, "rotate_y"), (-2, "rotate_y"), (3, "rotate_z"), (-3, "rotate_z")]:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0 if abs(axis) == 1 else 0.0
            control.orientation.y = 1.0 if abs(axis) == 2 else 0.0
            control.orientation.z = 1.0 if abs(axis) == 3 else 0.0
            control.name = name
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)

        # Insert marker and set callback
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose = feedback.pose
            self.target_pose_pub.publish(pose)
            rospy.loginfo("Published target pose: %s", pose)

if __name__ == "__main__":
    try:
        SciurusInteractiveMarker()
    except rospy.ROSInterruptException:
        pass
