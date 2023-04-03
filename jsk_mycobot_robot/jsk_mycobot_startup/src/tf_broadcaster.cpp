// tf_broadcaster.cpp
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TfBroadcaster
{
public:
    TfBroadcaster();
    ~TfBroadcaster();
// Broadcast
    void BroadcastStaticTfFromCameraToMyCobot();

private:
    ros::NodeHandle nh_;

// TF Broadcaster
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
// constant
    double PI_ = 3.14159265;
};

TfBroadcaster::TfBroadcaster(){}
TfBroadcaster::~TfBroadcaster(){}

void TfBroadcaster::BroadcastStaticTfFromCameraToMyCobot()
{
    std::string camera_frame_id, base_frame_id;
    double tf_x, tf_y, tf_z;

    ros::param::set("/tf_broadcaster/camera_frame_id", "camera_color_optical_frame");
    ros::param::set("/tf_broadcaster/base_frame_id", "joint1");
    ros::param::set("/tf_broadcaster/x", -0.3);
    ros::param::set("/tf_broadcaster/y", -0.3);
    ros::param::set("/tf_broadcaster/z", -0.3);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = ros::param::get("/tf_broadcaster/camera_frame_id", camera_frame_id) ; // 親link
    transformStamped.child_frame_id = ros::param::get("/tf_broadcaster/base_frame_id", base_frame_id); // 子link

    // 平行移動
    transformStamped.transform.translation.x = ros::param::get("/tf_broadcaster/x", tf_x);
    transformStamped.transform.translation.y = ros::param::get("/tf_broadcaster/y", tf_y);
    transformStamped.transform.translation.z = ros::param::get("/tf_broadcaster/z", tf_z);
    // 回転
    tf2::Quaternion q;
    q.setEuler(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    static_tf_broadcaster_.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_mycobot_broadcaster");
    TfBroadcaster tf_mycobot_broadcaster;
    tf_mycobot_broadcaster.BroadcastStaticTfFromCameraToMyCobot();

    ros::Rate loop_rate(20);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
