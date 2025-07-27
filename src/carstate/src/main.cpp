#include "ros/ros.h"
#include "fsd_common_msgs/CarState.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "carstatepub");
    ros::NodeHandle nh;
    bool first_msg = true;
    geometry_msgs::Pose2D origin;
    tf2_ros::TransformBroadcaster br;
    tf2_ros::StaticTransformBroadcaster static_br;

    // 发布静态坐标变换 car -> rslidar
    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = "car";
    static_transform.child_frame_id = "rslidar";
    static_transform.transform.translation.x = 4.2;  // 车子正前方4.2m
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;
    static_br.sendTransform(static_transform);

    // 发布动态坐标变换 map -> car
    ros::Subscriber carsub = nh.subscribe<fsd_common_msgs::CarState>("/estimation/slam/state", 10, [&](const fsd_common_msgs::CarStateConstPtr &msg){
        const geometry_msgs::Pose2D &car_state = msg->car_state;
        if (first_msg) {
            origin.x = car_state.x - 0.3;
            origin.y = car_state.y;
            origin.theta = car_state.theta;
            first_msg = false;
            return;
        }

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "car";

        transformStamped.transform.translation.x = car_state.x - origin.x;
        transformStamped.transform.translation.y = car_state.y - origin.y;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, car_state.theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    });
    ros::spin();
    return 0;
}