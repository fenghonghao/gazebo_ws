#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "fsd_common_msgs/CarState.h"
#include "tf2/utils.h"

ros::Publisher car_state_pub;

void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    fsd_common_msgs::CarState car_state;
    car_state.header = odom_msg->header;
    
    // 提取位置
    car_state.car_state.x = odom_msg->pose.pose.position.x;
    car_state.car_state.y = odom_msg->pose.pose.position.y;
    
    // 提取偏航角（theta）
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    car_state.car_state.theta = yaw;

    car_state_pub.publish(car_state);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "state_converter");
    ros::NodeHandle nh;
    car_state_pub = nh.advertise<fsd_common_msgs::CarState>("/estimation/slam/state", 10);
    ros::Subscriber odom_sub = nh.subscribe("/estimation/slam/state_raw", 10, odomCallback);
    ros::spin();
    return 0;
}