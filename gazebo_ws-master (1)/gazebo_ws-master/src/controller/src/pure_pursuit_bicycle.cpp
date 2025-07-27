#include "ros/ros.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"

class BicycleController {
public:
    BicycleController(ros::NodeHandle& nh) : nh_(nh) {
        // 订阅车辆状态和路径点
        car_state_sub_ = nh_.subscribe("/estimation/slam/state", 10, 
            &BicycleController::carStateCallback, this);
        path_sub_ = nh_.subscribe("/path", 10,  
            &BicycleController::pathCallback, this);
        
        // 发布控制指令（自行车模型：速度+转向角）
        control_pub_ = nh_.advertise<fsd_common_msgs::ControlCommand>("/control/command", 10);

        // 加载参数（轴距、最大转向角等）
        nh_.param("wheelbase", wheelbase_, 2.5);  // 轴距（米）
        nh_.param("max_steering_angle", max_steering_angle_, 0.523);  // 最大转向角（30度，弧度）
        nh_.param("lookahead_distance", lookahead_distance_, 5.0);  // 预瞄距离（米）
        nh_.param("target_speed", target_speed_, 8.0);  // 目标线速度（米/秒）
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber car_state_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher control_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::Point> path_points_;  // 存储路径点

    // 自行车模型参数
    double wheelbase_;          // 轴距 L
    double max_steering_angle_; // 最大转向角限制
    double lookahead_distance_; // 预瞄距离
    double target_speed_;       // 目标线速度

    // 路径点回调
    void pathCallback(const nav_msgs::PathConstPtr& msg) {
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.pose.position);
        }
    }


    // 车辆状态回调
    void carStateCallback(const fsd_common_msgs::CarStateConstPtr& car_state) {
        if (path_points_.empty()) {
            ROS_WARN("No path points received");
            return;
        }

        // 1. 查找预瞄点
        geometry_msgs::Point lookahead_point = findLookaheadPoint(car_state);
        if (lookahead_point.x == 0 && lookahead_point.y == 0) {
            ROS_WARN("No valid lookahead point found");
            return;
        }

        // 2. 将预瞄点从map系转换到car系（获取横向偏差y）
        double y_car = transformToCarFrame(lookahead_point, car_state);

        // 3. 计算转向角 
        // 其中 D 为预瞄距离，L 为轴距
        double delta = asin(2 * wheelbase_ * y_car / (lookahead_distance_ * lookahead_distance_));

        // 4. 限制转向角在物理范围内
        delta = std::clamp(delta, -max_steering_angle_, max_steering_angle_);

        // 5. 发布控制指令（线速度+转向角）
        fsd_common_msgs::ControlCommand cmd;
        cmd.speed = target_speed_;  // 可根据路径曲率动态调整
        cmd.steering_angle = delta;
        control_pub_.publish(cmd);
    }

    // 查找预瞄点
    geometry_msgs::Point findLookaheadPoint(const fsd_common_msgs::CarStateConstPtr& car_state) {
        geometry_msgs::Point car_pos = {car_state->car_state.x, car_state->car_state.y, 0.0};
        double min_dist = 1e9;
        geometry_msgs::Point best_point;

        for (const auto& point : path_points_) {
            double dist = hypot(point.x - car_pos.x, point.y - car_pos.y);
            // 寻找距离最接近预瞄距离的点
            if (fabs(dist - lookahead_distance_) < min_dist) {
                min_dist = fabs(dist - lookahead_distance_);
                best_point = point;
            }
        }
        return best_point;
    }

    // 将map系下的预瞄点转换到car系（获取横向偏差y）
    double transformToCarFrame(const geometry_msgs::Point& map_point, 
                             const fsd_common_msgs::CarStateConstPtr& car_state) {
        geometry_msgs::PointStamped point_in, point_out;
        point_in.header.frame_id = "map";
        point_in.header.stamp = ros::Time::now();
        point_in.point = map_point;

        try {
            // 利用carstate发布的map->car变换进行坐标转换
            tf_buffer_.transform(point_in, point_out, "car", ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return 0.0;
        }

        // car系下的y坐标即为横向偏差（左侧为正，右侧为负）
        return point_out.point.y;
    }
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "bicycle_controller");
    ros::NodeHandle nh;
    BicycleController controller(nh);
    ros::spin();
    return 0;
}