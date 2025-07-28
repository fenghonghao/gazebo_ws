#include "ros/ros.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

class BicycleController {
public:
    BicycleController(ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {
        // 订阅车辆状态和路径点
        car_state_sub_ = nh_.subscribe("/racecar/rear_pose", 10, 
            &BicycleController::carStateCallback, this);
        path_sub_ = nh_.subscribe("/path", 10,  
            &BicycleController::pathCallback, this);
        finished = 0;
        lastidx = 0; // 上一个育苗点索引
        // 发布控制指令（自行车模型：速度+转向角）
        control_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 加载参数（轴距、最大转向角等）
        nh_.param("wheelbase", wheelbase_, 2.6);  // 轴距（米）
        nh_.param("max_steering_angle", max_steering_angle_, 0.0523);  // 最大转向角（30度，弧度）
        nh_.param("lookahead_distance", lookahead_distance_, 5.0);  // 预瞄距离（米）
        nh_.param("target_speed", target_speed_, 5.0);  // 目标线速度（米/秒）
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
    bool finished;
    int lastidx;
    // 路径点回调
    void pathCallback(const nav_msgs::PathConstPtr& msg) {
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.pose.position);
        }
    }


    // 车辆状态回调
    void carStateCallback(const geometry_msgs::PoseStampedConstPtr& car_state) {
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
        if(finished == 1) {
            ROS_ERROR("Finished path tracking");
            geometry_msgs::Twist control_msg;
            control_msg.linear.x = 0.0;
            control_msg.angular.z = 0.0;
            control_pub_.publish(control_msg);
            return;
        }
        // 2. 将预瞄点从map系转换到base_link系（获取横向偏差y）
        double y_car = transformToCarFrame(lookahead_point, car_state);

        // 3. 计算转向角 
        // 其中 D 为预瞄距离，L 为轴距
        double delta = asin(2 * wheelbase_ * y_car / (lookahead_distance_ * lookahead_distance_));

        // 4. 限制转向角在物理范围内
        delta = std::max(-max_steering_angle_, std::min(delta, max_steering_angle_));

        // 5. 发布控制指令（线速度+转向角）
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = target_speed_;
        control_msg.angular.z = target_speed_ * tan(delta) / wheelbase_;
        control_pub_.publish(control_msg);
    }

    // 查找预瞄点
    geometry_msgs::Point findLookaheadPoint(const geometry_msgs::PoseStampedConstPtr& car_state) {
        geometry_msgs::Point car_pos;
        car_pos.x = car_state->pose.position.x;
        car_pos.y = car_state->pose.position.y;
        car_pos.z = 0.0;
        double min_dist = 1e9;
        geometry_msgs::Point best_point;

        if(lastidx >= path_points_.size()) {
            return best_point;  // 如果上次的索引超出范围，直接返回
        }
        ROS_INFO("lastidx: %d, path size: %ld", lastidx, path_points_.size());
        for (int i=lastidx; i < path_points_.size(); ++i) {
            double dist = hypot(path_points_[i].x - car_pos.x, path_points_[i].y - car_pos.y);
            // 寻找距离最接近预瞄距离的点
            ROS_INFO("idx: %d, dist: %f", i, dist);
            
            if (fabs(dist - lookahead_distance_) < min_dist) {
                min_dist = fabs(dist - lookahead_distance_);
                ROS_INFO("mini_dist: %f, lookahead_distance: %f", min_dist, lookahead_distance_);
                best_point = path_points_[i];
                lastidx = i;
                if (i >= 15 )
                {
                    finished = 1;
                    break;
                }
            }
        }
        return best_point;
    }

    // 将map系下的预瞄点转换到car系（获取横向偏差y）
    double transformToCarFrame(const geometry_msgs::Point& map_point, 
                             const geometry_msgs::PoseStampedConstPtr& car_state) {
        geometry_msgs::PointStamped point_in, point_out;
        point_in.header.frame_id = "map";
        point_in.header.stamp = ros::Time::now();
        point_in.point = map_point;

        try {
            // 利用carstate发布的map->base_link变换进行坐标转换
            tf_buffer_.transform(point_in, point_out, "base_footprint", ros::Duration(0.1));
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