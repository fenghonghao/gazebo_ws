#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "fsd_common_msgs/ConeDetections.h"
#include "fsd_common_msgs/Cone.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class LidarProcessor {
public:
    LidarProcessor(ros::NodeHandle& nh) {
        pub_ = nh.advertise<fsd_common_msgs::ConeDetections>(
            "/perception/lidar/cone_detections", 10);
        sub_ = nh.subscribe("/perception/lidar/point_cloud", 10, 
            &LidarProcessor::cloudCallback, this);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        fsd_common_msgs::ConeDetections cone_msg;
        cone_msg.header = cloud_msg->header;

        // 简单聚类：距离小于0.5m的点视为同一个锥桶
        for (const auto& point : cloud.points) {
            // 过滤无效点
            if (std::isnan(point.x) || std::isnan(point.y)) continue;

            fsd_common_msgs::Cone cone;
            cone.position.x = point.x;
            cone.position.y = point.y;
            cone.position.z = point.z;
            if (cone.position.y > 0.5) {  // 右侧红色锥桶
                cone.color = fsd_common_msgs::Cone::COLOR_RED;
            } else if (cone.position.y < -0.5) {  // 左侧蓝色锥桶
                cone.color = fsd_common_msgs::Cone::COLOR_BLUE;
            } else {
                cone.color = fsd_common_msgs::Cone::COLOR_YELLOW; 
            }
            cone_msg.cone_detections.push_back(cone);
        }

        pub_.publish(cone_msg);
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "lidar_processor");
    ros::NodeHandle nh;
    LidarProcessor processor(nh);
    ros::spin();
    return 0;
}