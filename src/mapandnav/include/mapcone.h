#ifndef _CONES_H_
#define _CONES_H_
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "fsd_common_msgs/Cone.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/point_types.h"
#include "pcl-1.10/pcl/kdtree/kdtree_flann.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include <string>
struct cone
{
    geometry_msgs::Point position;
    std::string color;
    ros::Time timestamp;
    int cnt;
};

class MapCone
{
    std::vector<cone> cones;
    nav_msgs::Path path;
    std::vector<geometry_msgs::Point> track_centers;
    double distance_threshold;
    double track_width;
    ros::Publisher marker_pub;    ros::Publisher path_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
public:
    MapCone(ros::NodeHandle &, double, double);
    // 添加锥筒
    void addPoints(const std::vector<fsd_common_msgs::Cone> &conesAdd);
    // 清除误检测锥筒
    void clear();
    // 发布路径
    void pubpath();
    // 可视化锥筒和路径点
    void vis();
};
#endif