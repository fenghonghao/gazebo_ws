#ifndef _CONES_H_
#define _CONES_H_
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "fsd_common_msgs/Cone.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/point_types.h"
#include "pcl-1.10/pcl/kdtree/kdtree_flann.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
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
    double distance_threshold;
    ros::Publisher marker_pub;
public:
    MapCone(ros::NodeHandle &_nh, double distance_threshold = 0.3) : distance_threshold(distance_threshold)
    {
        marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    }
    void addPoints(const std::vector<fsd_common_msgs::Cone> &conesAdd)
    {
        for (auto coneadd : conesAdd)
        {
            double minidis = 1e18;
            int idx = -1;
            for (int i = 0; i < cones.size(); ++i)
            {
                double dx = coneadd.position.x - cones[i].position.x;
                double dy = coneadd.position.y - cones[i].position.y;
                double dis = sqrt(dx * dx + dy * dy);
                if (dis < minidis)
                {
                    minidis = dis;
                    idx = i;
                }
            }
            if (minidis < distance_threshold)
            {
                cones[idx].position.x = (coneadd.position.x + cones[idx].position.x) / 2;
                cones[idx].position.y = (coneadd.position.y + cones[idx].position.y) / 2;
                cones[idx].cnt ++;
            }
            else cones.push_back({coneadd.position, coneadd.color.data, ros::Time::now(), 1});
        }
        vis();
        clear();
        ROS_INFO("Cone size: %ld", cones.size());
    }
    void clear()
    {
        for(int i=0;i<cones.size();++i)
        {
            for(int j=i+1;j<cones.size();++j)
            {
                double dx = cones[i].position.x - cones[j].position.x;
                double dy = cones[i].position.y - cones[j].position.y;
                double dis = sqrt(dx * dx + dy * dy);
                if(dis < distance_threshold)
                {
                    cones[i].position.x = (cones[i].position.x + cones[j].position.x) / 2;
                    cones[i].position.y = (cones[i].position.y + cones[j].position.y) / 2;
                    cones[i].color = cones[i].cnt > cones[j].cnt ? cones[i].color : cones[j].color;
                    cones[i].cnt += cones[j].cnt;
                    cones.erase(cones.begin() + j);
                    j--;
                }
            }
        }
        for(int i=0;i<cones.size();++i)
        {
            ROS_INFO("Cone[%d]: x=%.2f, y=%.2f, color=%s, cnt=%d", i, cones[i].position.x, cones[i].position.y, cones[i].color.c_str(), cones[i].cnt);
            if(cones[i].cnt < 12 && ros::Time::now() - cones[i].timestamp > ros::Duration(2.0))
            {
                cones.erase(cones.begin() + i);
                i--;
            }
        }
    }
    void vis()
    {
        int cntb = 0, cntr = 0, cntu = 0;
        visualization_msgs::MarkerArray marker_array;
        std_msgs::Header header;
        header.frame_id = "map";
        header.stamp = ros::Time::now();

        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.id = 0;
        clear_marker.ns = "Cone";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for(int i = 0; i < cones.size(); ++i)
        {
            const auto& cone = cones[i];
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "Cone";
            marker.id = i + 1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = cone.position;
            // 初始化四元数，否则rviz会报Warn
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;
            // 模型显示大小
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            // marker.lifetime = ros::Duration(1);
            if(cone.color == "b")
            {
                cntb++;
                marker.mesh_resource = "package://mapandnav/meshes/blue.stl";
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            else if(cone.color == "r")
            {
                cntr++;
                marker.mesh_resource = "package://mapandnav/meshes/red.stl";
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
            }
            else if(cone.color == "u")
            {
                cntu++;
                marker.mesh_resource = "package://mapandnav/meshes/red.stl";
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            marker_array.markers.push_back(marker);
        }

        ROS_INFO("frame_id=%s", header.frame_id.c_str());
        ROS_INFO("All:%ld",cones.size());
        ROS_INFO("Blue:%d",cntb);
        ROS_INFO("Red:%d",cntr);
        ROS_INFO("Other:%d",cntu);
        marker_pub.publish(marker_array);
    }
};
#endif