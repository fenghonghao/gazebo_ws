#include "mapcone.h"

MapCone::MapCone(ros::NodeHandle &_nh, double distance_threshold = 0.3, double track_width = 3.2)
 : distance_threshold(distance_threshold), tfListener(tfBuffer), track_width(track_width)
{
    marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    path_pub = _nh.advertise<nav_msgs::Path>("/path", 10);
}
void MapCone::addPoints(const std::vector<fsd_common_msgs::Cone> &conesAdd)
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
            cones[idx].cnt++;
        }
        else
            cones.push_back({coneadd.position, coneadd.color.data, ros::Time::now(), 1});
    }
    vis();
    clear();
    pubpath();
    ROS_INFO("Cone size: %ld", cones.size());
}
void MapCone::clear()
{
    for (int i = 0; i < cones.size(); ++i)
    {
        for (int j = i + 1; j < cones.size(); ++j)
        {
            double dx = cones[i].position.x - cones[j].position.x;
            double dy = cones[i].position.y - cones[j].position.y;
            double dis = sqrt(dx * dx + dy * dy);
            if (dis < distance_threshold)
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
    for (int i = 0; i < cones.size(); ++i)
    {
        // ROS_INFO("Cone[%d]: x=%.2f, y=%.2f, color=%s, cnt=%d", i, cones[i].position.x, cones[i].position.y, cones[i].color.c_str(), cones[i].cnt);
        if (cones[i].cnt < 12 && ros::Time::now() - cones[i].timestamp > ros::Duration(2.0))
        {
            cones.erase(cones.begin() + i);
            i--;
        }
    }
}
void MapCone::pubpath()
{
    visualization_msgs::MarkerArray marker_array;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    path.poses.clear();
    std::vector<int> leftcones, rightcones;
    track_centers.clear();
    for (int i = 0; i < cones.size(); ++i)
    {
        if (cones[i].color == "r")
        {
            leftcones.push_back(i);
        }
        else if (cones[i].color == "b")
        {
            rightcones.push_back(i);
        }
    }
    int cnt = 0;
    for (int lidx : leftcones)
    {
        for (int ridx : rightcones)
        {
            double dx = cones[lidx].position.x - cones[ridx].position.x;
            double dy = cones[lidx].position.y - cones[ridx].position.y;
            double dis = sqrt(dx * dx + dy * dy);
            if (abs(track_width - dis) <= distance_threshold / 15)
            {
                geometry_msgs::Point center;
                center.x = (cones[lidx].position.x + cones[ridx].position.x) / 2;
                center.y = (cones[lidx].position.y + cones[ridx].position.y) / 2;

                // 计算方向向量
                double dir_x = cones[ridx].position.x - cones[lidx].position.x;
                double dir_y = cones[ridx].position.y - cones[lidx].position.y;
                double yaw = atan2(dir_y, dir_x);

                // tf2有根据角轴或者欧拉角初始化的构造函数
                tf2::Quaternion point_direction(tf2::Vector3(0, 0, 1), yaw + M_PI_2);

                // 创建路径点
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position = center;
                pose.pose.orientation = tf2::toMsg(point_direction);
                path.poses.push_back(pose);
            }
        }
        ROS_WARN("centers: %d\n", cnt);
    }

    // 由近到元排序
    sort(path.poses.begin(), path.poses.end(), [](const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
         { return hypot(a.pose.position.x, a.pose.position.y) < hypot(b.pose.position.x, b.pose.position.y); });
    path_pub.publish(path);
}
void MapCone::vis()
{
    int cntb = 0, cntr = 0;
    visualization_msgs::MarkerArray marker_array;
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();

    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    for (int i = 0; i < cones.size(); ++i)
    {
        const auto &cone = cones[i];
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
        marker.mesh_resource = "package://mapandnav/meshes/blue.stl";
        if (cone.color == "b")
        {
            cntb++;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        else if (cone.color == "r")
        {
            cntr++;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        marker_array.markers.push_back(marker);
    }
    // ROS_INFO("frame_id=%s", header.frame_id.c_str());
    // ROS_INFO("All:%ld",cones.size());
    // ROS_INFO("Blue:%d",cntb);
    // ROS_INFO("Red:%d",cntr);

    for (int i = 0; i < path.poses.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "Path";
        marker.id = i + 1 + cones.size();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = path.poses[i].pose;
        marker.pose.orientation = path.poses[i].pose.orientation;
        // 模型显示大小
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color.a = 1.0f;
        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
}