#include "ros/ros.h"
#include "fsd_common_msgs/ConeDetections.h"
#include "fsd_common_msgs/CarState.h"
#include "pcl-1.10/pcl/kdtree/kdtree_flann.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/point_types.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"
#include "mapcone.h"
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map_and_nav_node");
    ros::NodeHandle nh;
    MapCone mapcone(nh, 2);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Subscriber conesub = nh.subscribe<fsd_common_msgs::ConeDetections>("/perception/lidar/cone_detections", 10, [&](const fsd_common_msgs::ConeDetectionsConstPtr &msg){
        fsd_common_msgs::ConeDetections transformed_msg = *msg;
        transformed_msg.header.frame_id = "map";
        
        // Get the transform once for this frame
        geometry_msgs::TransformStamped transform;
        try {
            // rslidar -> map
            transform = tf_buffer.lookupTransform("map", msg->header.frame_id, ros::Time::now(), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform lookup failed: %s", ex.what());
            return;
        }

        // Apply the same transform to all cones
        for (auto& cone : transformed_msg.cone_detections) {
            geometry_msgs::PointStamped point_in, point_out;
            point_in.header = msg->header;
            point_in.point.x = cone.position.x;
            point_in.point.y = cone.position.y;
            point_in.point.z = cone.position.z;
            cone.color.data = cone.position.y>=-1?"r":"b";
            tf2::doTransform(point_in, point_out, transform);
            
            cone.position.x = point_out.point.x;
            cone.position.y = point_out.point.y;
            cone.position.z = point_out.point.z;
        }
        mapcone.addPoints(transformed_msg.cone_detections);
    });
    ros::spin();
    return 0;
}