#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
tf::TransformListener *tf_listener;
//std::string target_frame_("world_ned");
//std::string camera_frame_("frontcolor");
ros::Publisher pub;
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input_)
{
    static int count = 1;
    if (count != 5) {
        count++;
        return;
    }
    sensor_msgs::PointCloud2 input = *input_;
    input.header.frame_id = "frontcolor";
    //input.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2 output;
    tf_listener->waitForTransform("world_ned", "frontcolor", ros::Time::now(), ros::Duration(10.0));
    pcl_ros::transformPointCloud("world_ned", input, output, *tf_listener);

    pub.publish(output);
    count = 1;
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;
    tf_listener = new tf::TransformListener();

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/cloud_transformed", 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/frontr200/camera/depth_registered/points", 1, cloud_callback);

    ros::spin();
    return 0;

}