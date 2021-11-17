#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/openni_grabber.h>
ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
   
//edge
pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);

pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::Normal, pcl::Label> oed;
   
  oed.setInputNormals (normal);
  oed.setInputCloud (cloud);
  oed.setDepthDisconThreshold (0.1);
  oed.setMaxSearchNeighbors (10);
  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.compute (labels, label_indices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  bbb (new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::copyPointCloud(*cloud, label_indices[4].indices, *bbb);

pcl::PointCloud<pcl::PointXYZRGB> aaa;
aaa=*bbb;
 for (size_t i = 0; i < aaa.size (); ++i) {
            // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
            // << cloud.points[inliers->indices[i]].y << " "
            // << cloud.points[inliers->indices[i]].z << std::endl;
            aaa.points[i].r = 255;
            aaa.points[i].g = 0;
            aaa.points[i].b = 0;
        }

sensor_msgs::PointCloud2 output;  
pcl::toROSMsg(aaa, output);

pub.publish(output);

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the model coefficients
pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl/through/1", 1);
  // Spin
  ros::spin ();
}



































/*#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/openni_grabber.h>
ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
   
//edge
pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);

pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::Normal, pcl::Label> oed;
   
  oed.setInputNormals (normal);
  oed.setInputCloud (cloud);
  oed.setDepthDisconThreshold (0.1);
  oed.setMaxSearchNeighbors (10);
  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.compute (labels, label_indices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  bbb (new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::copyPointCloud(*cloud, label_indices[4].indices, *bbb);

pcl::PointCloud<pcl::PointXYZRGB> aaa;
aaa=*bbb;
 for (size_t i = 0; i < aaa.size (); ++i) {
            // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
            // << cloud.points[inliers->indices[i]].y << " "
            // << cloud.points[inliers->indices[i]].z << std::endl;
            aaa.points[i].r = 255;
            aaa.points[i].g = 0;
            aaa.points[i].b = 0;
        }

sensor_msgs::PointCloud2 output;  
pcl::toROSMsg(aaa, output);

pub.publish(output);

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the model coefficients
pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl/through/1", 1);
  // Spin
  ros::spin ();
}
*/