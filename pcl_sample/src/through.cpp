#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>

ros::Publisher through1_pub;
ros::Publisher through2_pub;
ros::Publisher through3_pub;


void PCL_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);


/////////filter1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass1;
    pass1.setInputCloud (cloud);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (-10, 30);
    pass1.filter (*cloud_filtered1);
    pass1.setInputCloud (cloud_filtered1);
    pass1.setFilterFieldName ("y");
    pass1.setFilterLimits (-2, -0.5);
    pass1.filter (*cloud_filtered1);
    pass1.setInputCloud (cloud_filtered1);
    pass1.setFilterFieldName ("z");
    pass1.setFilterLimits (0, 65);
    //pass.setFilterLimitsNegative (true);
    pass1.filter (*cloud_filtered1);
    sensor_msgs::PointCloud2 output1;  
    pcl::toROSMsg(*cloud_filtered1, output1);

/////////filter2
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass2;
    pass2.setInputCloud (cloud);
    pass2.setFilterFieldName ("x");
    pass2.setFilterLimits (-10, 30);
    pass2.filter (*cloud_filtered2);
    pass2.setInputCloud (cloud_filtered2);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-0.5, 0.5);
    pass2.filter (*cloud_filtered2);
    pass2.setInputCloud (cloud_filtered2);
    pass2.setFilterFieldName ("z");
    pass2.setFilterLimits (0, 65);
    //pass.setFilterLimitsNegative (true);
    pass2.filter (*cloud_filtered2);
    sensor_msgs::PointCloud2 output2;  
    pcl::toROSMsg(*cloud_filtered2, output2);

//////////fliter3
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass3;
    pass3.setInputCloud (cloud);
    pass3.setFilterFieldName ("x");
    pass3.setFilterLimits (-10, 30);
    pass3.filter (*cloud_filtered3);
    pass3.setInputCloud (cloud_filtered3);
    pass3.setFilterFieldName ("y");
    pass3.setFilterLimits (0.5, 1);
    pass3.filter (*cloud_filtered3);
    pass3.setInputCloud (cloud_filtered3);
    pass3.setFilterFieldName ("z");
    pass3.setFilterLimits (0, 65);
    //pass.setFilterLimitsNegative (true);
    pass3.filter (*cloud_filtered3);
    sensor_msgs::PointCloud2 output3;  
    pcl::toROSMsg(*cloud_filtered3, output3);

////publish
    through1_pub.publish (output1);
    through2_pub.publish (output2);
    through3_pub.publish (output3);

    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double max_z;

    for(int i = 0; i < cloud->points.size(); i++){
        if(max_z > cloud->points[i].z)
            max_z = max_z;
        else
            max_z = cloud->points[i].z;

        if(max_y > cloud->points[i].y)
            max_y = max_y;
        else
            max_y = cloud->points[i].y;

        if(max_x > cloud->points[i].x)
            max_x = max_x;
        else
            max_x = cloud->points[i].x;

        if(min_z < cloud->points[i].z)
            min_z = min_z;
        else
            min_z = cloud->points[i].z;

        if(min_y < cloud->points[i].y)
            min_y = min_y;
        else
            min_y = cloud->points[i].y;

        if(min_x < cloud->points[i].x)
            min_x = min_x;
        else
            min_x = cloud->points[i].x;
    }

    ROS_INFO("x : min%f max%f",min_x,max_x);
    ROS_INFO("y : min%f max%f",min_y,max_y);
    ROS_INFO("z : min%f max%f",min_z,max_z);
}

int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "through");
    ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("camera/depth/color/points", 1, PCL_callback);
  
  // Create a ROS publisher for the output point cloud
    through1_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl/through/1", 1);
    through2_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl/through/2", 1);
    through3_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl/through/3", 1);
  // Spin
  ros::spin ();
}
