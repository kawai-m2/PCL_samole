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
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//segmentationオブジェクトの生成
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//RANSACにおいて最適化を実施するかどうか
    seg.setOptimizeCoefficients (true);
//抽出モデルに平面を指定
    seg.setModelType (pcl::SACMODEL_PLANE);
//抽出モデルにRANSACを指定
    seg.setMethodType (pcl::SAC_RANSAC);
//許容する誤差しきい値
    seg.setDistanceThreshold(0.1);
//モデル抽出対象点群のセット
    seg.setInputCloud(cloud);
//モデル抽出の実行
    seg.segment(*inliers, *coefficients);

//inliners  検出した平面の点群
//coefficients 平面モデル（方程式）の変数
//ax+by+cz+d=0

pcl::PointCloud<pcl::PointXYZRGB> test;
test = *cloud;


/*if(inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given data" << std::endl;
}
else{*/
    //抽出したpointの部分だけ色づけ
        for (size_t i = 0; i < inliers->indices.size (); ++i) {
            // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
            // << cloud.points[inliers->indices[i]].y << " "
            // << cloud.points[inliers->indices[i]].z << std::endl;
            test.points[inliers->indices[i]].r = 255;
            test.points[inliers->indices[i]].g = 0;
            test.points[inliers->indices[i]].b = 0;
        }
    //}
ROS_INFO("coefficients : [0, 1, 2, 3] = [%5.3lf, %5.3lf, %5.3lf, %5.3lf]",
             coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
sensor_msgs::PointCloud2 output;  
pcl::toROSMsg(test, output);

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
