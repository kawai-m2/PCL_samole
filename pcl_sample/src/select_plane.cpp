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

//segmentation
    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices);
//segmentationオブジェクトの生成
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
//RANSACにおいて最適化を実施するかどうか
    seg1.setOptimizeCoefficients (true);
//抽出モデルに平面を指定
    seg1.setModelType (pcl::SACMODEL_PLANE);
//抽出モデルにRANSACを指定
    seg1.setMethodType (pcl::SAC_RANSAC);
//許容する誤差しきい値
    seg1.setDistanceThreshold(0.1);
//モデル抽出対象点群のセット
    seg1.setInputCloud(cloud_filtered1);
//モデル抽出の実行
    seg1.segment(*inliers1, *coefficients1);



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

//segmentation
    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
//segmentationオブジェクトの生成
    pcl::SACSegmentation<pcl::PointXYZRGB> seg2;
//RANSACにおいて最適化を実施するかどうか
    seg2.setOptimizeCoefficients (true);
//抽出モデルに平面を指定
    seg2.setModelType (pcl::SACMODEL_PLANE);
//抽出モデルにRANSACを指定
    seg2.setMethodType (pcl::SAC_RANSAC);
//許容する誤差しきい値
    seg2.setDistanceThreshold(0.1);
//モデル抽出対象点群のセット
    seg2.setInputCloud(cloud_filtered2);
//モデル抽出の実行
    seg2.segment(*inliers2, *coefficients2);


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

//segmentation
    pcl::ModelCoefficients::Ptr coefficients3 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers3 (new pcl::PointIndices);
//segmentationオブジェクトの生成
    pcl::SACSegmentation<pcl::PointXYZRGB> seg3;
//RANSACにおいて最適化を実施するかどうか
    seg3.setOptimizeCoefficients (true);
//抽出モデルに平面を指定
    seg3.setModelType (pcl::SACMODEL_PLANE);
//抽出モデルにRANSACを指定
    seg3.setMethodType (pcl::SAC_RANSAC);
//許容する誤差しきい値
    seg3.setDistanceThreshold(0.1);
//モデル抽出対象点群のセット
    seg3.setInputCloud(cloud_filtered3);
//モデル抽出の実行
    seg3.segment(*inliers3, *coefficients3);

pcl::PointCloud<pcl::PointXYZRGB> seg_cloud1;
seg_cloud1 = *cloud_filtered1;
pcl::PointCloud<pcl::PointXYZRGB> seg_cloud2;
seg_cloud2 = *cloud_filtered2;
pcl::PointCloud<pcl::PointXYZRGB> seg_cloud3;
seg_cloud3 = *cloud_filtered3;

 for (size_t i = 0; i < inliers1->indices.size (); ++i) {
             std::cerr << inliers1->indices[i] << "    " << seg_cloud1.points[inliers1->indices[i]].x << " "
             << seg_cloud1.points[inliers1->indices[i]].y << " "
             << seg_cloud1.points[inliers1->indices[i]].z << std::endl;

            seg_cloud1.points[inliers1->indices[i]].r = 255;
            seg_cloud1.points[inliers1->indices[i]].g = 0;
            seg_cloud1.points[inliers1->indices[i]].b = 0;
        }

 for (size_t i = 0; i < inliers2->indices.size (); ++i) {
            // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
            // << cloud.points[inliers->indices[i]].y << " "
            // << cloud.points[inliers->indices[i]].z << std::endl;
            seg_cloud2.points[inliers2->indices[i]].r = 0;
            seg_cloud2.points[inliers2->indices[i]].g = 255;
            seg_cloud2.points[inliers2->indices[i]].b = 0;
        }

 for (size_t i = 0; i < inliers3->indices.size (); ++i) {
            // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
            // << cloud.points[inliers->indices[i]].y << " "
            // << cloud.points[inliers->indices[i]].z << std::endl;
            seg_cloud3.points[inliers3->indices[i]].r = 0;
            seg_cloud3.points[inliers3->indices[i]].g = 0;
            seg_cloud3.points[inliers3->indices[i]].b = 255;
        }

    sensor_msgs::PointCloud2 output1;  
    sensor_msgs::PointCloud2 output2;  
    sensor_msgs::PointCloud2 output3;  
    pcl::toROSMsg(seg_cloud1, output1);
    pcl::toROSMsg(seg_cloud2, output2);
    pcl::toROSMsg(seg_cloud3, output3);

    through1_pub.publish (output1);
    through2_pub.publish (output2);
    through3_pub.publish (output3);
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
