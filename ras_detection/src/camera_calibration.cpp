#include <ros/ros.h>
#include <math.h>
#include <stdio.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformStamped;

//void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) //this is to import directly a pcl point cloud
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  sensor_msgs::PointCloud2  cloudTransformed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudUnFiltered(new pcl::PointCloud<pcl::PointXYZ>);

  try{
    tf2::doTransform(*cloud, cloudTransformed, transformStamped);
    pcl::fromROSMsg(cloudTransformed, *cloudUnFiltered);//We convert it to a PCL message
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloudUnFiltered);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0){
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    std::cout << "Eq: " << coefficients->values[0] << " X + "
              << coefficients->values[1] << " Y + "
              << coefficients->values[2] << " Z + "
              << coefficients->values[3] << " = 0 "<<std::endl;

    std::cout << "Pitch: " << -180*(coefficients->values[0]/coefficients->values[2])/M_PI << " º" << std::endl;
    std::cout << "Roll: " << -180*(coefficients->values[1]/coefficients->values[2])/M_PI << " º" << std::endl;
    std::cout << "Height: " << -(coefficients->values[3]/coefficients->values[2]) << " m" << std::endl;
    std::cout << "Plane model inliers: " << inliers->indices.size () << std::endl << std::endl;

  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "detect_battery");
  ros::NodeHandle nh;

  ros::Rate loop_rate(20.0);

  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber sub_cloud = nh.subscribe("/camera/depth/points", 1, cloudCallback);

  try{
    transformStamped = tfBuffer.lookupTransform( "base_link", "camera_depth_frame", ros::Time(0), ros::Duration(10.0));
    ROS_INFO("TF Available");
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform camera_link to map: %s", ex.what());
    return 1;
  }

  while(nh.ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}
