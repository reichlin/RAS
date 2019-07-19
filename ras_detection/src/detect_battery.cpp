#include <ros/ros.h>
#include <math.h>
#include <stdio.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>

#include <fstream>
#include <istream>
#include <string>
#include <iostream>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

int sizeX, sizeY;
int consecutiveSightings;
float resolution;
bool** map = NULL;
int** batteryMap = NULL;
bool mapInit = false;
bool newBattery = false;
visualization_msgs::Marker marker;
visualization_msgs::MarkerArray marker_array;
float max_w_velocity = 0.2;
float angular_velocity = 100.0, linear_velocity;


int marker_count;
geometry_msgs::PoseArray batteryPoses;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformStamped;

//void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) //this is to import directly a pcl point cloud
void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSampledPC2(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2::Ptr cloudTransformed(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr cloudROS(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudUnFiltered(new pcl::PointCloud<pcl::PointXYZ>);

  try
  {
    //std::cout << transformStamped << std::endl;


    pcl::VoxelGrid<pcl::PointXYZ> sor;//sor is the downsampling object
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.007f, 0.007f, 0.007f);
    sor.filter (*cloudDownSampledPC2);

    pcl::toROSMsg(*cloudDownSampledPC2,*cloudROS);//We convert it to a ROS message
    tf2::doTransform(*cloudROS, *cloudTransformed, transformStamped);//this is an ugly solution
    pcl::fromROSMsg(*cloudTransformed, *cloudUnFiltered);//We convert it to a PCL message

    pcl::PassThrough<pcl::PointXYZ> pass;//pass is the filtering object
    pass.setInputCloud(cloudUnFiltered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.055f+0.03f, 0.11f+0.03f);
    pass.filter(*cloudFiltered);

    pcl::VoxelGrid<pcl::PointXYZ> sor2;//sor is the downsampling object
    sor2.setInputCloud (cloudFiltered);
    sor2.setLeafSize (0.02f, 0.02f, 0.3f);
    sor2.filter (*cloudDownSampled);

    //ROS_INFO("cloud size: %d",cloudDownSampled->width);


    for(int i = 0; i < sizeY; ++i) {
      for(int j = 0; j < sizeX; ++j) {
        //if((j<186 && j>(sizeX)/4) && (i<186 && i>(sizeY)/4))
          //std::cout<<(batteryMap[i][j]>0? batteryMap[i][j] :(map[i][j]?9:0));
      }
      //if(i<186 && i>(sizeY)/4)
        //std::cout<<"\n";
    }


    batteryPoses.poses.clear();
    geometry_msgs::Pose bPose;
    for(int i = 0; i < ( cloudDownSampled->width); ++i){
      if(!isnan(cloudDownSampled->points[i].z) && mapInit && fabs(linear_velocity) < 0.004 &&fabs(angular_velocity) < max_w_velocity){//
        if(!map[(int) round((sizeY)/4.0 +  cloudDownSampled->points[i].y/resolution)][ (int) round((sizeX)/4.0 +cloudDownSampled->points[i].x/resolution)])
        {
          if( batteryMap[(int) round((sizeY)/4.0+ cloudDownSampled->points[i].y/resolution)][(int) round((sizeX)/4.0+ cloudDownSampled->points[i].x/resolution)] < 100)
          {
          batteryMap[(int) round((sizeY)/4.0+ cloudDownSampled->points[i].y/resolution)][(int) round((sizeX)/4.0+ cloudDownSampled->points[i].x/resolution)] += 3;
          }
          if(batteryMap[(int) round((sizeY)/4.0+ cloudDownSampled->points[i].y/resolution)][(int) round((sizeX)/4.0+ cloudDownSampled->points[i].x/resolution)] > 15 &&
             batteryMap[(int) round((sizeY)/4.0+ cloudDownSampled->points[i].y/resolution)][(int) round((sizeX)/4.0+ cloudDownSampled->points[i].x/resolution)] < 100)
            newBattery = true;
        }
      }
    }

    for(int i = 0; i < sizeY; ++i) {
      for(int j = 0; j < sizeX; ++j) {
        {
          if(batteryMap[i][j] > 0 && batteryMap[i][j]< 100)
          {
            batteryMap[i][j]--;
          }
        }
      }
    }


    for(int i = 0; i < ( cloudDownSampled->width); ++i){
      if(!isnan(cloudDownSampled->points[i].z) && mapInit && fabs(angular_velocity) < max_w_velocity){//
        if(!map[(int) round((sizeY)/4.0 +  cloudDownSampled->points[i].y/resolution)][ (int) round((sizeX)/4.0 +cloudDownSampled->points[i].x/resolution)])
        {
          if(newBattery && batteryMap[(int) round((sizeY)/4.0+ cloudDownSampled->points[i].y/resolution)][(int) round((sizeX)/4.0+ cloudDownSampled->points[i].x/resolution)] > 3)
          {
            batteryMap[(int) round((sizeY)/4.0+ cloudDownSampled->points[i].y/resolution)][(int) round((sizeX)/4.0+ cloudDownSampled->points[i].x/resolution)] = 100;
            bPose.position.x = cloudDownSampled->points[i].x;
            bPose.position.y = cloudDownSampled->points[i].y;
            batteryPoses.poses.push_back(bPose);

            marker.pose.position.x = cloudDownSampled->points[i].x;
            marker.pose.position.y = cloudDownSampled->points[i].y;
            marker.pose.position.z = cloudDownSampled->points[i].z;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time();
            marker.ns = "node";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::MODIFY;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.080;
            marker.scale.y = 0.080;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.1;
            marker.color.g = 0.1;
            marker.color.b = 0.1;
            marker.id = marker_count ;


          }

        }
      }
    }

    marker_array.markers.push_back(marker);
    marker_count++;
    newBattery = false;

  }


  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void saveCallback(const std_msgs::BoolConstPtr& msg)
{
  std::ofstream outfile("/home/ras28/catkin_ws/src/ras_project/ras_data/battery_map.csv");

  outfile << sizeX << ',';
  outfile << sizeY << ',';

  for(int i = 0; i < sizeY; ++i){
      for(int j=0; j < sizeX; ++j){
          outfile <<batteryMap[i][j] << ',';
      }
  }
  outfile.close();
}

void odomCallback(const geometry_msgs::Pose2DConstPtr& msg)
{
  angular_velocity = msg->theta;
  linear_velocity = msg->theta;
  //ROS_INFO("lin vel: %f",linear_velocity);

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  resolution = msg->info.resolution;
  sizeX = msg->info.width; // number of cells in x dir
  sizeY = msg->info.height; // number of cells in y dir

  int c = 0;
  map = new bool*[sizeY];
  for(int i = 0; i < sizeY; i++) {
    map[i] = new bool[sizeX];
    for(int j = 0; j < sizeX; j++) {
      map[i][j] = (msg->data[c]>94?true:false);
      //if((j<186 && j>58) && (i<186 && i>58))
        //std::cout<<(msg->data[c]>96?"#":".");
      c++;
    }
   // if(i<186 && i>58)
     // std::cout<<"\n";
  }

  if(!mapInit)
  {
    std::ifstream map_file("/home/ras28/catkin_ws/src/ras_project/ras_data/battery_map.csv", std::ifstream::in);
    if(!map_file.fail()) {

      std::string c_x, c_y;
      std::string val;
      getline(map_file, c_x, ',');
      getline(map_file, c_y, ',');

      sizeX = std::atoi(c_x.c_str());
      sizeY = std::atoi(c_y.c_str());

      batteryMap = new int*[sizeY];
      for(int i = 0; i < sizeY; ++i) {
        batteryMap[i] = new int[sizeX];
        for(int j = 0; j < sizeX; ++j) {
          getline(map_file, val, ',');
          batteryMap[i][j] = std::atoi( val.c_str());
        }
      }
      map_file.close();

    } else {

      batteryMap = new int*[sizeY];
      for(int i = 0; i < sizeY; ++i) {
        batteryMap[i] = new int[sizeX];
        for(int j = 0; j < sizeX; ++j) {
          batteryMap[i][j] = 0;
        }
      }
    }
    mapInit = true;
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "detect_battery");
  ros::NodeHandle nh;

  //nh.getParam("/frequency", frequency);//TODO add frecuacy fromp aram srv
  nh.getParam("/battery_mapping/max_w_velocity", max_w_velocity);
  ros::Rate loop_rate(10.0);

  marker_count = 0;

  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber mapSub = nh.subscribe("/map/extended", 1, mapCallback);
  ros::Subscriber odom_sub = nh.subscribe("/robot/position_change", 10, odomCallback);
  ros::Subscriber sub_cloud = nh.subscribe("/camera/depth/points", 1, cloudCallback);
  ros::Subscriber save_sub = nh.subscribe("/master/save", 1, saveCallback);
  ros::Publisher battery_pub = nh.advertise<geometry_msgs::PoseArray>("/battery", 2);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/battery_marker", 2);

  try{
    transformStamped = tfBuffer.lookupTransform( "map", "camera_depth_frame", ros::Time(0), ros::Duration(10.0));
    ROS_INFO("TF Available");
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform camera_link to map: %s", ex.what());
    return 1;
  }

  while(nh.ok()) {
    ros::spinOnce();
    try{
      transformStamped = tfBuffer.lookupTransform( "map", "camera_depth_frame", ros::Time(0), ros::Duration(10.0));
      //ROS_INFO("TF Available");
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could NOT transform camera_link to map: %s", ex.what());
      return 1;
    }

    marker_pub.publish(marker_array);
    if(!batteryPoses.poses.empty())
    {
      battery_pub.publish(batteryPoses);
      ROS_INFO("--------##Rubble Sighted##----------");
    }
    batteryPoses.poses.clear();
    loop_rate.sleep();
  }
}
