#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>

float startPointX = 0.0;
float startPointY = 0.0;
float objectX;
float objectY;
bool foundObject = false;

class Master
{
public:  
  //ros::Subscriber oneSubscriber;
  ros::Publisher onePublisher;
  ros::Subscriber position_sub;
  ros::Subscriber object_position_sub;
  ros::NodeHandle nh;

  Master()
  {
    nh = ros::NodeHandle("~"); //~ private node Handle
    position_sub = nh.subscribe("/robot/position", 1, &Master::posCallback, this);
    object_position_sub = nh.subscribe("/visualization_marker", 1, &Master::objCallback, this);
    onePublisher = nh.advertise<geometry_msgs::PoseArray>("/path", 1);
    // oneSubscriber = nh.subscribe("/hiagain", 1, &Master::oneCallback, this);
  }

  void posCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
  {
    startPointX = msg->x;
    startPointY = msg->y;
  }
  
  void objCallback(const visualization_msgs::Marker::ConstPtr& msg)
  {
    ROS_INFO("%f", msg->pose.position.x);
    if(!isnan(msg->pose.position.x) && !isnan(msg->pose.position.y))
    {
      objectX = (msg->pose.position.x) - 0.06 * cos(tan((msg->pose.position.y - startPointY) / (msg->pose.position.x - startPointX)));
      objectY = msg->pose.position.y - 0.06 * sin(tan((msg->pose.position.y - startPointY) / (msg->pose.position.x - startPointX)));
      foundObject = true;
    }
  }

  //private:
  // geometry_msgs::Twist msg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");
  Master master;
  ros::Rate loop_rate(0.1);
  ros::Rate loop_rate2(10);
  
  //while (ros::ok())
  //{
  
  geometry_msgs::PoseArray samplePath;
  geometry_msgs::Pose node;

  loop_rate.sleep();

  ROS_INFO("master has been created");
  
  while(!foundObject)
  {
    ROS_INFO("master is waiting");
    ros::spinOnce();
    loop_rate2.sleep();
  }

  ROS_INFO("master has a goal");

  samplePath.poses.clear();

  node.position.x = startPointX;
  node.position.y = startPointY;
  samplePath.poses.push_back(node);

  node.position.x = objectX;
  node.position.y = objectY;
  samplePath.poses.push_back(node);

  node.position.x = startPointX;
  node.position.y = startPointY;
  samplePath.poses.push_back(node);

  /*
node.position.x = 1.8;
node.position.y = 2.2;
samplePath.poses.push_back(node);

node.position.x = 1.8;
node.position.y = 1.0;
samplePath.poses.push_back(node);

*/

  master.onePublisher.publish(samplePath);
  ROS_INFO("I'm sorry, Dave. I'm afraid I can't do that");
  //while (ros::ok())
  //{
  ros::spinOnce();
  loop_rate.sleep();
  //  }

  return 0;
}
