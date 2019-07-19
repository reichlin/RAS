#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

bool obstacle;

void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  obstacle = false;
  for(int i = 246; i < 294; ++i)//70 degrees aperture, 90 degree tf rotation of lidar
  {
    if((isinf(msg->ranges[i])?msg->range_max:msg->ranges[i]) < 0.14){//the ternary condition sanitizes the input from inf's
      //STAHP!!
      ROS_INFO("I Feel Something Near me :/ , at: %f meters and angle: %d ",msg->ranges[i],i);
      obstacle = true;
    }
  }
  /*
  for(int i = 56; i < 124; ++i)//70 degrees aperture, 90 degree tf rotation of lidar
  {
    if((isinf(msg->ranges[i])?msg->range_max:msg->ranges[i]) < 0.14){//the ternary condition sanitizes the input from inf's
      //STAHP!!
      ROS_INFO("I Feel Something Near me :/ , at: %f meters and angle: %d ",msg->ranges[i],i);
      obstacle = true;
    }
  }
  */
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_avoidance");
  ros::NodeHandle nh;
  ros::Publisher inminentColision = nh.advertise<std_msgs::Bool>("/inminentColision", 1);
  std_msgs::Bool msg;
  ros::Subscriber lidarSubscriber = nh.subscribe("/scan", 1, lidarCallback);


  ros::Rate loop_rate(15);//TODO ask this to param server

  while (ros::ok() && nh.ok())
  {
    msg.data = obstacle;
    inminentColision.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
