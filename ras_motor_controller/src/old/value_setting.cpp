#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "phidgets/motor_params.h"
#include "phidgets/motor_encoder.h"
#include <pid/pid.h>
#include <geometry_msgs/Twist.h>

std::string motorr_name = "rightmotor";
std::string motorl_name = "leftmotor";

double v = 0.0;
double w = 0.0;
double estimated_r = 0.0;
double estimated_l = 0.0;
double desired_r = 0.0;
double desired_l = 0.0;

double frequency = 50.0;
double ticks = 1.0;

double b = 1.0;
double r = 1.0;

//Callback when something is published on 'set_velocity'
void setVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  v = msg->linear.x / 40.0; //TODO delete the: /40
  w = msg->angular.z;
  
  desired_r = (v + (b/2.0)*w)/r;
  desired_l = (v - (b/2.0)*w)/r;
}

void motorrCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  estimated_r = (msg->count_change * 2.0 * M_PI * frequency) / ticks;
}

void motorlCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  estimated_l = (msg->count_change * 2.0 * M_PI * frequency) / ticks;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_value");
  ros::NodeHandle nh("~");

  nh.getParam("/robot/motor/left/name", motorl_name);
  nh.getParam("/robot/motor/right/name", motorr_name);
  nh.getParam("/frequency_mc", frequency);
  nh.getParam("/robot/wheels_radius", r);
  nh.getParam("/robot/base", b);
  nh.getParam("/robot/ticks_per_rev", ticks);

  ros::Rate loop_rate(frequency);

  ros::Subscriber sub_vel = nh.subscribe("/set_velocity", 10, setVelocityCallback);
  ros::Subscriber sub_motor1 = nh.subscribe(motorr_name + "/encoder", 100, motorrCallback);
  ros::Subscriber sub_motor2 = nh.subscribe(motorl_name + "/encoder", 100, motorlCallback);

  ros::Publisher setpoint_r_pub = nh.advertise<std_msgs::Float64>("/setpoint_r", 10);
  ros::Publisher setpoint_l_pub = nh.advertise<std_msgs::Float64>("/setpoint_l", 10);
  ros::Publisher state_r_pub = nh.advertise<std_msgs::Float64>("/state_r", 10);
  ros::Publisher state_l_pub = nh.advertise<std_msgs::Float64>("/state_l", 10);

  std_msgs::Float64 setpoint_r;
  std_msgs::Float64 setpoint_l;
  std_msgs::Float64 state_r;
  std_msgs::Float64 state_l;

  while (ros::ok() && nh.ok())
  { 
    ros::spinOnce();

    setpoint_r.data = desired_r;
    setpoint_l.data = desired_l;
    state_r.data = estimated_r;
    state_l.data = estimated_l;

    setpoint_r_pub.publish(setpoint_r);
    state_r_pub.publish(state_r);
    setpoint_l_pub.publish(setpoint_l);  
    state_l_pub.publish(state_l);

    loop_rate.sleep();
  }
}
