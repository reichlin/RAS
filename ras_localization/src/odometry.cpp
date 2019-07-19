#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include "phidgets/motor_encoder.h"
#include <geometry_msgs/Pose2D.h>

std::string motorr_name = "/right_wheel/right_motor";
std::string motorl_name = "/left_wheel/left_motor";

int delta_encoder_r = 0;
int delta_encoder_l = 0;
int encoder_r = 0;
int encoder_l = 0;

double x_b = 0.0;
double y_b = 0.0;
double yaw_b = 3.14;
double odomx = 2.15, odomy = 0.25, odomtheta;
double odomtheta2 = 1.57;//TODO DELETE this

double dt = 1.0/30999.0;
double frequency = 30999.0;
double r = 9991.0;
double b = 9991.0;
double ticks = 360999.0;

void motorrCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  if(encoder_r == 0)
    encoder_r = msg->count;
  delta_encoder_r = msg->count - encoder_r;
  encoder_r = msg->count;
}

void motorlCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  if(encoder_l == 0)
    encoder_l = msg->count;
  delta_encoder_l = msg->count - encoder_l;
  encoder_l = msg->count;
}

void PoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    odomtheta = msg->theta;
    if(odomx == 0.0 && odomy == 0.0) {
        odomx = msg->x;
        odomy = msg->y;
    }
}


void update_base_link()
{

    //Corrected equations acording to the provided paper
    double thetaL =  ((double) delta_encoder_l) * 2.0 * M_PI / ticks;
    double thetaR = -((double) delta_encoder_r) * 2.0 * M_PI / ticks;

    x_b = r * cos(odomtheta) * (thetaR + thetaL) / 2.0;
    y_b = r * sin(odomtheta) * (thetaR + thetaL) / 2.0;
    yaw_b = (r / b) * (thetaR - thetaL);
    //ROS_INFO("odometry angle theta: %f", odomtheta2);
    odomx += x_b;
    odomy += y_b;
    odomtheta += yaw_b;
    odomtheta2 += yaw_b;


    //ROS_INFO("x: %f, y: %f", odomx, odomy);

}

int main(int argc, char** argv){
      ROS_INFO("Odometry Started");
  ros::init(argc, argv, "odometry");
  //ros::NodeHandle n;
  ros::NodeHandle nh("~");

  nh.getParam("/robot/motor/left/name", motorl_name);
  nh.getParam("/robot/motor/right/name", motorr_name);
  nh.getParam("/robot/base_link_x_bias", x_b);
  nh.getParam("/robot/base_link_y_bias", y_b);
  nh.getParam("/robot/base_link_yaw_bias", yaw_b);
  nh.getParam("/frequency_mc", frequency);
  nh.getParam("/robot/wheels_radius", r);
  nh.getParam("/robot/base", b);
  nh.getParam("/robot/ticks_per_rev", ticks);

  //dt = (double) 1.0/frequency;

  ros::Rate loop_rate(frequency);

  ros::Publisher position_pub = nh.advertise<geometry_msgs::Pose2D>("/robot/position_change", 1);

  ros::Subscriber sub_motor1 = nh.subscribe(motorr_name + "/encoder", 1, motorrCallback);
  ros::Subscriber sub_motor2 = nh.subscribe(motorl_name + "/encoder", 1, motorlCallback);
  ros::Subscriber sub_robotPose = nh.subscribe("/robot/position", 1, PoseCallback);

  geometry_msgs::Pose2D pose;

  //ROS_INFO("HI");

  while(ros::ok()){


    ros::spinOnce();

    update_base_link();

    pose.x = x_b;
    pose.y = y_b;
    pose.theta = yaw_b;

    position_pub.publish(pose);
    
    loop_rate.sleep();
  }
}









