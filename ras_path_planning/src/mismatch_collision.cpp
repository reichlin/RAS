#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include <cfloat>
#include <cmath>
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include <string>
#include "phidgets/motor_encoder.h"
#include "std_msgs/Float32.h"

bool obstacle = false;
bool standing_still = false;
bool started = false;
bool start_checking = false;
float pwm_r, pwm_l, encoder_r, encoder_l;
int count = 0;
int consecutive_times;
float posX, posY, posTheta, velX = 0;
float pos_error, frequency,  error_theta;
float old_posX = 0.0;
float old_posY = 0.0;
float old_posTheta = 0.0;
std::string motorr_name = "/right_wheel/right_motor";
std::string motorl_name = "/left_wheel/left_motor";

void startCallback(const std_msgs::BoolConstPtr& msg)
{
  start_checking = msg->data;
}

void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  if(velX>0.001)
  {
    for(int i = 236; i < 304; i++)
    {
      if((std::isinf(msg->ranges[i])?msg->range_max:msg->ranges[i]) < 0.14)
      {

        obstacle = true;
      }
    }
  }
  else if(velX<(-0.001))
  {
    for(int i = 56; i < 124; ++i)//70 degrees aperture, 90 degree tf rotation of lidar
    {
      if((std::isinf(msg->ranges[i])?msg->range_max:msg->ranges[i]) < 0.14){//the ternary condition sanitizes the input from inf's

        obstacle = true;
      }
    }
  }
}

void velCallback(const geometry_msgs::TwistConstPtr& msg)
{
  velX = msg->linear.x;
}

void posCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
  {  
    posX = msg->x;
    posY = msg->y;
    posTheta = msg->theta;
    

    if(fabs(old_posTheta) < 0.01 && fabs(old_posX) < 0.01 && fabs(old_posY) < 0.01)
    {
      old_posY = posY;
      old_posX = posX;
      old_posTheta = posTheta;

    }
    else if(!started && (fabs(posX - old_posX) < pos_error && fabs(posY - old_posY) < pos_error && fabs(posTheta - old_posTheta) < error_theta)){
      started= true;

    }
    else
    { 
      if(fabs(posX - old_posX) < pos_error && fabs(posY - old_posY) < pos_error && fabs(posTheta - old_posTheta) < error_theta)
       { 
        /*ROS_INFO("posX: %f, posY: %f, posTheta: %f", posX, posY, posTheta);
        ROS_INFO("OLDposX: %f, OLDposY: %f, OLDposTheta: %f", old_posX, old_posY, old_posTheta);
       ROS_INFO("FABS: X: %f, Y : %f, T: %f", fabs(posX - old_posX) , fabs(posY - old_posY) , fabs(posTheta -        old_posTheta));*/
        standing_still = true;
        //ROS_INFO("standing still condition fulfilled");
        }
      else
      {
         standing_still = false;
        //ROS_INFO("standing still condition NOT fulfilled");
        }

      old_posY = posY;
      old_posX = posX;
      old_posTheta = posTheta;
      /*
      if(count == consecutive_times)
      {
        standing_still = true;
        count = 0;
      }
      else if(!(posTheta > old_posTheta-error_theta && posTheta < old_posTheta+error_theta) ||
      (posX > old_posX-pos_error && posX < old_posX+pos_error) || 
      (posY > old_posY-pos_error && posY < old_posY+pos_error)) { 
        count = 0;
      }  
      else
      {
        count+=1;
      }
      */
    }
  }

void motorrCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  encoder_r = msg->count;
}

void motorlCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
  encoder_l = msg->count;
}



void pwmrCallback(const std_msgs::Float32::ConstPtr& msg)
{
  pwm_r = msg->data;
}

void pwmlCallback(const std_msgs::Float32::ConstPtr& msg)
{
  pwm_l = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mismatch");
  ros::NodeHandle n_;

  n_.getParam("/robot/motor/left/name", motorl_name);
  n_.getParam("/robot/motor/right/name", motorr_name);
  n_.getParam("/frequency_mc", frequency);
  n_.getParam("/position_error", pos_error);
  n_.getParam("/number_of_consecutive_times", consecutive_times);
  n_.getParam("/position_error_theta", error_theta);

  ros::Rate loop_rate(frequency);

  ros::Subscriber position_sub = n_.subscribe("/robot/position", 1, posCallback);
  ros::Subscriber velocity_sub = n_.subscribe("/set_velocity", 1, velCallback);
  ros::Subscriber lidarSubscriber = n_.subscribe("/scan", 1, lidarCallback);
  ros::Subscriber enc_motor1 = n_.subscribe(motorr_name + "/encoder", 1, motorrCallback);
  ros::Subscriber enc_motor2 = n_.subscribe(motorl_name + "/encoder", 1, motorlCallback);
  ros::Subscriber pwm_motor1 = n_.subscribe(motorr_name + "/cmd_vel", 1, pwmrCallback);
  ros::Subscriber pwm_motor2 = n_.subscribe(motorl_name + "/cmd_vel", 1, pwmlCallback);
  ros::Subscriber start_su= n_.subscribe("/start_col_avoidance", 1, startCallback);

  ros::Publisher imminentCollision = n_.advertise<std_msgs::Bool>("/imminentCollision", 1);
  ros::Publisher collision = n_.advertise<std_msgs::Bool>("/collision", 1);

  std_msgs::Bool msg;

  while (ros::ok() && n_.ok())
  { 
    ros::spinOnce();
 
    if(start_checking)
    {
      //ROS_INFO("count: %d",count);
      if(standing_still && ((fabs(pwm_r) > 10.0) || (fabs(pwm_l) > 10.0)))
      {
        count+=3;

      } 
      
      else if(count > 0)
        count--;

      if(count >= consecutive_times*3) {
        msg.data = true;
        imminentCollision.publish(msg);
        ROS_INFO("standing still and pwm mismatch ");
        count = 0;
      }

      /*if (((pwm_l != 0) && (encoder_l == 0)) || ((pwm_r != 0) &&  (encoder_r == 0)))
      {
        msg.data = true;;
        collision.publish(msg);
      }
  */
      if (obstacle)
      {
        msg.data = true;
        imminentCollision.publish(msg);
        ROS_INFO("Lidar detecting obstcale");
        obstacle = false;
      }
    }

    loop_rate.sleep();
  }
}
