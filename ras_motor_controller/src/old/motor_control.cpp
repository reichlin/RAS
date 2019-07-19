#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "phidgets/motor_params.h"
#include "phidgets/motor_encoder.h"
#include <pid/pid.h>

float control_effort_r;
float control_effort_l;
int frequency;
std::string motorr_name = "/right_wheel/right_motor";
std::string motorl_name = "/left_wheel/left_motor";

//Callback when something is published on 'control_effort'
void controlEffortCallback_r(const std_msgs::Float64::ConstPtr& control_effort_input)
{
  control_effort_r = control_effort_input->data;
}

void controlEffortCallback_l(const std_msgs::Float64::ConstPtr& control_effort_input)
{
  control_effort_l = control_effort_input->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh("~");

  nh.getParam("/robot/motor/left/name", motorl_name);
  nh.getParam("/robot/motor/right/name", motorr_name);
  nh.getParam("/frequency_mc", frequency);

  ros::Subscriber sub_r = nh.subscribe(motorr_name + "/control_effort", 100, controlEffortCallback_r);
  ros::Subscriber sub_l = nh.subscribe(motorl_name + "/control_effort", 100, controlEffortCallback_l);
  ros::Publisher pub_control_r = nh.advertise<std_msgs::Float32>(motorr_name + "/cmd_vel", 10);
  ros::Publisher pub_control_l = nh.advertise<std_msgs::Float32>(motorl_name + "/cmd_vel", 10);

  ros::Rate loop_rate(frequency);

  while (ros::ok() && nh.ok())
  {
    ros::spinOnce();
    std_msgs::Float32 msg_r;
    std_msgs::Float32 msg_l;
    msg_r.data = control_effort_r;
    msg_l.data = control_effort_l;

    /* PID library already does this, params changed
    if(control_effort_r < -100.0)
    {
      control_effort_r = -100.0;
    }

    if(control_effort_r > 100.0)
    {
      control_effort_r = 100.0;
    }

    if(control_effort_l < -100.0)
    {
      control_effort_l = -100.0;
    }

    if(control_effort_l > 100.0)
    {
      control_effort_l = 100.0;
    }*/
    
    pub_control_r.publish(msg_r);
    pub_control_l.publish(msg_l);

    loop_rate.sleep();
  }
}

