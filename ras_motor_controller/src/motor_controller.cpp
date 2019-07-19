#include "ros/ros.h"
#include "phidgets/motor_params.h"
#include "phidgets/motor_encoder.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <math.h>

double controlFrequency ;
double ticksPerRev ;



class MotorController
{public:

  ros::NodeHandle nh;
  ros::Subscriber encodersRightSubscriber;
  ros::Subscriber encodersLeftSubscriber;
  ros::Subscriber twistSubscriber;
  ros::Publisher pwmRightPublisher;
  ros::Publisher pwmLeftPublisher;

  MotorController()
  {
    nh = ros::NodeHandle("~"); //~ private node Handle
    deltaEncoderDouble = std::vector<double>(2, 0);// [0] left wheel, [1] right wheel
    targetW            = std::vector<double>(2, 0);
    estimatedW         = std::vector<double>(2, 0);
    error              = std::vector<double>(2, 0);
    errorAux           = std::vector<double>(2, 0);
    intError           = std::vector<double>(2, 0);
    derivError         = std::vector<double>(2, 0);
    alpha              = std::vector<double>(2, 0);// different paramenters for each wheel
    beta               = std::vector<double>(2, 0);
    gamma              = std::vector<double>(2, 0);

    wheelRadius = 0.05;
    base = 0.190;
    alpha[0] = 10.0;
    beta[0] = 200.0;
    alpha[1] = 10.0;
    beta[1] = 200.0;
    //gamma = 60

    deltaEncoderDouble[0] = 0.0;
    deltaEncoderDouble[1] = 0.0;



    nh.getParam("/robot/motor/left/alpha", alpha[0]);
    nh.getParam("/robot/motor/right/alpha", alpha[1]);
    nh.getParam("/robot/motor/left/beta", beta[0]);
    nh.getParam("/robot/motor/right/beta", beta[1]);
    nh.getParam("/robot/motor/left/gamma", gamma[0]);
    nh.getParam("/robot/motor/right/gamma", gamma[1]);
    nh.getParam("/robot/motor/left/name", motorl_name);
    nh.getParam("/robot/motor/right/name", motorr_name);
    nh.getParam("/frequency_motorcontroller", controlFrequency);
    nh.getParam("/robot/wheels_radius", wheelRadius);
    nh.getParam("/robot/base", base);
    nh.getParam("/robot/ticks_per_rev", ticksPerRev);

    linearVelocity = 0; //robot is stopped unless it recives instructions
    angularVelocity = 0;
    velocityTransformation(); //this initialices targetW to the desired default velocities
    estimatedW[0] = 0;//this initializes encoders at 0
    estimatedW[1] = 0;

    pwmRightPublisher = nh.advertise<std_msgs::Float32>(motorr_name + "/cmd_vel", 1);
    pwmLeftPublisher = nh.advertise<std_msgs::Float32>(motorl_name + "/cmd_vel", 1);
    encodersRightSubscriber = nh.subscribe(motorr_name + "/encoder", 1, &MotorController::encoderRightCallback, this);
    twistSubscriber = nh.subscribe("/set_velocity",1, &MotorController::twistCallback, this);
    encodersLeftSubscriber = nh.subscribe(motorl_name + "/encoder", 1, &MotorController::encoderLeftCallback, this);


  }
  // [0] corresponds to left wheel, [1] corresponds to right wheel
  void encoderLeftCallback(const phidgets::motor_encoder::ConstPtr& msg)
  {
    estimatedW[0] = 0.95 * estimatedW[0] + 0.05*( ((double) msg->count - deltaEncoderDouble[0]) *2*M_PI*controlFrequency)/(ticksPerRev);
    deltaEncoderDouble[0] = (double) msg->count;
  }
  void encoderRightCallback(const phidgets::motor_encoder::ConstPtr& msg)
  {
    estimatedW[1] = 0.95 * estimatedW[1] + 0.05*( ((double) msg->count - deltaEncoderDouble[1]) *2*M_PI*controlFrequency)/(ticksPerRev);
    deltaEncoderDouble[1] = (double) msg->count;
  }

  // [0] corresponds to left wheel, [1] corresponds to right wheel
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    angularVelocity = msg->angular.z;
    linearVelocity = 0.85 * linearVelocity + 0.15 * msg->linear.x;
    velocityTransformation(); //calculates tragetW
  }
  // [0] corresponds to left wheel, [1] corresponds to right wheel
  void velocityTransformation() //cinetic transformation
  {
    targetW[0] = (linearVelocity-(base/2)*angularVelocity)/(wheelRadius);
    targetW[1] = (linearVelocity+(base/2)*angularVelocity)/(wheelRadius);
  }

  double clamp(double n, double c)//saturates betwenn -100 and 100
  {
    return (n>c?c:(n<-c?-c:n));
  }


  void controller()
  {

    error[0] = targetW[0] - estimatedW[0]; //PI controller
    intError[0] = (error[0]==0.0?0.6:0.999)*clamp(intError[0] + error[0]/controlFrequency,10);
    derivError[0] = 0.9* derivError[0] + 0.1* (error[0]- errorAux[0]);
    errorAux[0] = error[0];
    outMsg.data = clamp(alpha[0]*error[0] + beta[0]*intError[0]+ gamma[0]*derivError[0],99);
    //ROS_INFO("PWM: %f targetw: %f error: %f ",outMsg.data, targetW[0], error[0]);
    pwmLeftPublisher.publish(outMsg);


    error[1] = targetW[1] + estimatedW[1];
    intError[1] = (error[1]==0.0?0.6:0.999)*clamp(intError[1] + error[1]/controlFrequency,10);//if error is 0 integral term dies out
    derivError[1] = 0.9* derivError[1] + 0.1* (error[1]- errorAux[1]);//exponetial smoothing
    errorAux[1] = error[1];
    outMsg.data = clamp(alpha[1]*error[1] + beta[1]*intError[1] + gamma[1]*derivError[1],99);
    //ROS_INFO("PWM: %f",outMsg.data);
    pwmRightPublisher.publish(outMsg);


  }

private:

  std::vector<double> deltaEncoderDouble;
  std::vector<double> targetW;
  std::vector<double> estimatedW;
  double linearVelocity;
  double angularVelocity;
  double wheelRadius;
  double base;
  std_msgs::Float32 outMsg;
  std::vector<double> error;
  std::vector<double> intError;
  std::vector<double> derivError;
  std::vector<double> errorAux;
  std::vector<double> alpha;
  std::vector<double> beta;
  std::vector<double> gamma;
  std::string motorr_name ;
  std::string motorl_name ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  MotorController motorController;
  ros::Rate loop_rate(controlFrequency);

  while (ros::ok() && motorController.nh.ok())
  {
    ros::spinOnce();
    motorController.controller();
    loop_rate.sleep();
  }
  return 0;
}
