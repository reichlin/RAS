#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>

double controlFrequency = 10;
double ticksPerRev = 360;

double constrainAngle(double x){
  x = fmod(x + M_PI,2*M_PI);
  if (x < 0)
    x += 2*M_PI;
  return x - M_PI;
}

class Driver
{public:
  double maxAcceleration;
  double targetLinearVelocity;
  double targetAngularVelocity;
  double headingDistance; //distance were the heading vector will point to
  bool newPath;
  bool stop;
  geometry_msgs::PoseArray pathArray;
  ros::NodeHandle nh;
  ros::Subscriber pathSubscriber;
  ros::Subscriber posSubscriber;
  ros::Subscriber lidarSubscriber;
  ros::Publisher velPublisher;

  Driver()
  {
    nh = ros::NodeHandle("~"); //~ private node Handle

    //deltaEncoderDouble = std::vector<double>(2, 0);// [0] left wheel, [1] right wheel

    alpha = 0.8; // variable for angle exponetial smoothing 0.91~1sec charact time
    beta = 1; //recomended value 0,3 ;variable for p controller, we are contrling v directly so 1 meand 1 second response time

    newPath = false;
    stop = false;

    headingDistance = 0.23; //distance were the heading vector will point to

    maxAcceleration = 2.0/controlFrequency;//!!!TODO revet to 2.0 this prevents the robot from slipping or falling over
    targetLinearVelocity = 0; //velocity we would want
    linearVelocity = 0; //veocity we are asking for (abrupt velocity changes are eliminated)
    targetAngularVelocity = 0;
    angularVelocity = 0;

    nh.getParam("/frequency", controlFrequency);
    nh.getParam("/robot/ticks_per_rev", ticksPerRev); //todo load params file in the launch file

    velPublisher = nh.advertise<geometry_msgs::Twist>("/set_velocity",1);
    lidarSubscriber = nh.subscribe("/scan", 1, &Driver::lidarCallback, this);
    pathSubscriber = nh.subscribe("/path", 1, &Driver::pathCallback, this);
    posSubscriber = nh.subscribe("/robot/position", 1, &Driver::posCallback, this);//TODO decide what topics to use


  }

  void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg)
  {

    //ROS_INFO("New LiDAR masage yum :]");
    for(int i = 235; i < 304; ++i)
    {
      if((isinf(msg->ranges[i])?msg->range_max:msg->ranges[i]) < 0.2)
      {
        //STAP!!
        ROS_INFO("I Feel Something Near me :?, at: %f meters  and angle: %d ",msg->ranges[i],i);
        stop = true;
      }
    }
  }

  void pathCallback(const geometry_msgs::PoseArrayConstPtr &msg)
  {
    newPath = true;
    pathArray.poses = msg->poses;

  }

  void posCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
    posX = msg->x;
    posY = msg->y;
    posTheta = constrainAngle(msg->theta);
  }

  void pathSegmentEquation(geometry_msgs::Pose start, geometry_msgs::Pose end)//function to find equation of current path segmet recives two points outputs(midifies private var) a,b,c /ax+by+c=0
  {
    a = end.position.x - start.position.x;
    b = start.position.y - end.position.y;
    c = -end.position.x* start.position.y + start.position.x* end.position.y;
  }

  std::pair<double,double> findClosestPoint()//finds nearest point on the current path segment recives none, uses posx posy and a b c
  {
    return std::make_pair((a*(a*posX-b*posY)-b*c)/(a*a+b*b),(b*(b*posY-a*posX)-a*c)/(a*a+b*b));
  }

  void calculateHeading(std::pair<double,double> nearestPoint,double nextNodeAngle)//using headingDistance closest point and posx posy and an angle we will have to calculate calculates desired heading
  {
    targetAngularVelocity =-beta*constrainAngle(((atan2(sqrt(pow(nearestPoint.first-posX,2)+pow(nearestPoint.second-posY,2)),headingDistance))+nextNodeAngle)-posTheta);//p controller beta is the gain
  }

  void publishVelocity()//publishes and smooths velocity, recives the two angles and does magic
  //TODO figure out how to stop at the end, probably the same module that decides when to switch to the nest node
  {
    angularVelocity = alpha* angularVelocity + (1-alpha)*targetAngularVelocity;
    msg.angular.z =angularVelocity;

    if(abs(targetLinearVelocity-linearVelocity)>maxAcceleration)    {
      linearVelocity += (targetLinearVelocity>linearVelocity?maxAcceleration:-maxAcceleration);
    }
    else    {
      linearVelocity = targetLinearVelocity;
    }
    msg.linear.x = linearVelocity;
    velPublisher.publish(msg);
  }

  bool closeToNode(double nextPosX, double nextPosY, double distance)//
  {
    return (pow(nextPosX-posX,2)+pow(nextPosY-posY,2)<pow(distance,2)?true:false);
  }

  void printStatus()
  {
    ROS_INFO("next point heading A%f B%f C%f ",a,b,c);
    ROS_INFO("### I'm at X%f Y%f theta%f",posX,posY,posTheta);
    ROS_INFO("speed %f targetspeed%f",linearVelocity,targetLinearVelocity);
    ROS_INFO("Ang speed %f Ang targetspeed%f",angularVelocity,targetAngularVelocity);
  }


private:

  double alpha; // variable for angle exponetial smoothing
  double beta; //variable for p controller

  double linearVelocity;
  double angularVelocity;

  double a; //path parametrization: ax +by +c =0
  double b;
  double c;
  double pathTheta;//angle form current path origin to current path end

  double posX;
  double posY;
  double posTheta;

  geometry_msgs::Twist msg;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver");
  Driver driver;
  ros::Rate loop_rate(controlFrequency);

  while (ros::ok() && driver.nh.ok())
  {
    while(!driver.newPath)
    {
      driver.stop = false;
      driver.targetLinearVelocity = 0.0;
      driver.targetAngularVelocity = 0.0;
      driver.publishVelocity();
      ros::spinOnce();
      loop_rate.sleep();
      ROS_INFO("I'm waiting for instructions from my master @.@ %d",(int)driver.newPath);
    }
    for(int i = 0; !(driver.stop) && (i < ((int) driver.pathArray.poses.size() - 2 )); ++i)
    {
      ROS_INFO("I'm Going On An Adventure :D , the amount of nodes is: %f",(float)driver.pathArray.poses.size());
      driver.newPath = false;
      driver.targetLinearVelocity = 30*sqrt(driver.maxAcceleration*driver.headingDistance);//TODO get param max_vel and use that and decide on a max vel, the equation enshure it can stop whitin halve circle

      driver.pathSegmentEquation(driver.pathArray.poses[i],driver.pathArray.poses[i+1]);
      while(!driver.stop && !(driver.closeToNode((float)driver.pathArray.poses[i+1].position.x,(float)driver.pathArray.poses[i+1].position.y,driver.headingDistance)))
      {
        driver.calculateHeading(driver.findClosestPoint(),driver.pathArray.poses[i+1].orientation.z);
        driver.publishVelocity();
        ROS_INFO("nearest X%f Y%f :O",driver.findClosestPoint().first,driver.findClosestPoint().second);
        ROS_INFO("next point heading %f",driver.pathArray.poses[i+1].orientation.z);

        ROS_INFO("###The next poit is at X%f, Y%f",driver.pathArray.poses[i+1].position.x,driver.pathArray.poses[i+1].position.y);
        ROS_INFO("###The start poit is at X%f, Y%f",driver.pathArray.poses[i].position.x,driver.pathArray.poses[i].position.y);
        driver.printStatus();
        ros::spinOnce();
        loop_rate.sleep();
      }
      ROS_INFO("I'm at node %d :O",i);
    }
    while(!driver.stop && !(driver.closeToNode((float)driver.pathArray.poses[(float)driver.pathArray.poses.size()-1.0].position.x,(float)driver.pathArray.poses[(float)driver.pathArray.poses.size()-1].position.y,0.15)))
    {
      driver.calculateHeading(driver.findClosestPoint(),(float)driver.pathArray.poses[(float)driver.pathArray.poses.size()-1.0].orientation.z);
      driver.targetLinearVelocity = 0.1*sqrt(2*driver.maxAcceleration*driver.headingDistance);//0.1*max speed for braking in circle
      driver.publishVelocity();
      //ROS_INFO("nearest X%f Y%f :O",driver.findClosestPoint().first,driver.findClosestPoint().second);
      ros::spinOnce();
      loop_rate.sleep();
      ROS_INFO("I'm at my destination wohooo :P");
    }
  }
  return 0;
}
