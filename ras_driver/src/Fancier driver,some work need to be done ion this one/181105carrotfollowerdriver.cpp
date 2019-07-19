#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Bool.h"
#include <math.h>

double controlFrequency = 10;
double ticksPerRev = 360;

double constrainAngle(double x)
{
  x = fmod(x + M_PI,2*M_PI);
  if (x < 0)
    x += 2*M_PI;
  return x - M_PI;
}

class Driver
{
public:
  //double maxAcceleration;
  double maxVelocity;
  double maxAngularVelocity;
  double targetLinearVelocity;
  double targetAngularVelocity;
  double distanceTreshold;//TODO add distance threshold to param server
  float angularTreshold;
  double posTheta;
  double posX;
  double posY;
  bool newPath;
  bool stop;
  geometry_msgs::PoseArray pathArray;
  ros::NodeHandle nh;
  ros::Subscriber pathSubscriber;
  ros::Subscriber posSubscriber;
  ros::Publisher velPublisher;
  ros::Publisher finishedPathPublisher;

  Driver()
  {
    nh = ros::NodeHandle("~"); //~ private node Handle

    //alpha = 0.20; // variable for angle exponetial smoothing 0.91~1sec charact time
    beta = 0.2; //recomended value 0,3 ;variable for p controller, we are contrling v directly so 1 meand 1 second response time

    newPath = false;
    stop = false;


    nh.getParam("/frequency_mc", controlFrequency);
    nh.getParam("/robot/ticks_per_rev", ticksPerRev); //todo load params file in the launch file

    maxVelocity = 0.03;//0.05      0.012 is super slow but works
    maxAngularVelocity = 0.7;
    //maxAcceleration = 3.0/((float)controlFrequency);//this prevents the robot from slipping or falling over 3 is max val aproxx
    targetLinearVelocity = 0; //velocity we would want
    linearVelocity = 0; //veocity we are asking for (abrupt velocity changes are eliminated)
    angularTreshold = 0.22;
    targetAngularVelocity = 0;
    angularVelocity = 0;
    distanceTreshold = 0.35;

    velPublisher = nh.advertise<geometry_msgs::Twist>("/set_velocity",1);
    finishedPathPublisher = nh.advertise<std_msgs::Bool>("/robot/finishedPath",1);

    pathSubscriber = nh.subscribe("/path", 1, &Driver::pathCallback, this);
    posSubscriber = nh.subscribe("/robot/position", 1, &Driver::posCallback, this);

  }



  void pathCallback(const geometry_msgs::PoseArrayConstPtr &msg)
  {
    newPath = true;
    if(msg->poses.size()==0)
    {
      stop = true;
      newPath = false;
    }
    pathArray.poses = msg->poses;
  }

  void posCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
    posX = msg->x;
    posY = msg->y;
    posTheta = constrainAngle(msg->theta);
  }

  void calculateHeading(double fPosX,double fPosY)//using headingDistance closest point and posx posy and an angle we will have to calculate calculates desired heading
  {
    targetAngularVelocity = constrainAngle(atan2(fPosY-posY,fPosX-posX)-posTheta);
  }

  void calculateAngularHeading(double fPosX,double fPosY)//using headingDistance closest point and posx posy and an angle we will have to calculate calculates desired heading
  {
    targetAngularVelocity = (constrainAngle(atan2(fPosY-posY,fPosX-posX)-posTheta)>0.0?maxAngularVelocity:-maxAngularVelocity);
    targetAngularVelocity = (fabs(constrainAngle(atan2(fPosY-posY,fPosX-posX)-posTheta))<angularTreshold?0:targetAngularVelocity);
  }

  void calculateHeadingBackwards(double fPosX,double fPosY)//using headingDistance closest point and posx posy and an angle we will have to calculate calculates desired heading
  {
    targetAngularVelocity = constrainAngle(M_PI+atan2(fPosY-posY,fPosX-posX)-posTheta);
  }

  void publishVelocity()//publishes and smooths velocity, recives the two angles and does magic
  {


    angularVelocity = targetAngularVelocity;

    msg.angular.z =beta*angularVelocity;

    //smooth acceleration sube-routine, is working, accel params should be selected
    /* if(fabs(targetLinearVelocity-linearVelocity)>maxAcceleration)    {
      linearVelocity += (targetLinearVelocity>linearVelocity?maxAcceleration:-maxAcceleration);
    }
    else    {*/
    linearVelocity = targetLinearVelocity;
    //}
    msg.linear.x = linearVelocity;
    velPublisher.publish(msg);
    //ROS_INFO("speed %f A speed%f",linearVelocity,angularVelocity);
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

  bool nearestCloseToNode(double nextPosX, double nextPosY, double distance)//is the asked point close to the nearest ponit in the path to the robot?
  {
    return (pow(nextPosX-findClosestPoint().first,2)+pow(nextPosY-findClosestPoint().second,2)<pow(distance,2));
  }

  bool closeToNode(double nextPosX, double nextPosY, double distance)//is the asked point close to the robot?
  {
    return (pow(nextPosX-posX,2)+pow(nextPosY-posY,2)<pow(distance,2));
  }


private:

  //double alpha; // variable for angle exponetial smoothing
  double beta; //variable for p controller
  double linearVelocity;
  double angularVelocity;
  double a; //path parametrization: ax +by +c =0
  double b;
  double c;


  geometry_msgs::Twist msg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver");
  Driver driver;
  ros::Rate loop_rate(controlFrequency);
  ros::Duration pause(1);


  while (ros::ok() && driver.nh.ok())
  {
    ROS_INFO("I'm waiting for instructions from my master @.@ ");

    while(ros::ok() && driver.nh.ok() && !driver.newPath)
    {
      loop_rate.sleep();
      ros::spinOnce();
      driver.stop = false;
      driver.targetLinearVelocity = 0.0;
      driver.targetAngularVelocity = 0.0;
      driver.publishVelocity();

    }

    ROS_INFO("I'm Going On An Adventure :D , the amount of nodes is: %d",(int)driver.pathArray.poses.size());

    if((int)driver.pathArray.poses.size()==1 && driver.closeToNode((float)driver.pathArray.poses[0].position.x,(float)driver.pathArray.poses[0].position.y,driver.distanceTreshold))
      /* i'm at the node and there is only one in the whole path*/
    {
      while(driver.nh.ok() && fabs(driver.pathArray.poses[0].orientation.z-driver.posTheta)>driver.angularTreshold) //0.1 Rad ~ 6°
      {
        driver.targetAngularVelocity = (constrainAngle(constrainAngle(driver.pathArray.poses[0].orientation.z)-constrainAngle(driver.posTheta))>0.0?driver.maxAngularVelocity:-driver.maxAngularVelocity);
        driver.publishVelocity();
        ros::spinOnce();
        loop_rate.sleep();
      }
      std_msgs::Bool msg;
      msg.data = true;
      driver.finishedPathPublisher.publish(msg);
      ROS_INFO("I'm at my Angular destination, WooHoo  o(^_^)o ");//ヾ（〃＾∇＾）ﾉ♪
      driver.newPath = false;
    }
    else if((int)driver.pathArray.poses.size()==1 && !driver.closeToNode((float)driver.pathArray.poses[0].position.x,(float)driver.pathArray.poses[0].position.y,driver.distanceTreshold))
      /* i'm not at the node and there is only one in the whole path*/
    {
      driver.targetLinearVelocity = driver.maxVelocity*0.7;
      while(driver.nh.ok() && !driver.stop && !(driver.closeToNode((float)driver.pathArray.poses[0].position.x,(float)driver.pathArray.poses[0].position.y,driver.distanceTreshold)))
      {
        driver.calculateHeading(driver.pathArray.poses[0].position.x,driver.pathArray.poses[0].position.y);
        driver.publishVelocity();
        ros::spinOnce();
        loop_rate.sleep();
      }
      std_msgs::Bool msg;
      msg.data = true;
      driver.finishedPathPublisher.publish(msg);
      ROS_INFO("I'm at my  one node destination, WooHoo o(^_^)o ");
      driver.newPath = false;
    }
    else if( fabs(driver.pathArray.poses[driver.pathArray.poses.size()-1].orientation.z - 180.0) < 0.001)//flotating point asertion done right
      /*there are many nodes and have to drive bakwards*/
    {
      driver.targetLinearVelocity = -driver.maxVelocity*0.7;
      while(driver.nh.ok() && !driver.stop && !(driver.closeToNode((float)driver.pathArray.poses[0].position.x,(float)driver.pathArray.poses[0].position.y,driver.distanceTreshold)))//Are we already at node 0?
      {
        driver.calculateHeadingBackwards(driver.pathArray.poses[0].position.x,driver.pathArray.poses[0].position.y);
        driver.publishVelocity();
        ros::spinOnce();
        loop_rate.sleep();
      }
      ROS_INFO("I'm at node %d :O",1);
      for(int i = 0; !driver.stop && (i < ((int) driver.pathArray.poses.size() - 1 )); ++i)
      {

        driver.pathSegmentEquation(driver.pathArray.poses[i], driver.pathArray.poses[i+1]);// calculates equation of current path segment

        driver.targetLinearVelocity = -driver.maxVelocity *0.7;//
        ROS_INFO("my expected velocity is: %f", driver.targetLinearVelocity);

        while(driver.nh.ok() && !driver.stop && !(driver.nearestCloseToNode((float)driver.pathArray.poses[i+1].position.x,(float)driver.pathArray.poses[i+1].position.y,driver.distanceTreshold)))
        {
          driver.calculateHeadingBackwards(driver.pathArray.poses[i+1].position.x,driver.pathArray.poses[i+1].position.y);
          driver.publishVelocity();
          ros::spinOnce();
          loop_rate.sleep();
        }

        if(i == (int) driver.pathArray.poses.size() - 2) //last node
        {
          std_msgs::Bool msg;
          msg.data = true;
          driver.finishedPathPublisher.publish(msg);
          ROS_INFO("I'm at my destination, WooHoo \\ (^_^) / ");
          driver.newPath = false;
        }
        else
        {
          ROS_INFO("I'm at node %d :O",i+1);
        }
      }
    }
    else
      /*there are many nodes in the path*/
    {
      driver.targetLinearVelocity = driver.maxVelocity;
      while(driver.nh.ok() && !driver.stop && !(driver.closeToNode((float)driver.pathArray.poses[0].position.x,(float)driver.pathArray.poses[0].position.y,driver.distanceTreshold)))//Are we already at node 0?
      {
        loop_rate.sleep();
        ros::spinOnce();
        driver.calculateHeading(driver.pathArray.poses[0].position.x,driver.pathArray.poses[0].position.y);
        driver.publishVelocity();

      }
      ROS_INFO("I'm at node 0 ");
      for(int i = 0; !driver.stop && (i < ((int) driver.pathArray.poses.size() - 1 )); ++i)
      {

        driver.targetLinearVelocity = 0.0;
        ROS_INFO("my expected velocity is: %f", driver.targetLinearVelocity);
        ROS_INFO("im stopping and turning");

        driver.pathSegmentEquation(driver.pathArray.poses[i], driver.pathArray.poses[i+1]);// calculates equation of current path segment

        pause.sleep();

        while(driver.nh.ok() && !driver.stop && (fabs(driver.targetAngularVelocity) > driver.angularTreshold)) //0.1 Rad ~ 6°
        {
          loop_rate.sleep();
          ros::spinOnce();
          driver.calculateAngularHeading(driver.pathArray.poses[i+1].position.x,driver.pathArray.poses[i+1].position.y);
          driver.publishVelocity();

        }

        pause.sleep();

        driver.targetLinearVelocity = driver.maxVelocity *((i == (int) driver.pathArray.poses.size() - 2)? 0.7: 1.0);//(last node? go slower : normal speed )
        ROS_INFO("my expected velocity is: %f", driver.targetLinearVelocity);

        while(driver.nh.ok() && !driver.stop && !(driver.nearestCloseToNode((float)driver.pathArray.poses[i+1].position.x,(float)driver.pathArray.poses[i+1].position.y,driver.distanceTreshold)))
        {
          loop_rate.sleep();
          ros::spinOnce();
          driver.calculateHeading(driver.pathArray.poses[i+1].position.x,driver.pathArray.poses[i+1].position.y);
          driver.publishVelocity();

        }

        if(i == (int) driver.pathArray.poses.size() - 2) //last node
        {
          std_msgs::Bool msg;
          msg.data = true;
          driver.finishedPathPublisher.publish(msg);
          ROS_INFO("I'm at my destination, WooHoo \\ (^_^) / ");
          driver.newPath = false;
        }
        else
        {
          ROS_INFO("I'm at node %d :O posx: %f posy: %f  expectedx: %f expectedy: %f",i+1,driver.posX,driver.posY,driver.pathArray.poses[i+1].position.x,driver.pathArray.poses[i+1].position.y);
        }
      }
    }
  }
  return 0;
}
