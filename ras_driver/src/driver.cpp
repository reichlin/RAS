#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Bool.h"
#include <math.h>

double controlFrequency = 20 , ticksPerRev;
double linearVelocity = 0.20, angularVelocity = 1.5, maxAngularVelocity = 3.0, curvature;//for 0.02 vel use 0,22 löook ahead
bool stop = true;
double orientation = 1.0, lastAngularSpeed = 0.0;
double posX, posY, posTheta;
double lookAheadDistance = 0.20, distanceTreshold = 0.11, angleTreshold = 0.20, slowdown = 0.5;//0.035rad ~ 20°
double closestX,closestY,goalX,goalY,goalX_R,goalY_R;
geometry_msgs::PoseArray pathArray;
geometry_msgs::Twist velMsg;

double constrainAngle(double x){
  x = fmod(x + M_PI,2*M_PI);
  if (x < 0)
    x += 2*M_PI;
  return x - M_PI;
}

double constrainAngularSpeed(double aSpeed){
  aSpeed = (aSpeed > maxAngularVelocity? maxAngularVelocity: aSpeed);
  return (aSpeed < (-maxAngularVelocity)? -maxAngularVelocity: aSpeed);
}

void pathCallback(const geometry_msgs::PoseArrayConstPtr &msg)
{

  if(msg->poses.size()==0)
  {
    stop = true;
  }
  else{
    stop = false;
    if(fabs(msg->poses[msg->poses.size()-1].orientation.z-180.0) < 0.01)//flotating point comparisson, if 180 in the last angle => bakcwards driving
      orientation = -1.0;
    else
      orientation = 1.0;
    pathArray.poses = msg->poses;
  }
  //Speed selection
  if(fabs(pathArray.poses[0].orientation.w - 1.0) < 0.01)//explore, no slowdown
  {
    linearVelocity = 0.19;
    lookAheadDistance = 0.19;
    slowdown = 1.0;
    distanceTreshold = 0.06;
    ROS_INFO("explore speed");
  }
  else if(fabs(pathArray.poses[0].orientation.w - 2.0) < 0.01){//rescue objects//fast

    linearVelocity = 0.45;//0.4
    lookAheadDistance = 0.27;//0.26
    slowdown = 0.5;
    distanceTreshold = 0.06;
    ROS_INFO("rescue speed (fast)");
  }
  else if(fabs(pathArray.poses[0].orientation.w - 3.0) < 0.01){//grab //slow
    linearVelocity = 0.15;
    lookAheadDistance = 0.12;
    slowdown = 0.8;
    distanceTreshold = 0.105;
    ROS_INFO("catch speed");
  }
  else{
    linearVelocity = 0.21;
    lookAheadDistance = 0.20;
    slowdown = 0.5;
    distanceTreshold = 0.08;
    ROS_INFO("default speed");
  }

}

void posCallback(const geometry_msgs::Pose2DConstPtr &msg){
  posX = msg->x;
  posY = msg->y;
  posTheta = constrainAngle((orientation > 0.0 ? 0.0: M_PI)+ msg->theta);
}

void findClosestPoint(geometry_msgs::Pose start, geometry_msgs::Pose end){//function to find finds nearest point on the current path segment recives two points
  double a = end.position.x - start.position.x;
  double b = start.position.y - end.position.y;
  double c = -end.position.x* start.position.y + start.position.x* end.position.y;
  closestX = (a*(a*posX-b*posY)-b*c)/(a*a+b*b);
  closestY = (b*(b*posY-a*posX)-a*c)/(a*a+b*b);
}

bool closeToNode(double nextPosX, double nextPosY, double distance){//answers: is the asked point close to the robot?
  return (pow(nextPosX-posX,2.0)+pow(nextPosY-posY,2.0)<pow(distance,2.0));
}

bool closeToAngle( double targetAngle, double angleRange){//answers: is the asked angle close to the robots angle?
  return ( fabs(constrainAngle(posTheta-constrainAngle(targetAngle))) < angleRange);
}

void findGoalPoint(geometry_msgs::Pose end){//function that finds goal pos in global coords
  goalX = closestX;
  goalY = closestY;
  for(int j = 0; j < 251 && closeToNode(goalX,goalY,lookAheadDistance); ++j){
    goalX = 0.004* ((closestX * (float)(250-j) + end.position.x * (float) j));
    goalY = 0.004* ((closestY * (float)(250-j) + end.position.y * (float) j));
  }
}

void calculateGoalRobotCoords(){
  goalX_R = cos(posTheta) * (goalX-posX) + sin(posTheta) * (goalY-posY);
  goalY_R =-sin(posTheta) * (goalX-posX) + cos(posTheta) * (goalY-posY);
}

void calculateRotationCurvature(){
  curvature = (2.0 * goalY_R)/(lookAheadDistance*lookAheadDistance);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "driver");
  ros::NodeHandle nh("~"); //~ private node Handle
  nh.getParam("/frequency_mc", controlFrequency);
  nh.getParam("/robot/ticks_per_rev", ticksPerRev);
  ros::Rate loop_rate(controlFrequency);

  ros::Publisher velPublisher = nh.advertise<geometry_msgs::Twist>("/set_velocity",1);
  ros::Publisher finishedPathPublisher = nh.advertise<std_msgs::Bool>("/robot/finishedPath",1);
  ros::Publisher readyToGoPublisher = nh.advertise<std_msgs::Bool>("/robot/readyToGo",5);
  ros::Subscriber pathSubscriber = nh.subscribe("/path", 5, &pathCallback);
  ros::Subscriber posSubscriber = nh.subscribe("/robot/position", 1, &posCallback);

  while (ros::ok() && nh.ok()){
    ROS_INFO("Waiting for driving instructions");



    while(ros::ok() && nh.ok() && stop){

      std_msgs::Bool msg; msg.data = true;
      readyToGoPublisher.publish(msg);

      velMsg.angular.z = 0.0;
      velMsg.linear.x = 0.0;
      velPublisher.publish(velMsg);

      loop_rate.sleep();
      ros::spinOnce();
    } ROS_INFO("Recived driving instructions");
    //angular message
    if(closeToNode(pathArray.poses[0].position.x,pathArray.poses[0].position.y,distanceTreshold) && (pathArray.poses.size()==1)){
      while(ros::ok() && nh.ok() && !stop && !closeToAngle(pathArray.poses[0].orientation.z,angleTreshold)){

        velMsg.angular.z = (constrainAngle( constrainAngle(pathArray.poses[0].orientation.z)-constrainAngle(posTheta ))> 0.0? angularVelocity: -angularVelocity);
        velMsg.linear.x = 0;
        velPublisher.publish(velMsg);
        
        loop_rate.sleep();
        ros::spinOnce();
      }
    }
    else{//linear message
      //we point to the first node
        pathArray.poses[0].orientation.z = constrainAngle(atan2(pathArray.poses[0].position.y-posY,pathArray.poses[0].position.x-posX));
      if(!closeToNode(pathArray.poses[0].position.x,pathArray.poses[0].position.y,lookAheadDistance))//at first node
      {

        lastAngularSpeed =(constrainAngle(constrainAngle(pathArray.poses[0].orientation.z)-constrainAngle(posTheta))>0.0? angularVelocity: -angularVelocity);

        while(ros::ok() && nh.ok() && !stop && !closeToAngle(pathArray.poses[0].orientation.z,angleTreshold)){


          velMsg.angular.z = (constrainAngle(constrainAngle(pathArray.poses[0].orientation.z)-constrainAngle(posTheta))>0.0? angularVelocity: -angularVelocity);
          velMsg.linear.x = 0.0;
          if(fabs(lastAngularSpeed - velMsg.angular.z)>0.001  && fabs(constrainAngle(pathArray.poses[0].orientation.z)-constrainAngle(posTheta))<0.3)//if the sign changes mans that we are in the right track
          {
            break;
            ROS_INFO("break");
          }
          lastAngularSpeed = velMsg.angular.z;
          //ROS_INFO("desired angle: %f robot angle: %f angular velocity: %f angle treshold: %f ",pathArray.poses[0].orientation.z, posTheta, velMsg.angular.z, angleTreshold);

          velPublisher.publish(velMsg);

          loop_rate.sleep();
          ros::spinOnce();
        }
      }
      else//if at first node we point to the next node
      {
        pathArray.poses[1].orientation.z = constrainAngle(atan2(pathArray.poses[1].position.y-posY,pathArray.poses[1].position.x-posX));
        lastAngularSpeed =(constrainAngle(constrainAngle(pathArray.poses[1].orientation.z)-constrainAngle(posTheta))>0.0? angularVelocity: -angularVelocity);

        while(ros::ok() && nh.ok() && !stop && !closeToAngle(pathArray.poses[1].orientation.z,angleTreshold)){

          velMsg.angular.z = (constrainAngle(constrainAngle(pathArray.poses[1].orientation.z)-constrainAngle(posTheta))>0.0? angularVelocity: -angularVelocity);
          velMsg.linear.x = 0.0;
          if(fabs(lastAngularSpeed - velMsg.angular.z)>0.001 && fabs(constrainAngle(pathArray.poses[1].orientation.z)-constrainAngle(posTheta))<0.3)//if the sign changes mans that we are in the right track
          {
            break;
            ROS_INFO("break");
          }
          lastAngularSpeed = velMsg.angular.z;
          velPublisher.publish(velMsg);
          //ROS_INFO("desired angle: %f robot angle: %f angular velocity: %f angle treshold: %f ",pathArray.poses[1].orientation.z, posTheta, velMsg.angular.z, angleTreshold);

          loop_rate.sleep();
          ros::spinOnce();
        }
      }

      velMsg.angular.z = 0.0;
      velMsg.linear.x = 0.0;
      velPublisher.publish(velMsg);
      loop_rate.sleep();
      ros::spinOnce();


      //then the path starts
      for(int i= 0;ros::ok() && nh.ok() && !stop && i< pathArray.poses.size()-1; ++i){
        ROS_INFO("im at node:%d  X:%f Y:%f , pos: X:%f Y:%f",i,pathArray.poses[i].position.x,pathArray.poses[i].position.y,posX,posY);
        pathArray.poses[i+1].orientation.z = constrainAngle(atan2(pathArray.poses[i+1].position.y-posY,pathArray.poses[i+1].position.x-posX));
        if(fabs( constrainAngle(pathArray.poses[i+1].orientation.z-constrainAngle(posTheta)))>1.3)//0,87rad=50grad
        {

          lastAngularSpeed =(constrainAngle(constrainAngle(pathArray.poses[i+1].orientation.z)-constrainAngle(posTheta))>0.0? angularVelocity: -angularVelocity);

          while(ros::ok() && nh.ok() && !stop && !closeToAngle(pathArray.poses[i+1].orientation.z,angleTreshold)){

            velMsg.angular.z = (constrainAngle(constrainAngle(pathArray.poses[i+1].orientation.z)-constrainAngle(posTheta))>0.0? angularVelocity: -angularVelocity);
            velMsg.linear.x = 0.0;
            if(fabs(lastAngularSpeed - velMsg.angular.z)>0.001 && fabs(constrainAngle(pathArray.poses[i+1].orientation.z)-constrainAngle(posTheta))<0.3)//if the sign changes mans that we are in the right track
            {
              break;
            ROS_INFO("break");
            }
            lastAngularSpeed = velMsg.angular.z;
            velPublisher.publish(velMsg);
            //ROS_INFO("desired angle: %f robot angle: %f angular velocity: %f angle treshold: %f ",pathArray.poses[1].orientation.z, posTheta, velMsg.angular.z, angleTreshold);

            loop_rate.sleep();
            ros::spinOnce();
          }

        }

        while(ros::ok() && nh.ok() && !stop && !closeToNode(pathArray.poses[i+1].position.x,pathArray.poses[i+1].position.y,lookAheadDistance)){

          ROS_INFO("desired angle: %f robot angle: %f angular velocity: %f angle treshold: %f ",pathArray.poses[1].orientation.z, posTheta, velMsg.angular.z, angleTreshold);


          findClosestPoint(pathArray.poses[i],pathArray.poses[i+1]);
          findGoalPoint(pathArray.poses[i+1]);
          calculateGoalRobotCoords();
          calculateRotationCurvature();
          velMsg.linear.x = orientation * linearVelocity ;//* (300.0-fabs(curvature))/300.0
          velMsg.angular.z = constrainAngularSpeed( linearVelocity * curvature );//* (300.0-fabs(curvature))/300.0
          velPublisher.publish(velMsg);
          //ROS_INFO("im at  X:%f Y:%f, last nodeX: %f Y:%f, next node pos: X:%f Y:%f",posX,posY,pathArray.poses[i].position.x,pathArray.poses[i].position.y,pathArray.poses[i+1].position.x,pathArray.poses[i+1].position.y);
          loop_rate.sleep();
          ros::spinOnce();
        }
      }
      while(ros::ok() && nh.ok() && !stop && !closeToNode(pathArray.poses[pathArray.poses.size()-1].position.x,pathArray.poses[pathArray.poses.size()-1].position.y,distanceTreshold)){//this is the last node closer than look ahead distance

        goalX = pathArray.poses[pathArray.poses.size()-1].position.x;
        goalY = pathArray.poses[pathArray.poses.size()-1].position.y;
        calculateGoalRobotCoords();
        calculateRotationCurvature();

        velMsg.linear.x = orientation * slowdown *linearVelocity ;//* (300.0-fabs(curvature))/300.0
        velMsg.angular.z = constrainAngularSpeed( slowdown * linearVelocity * curvature );//* (300.0-fabs(curvature))/300.0
        velPublisher.publish(velMsg);
        

        loop_rate.sleep();
        ros::spinOnce();
      }
    }
    if(!stop)
    {
      std_msgs::Bool msg; msg.data = true;
      finishedPathPublisher.publish(msg);
      stop = true;
    }
  }
}
