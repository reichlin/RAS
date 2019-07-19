#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "ras_msgs/classified_object.h"
#include "visualization_msgs/MarkerArray.h"
#include "arduino_servo_control/SetServoAngles.h"
#include <math.h>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <istream>
#include <string>
#include <iostream>

using namespace std;

float posX = 0.0, posY = 0.0, posTheta = 0.0;
float startX = 0.0, startY = 0.0;
float x, y, phi;
float objectX, objectY;
int numberOfExploredNodes = 0;

struct Object{
  string name;
  double posX, posY;
  int frameX, frameY;
  int price;//how many points we get for this object
};

bool sortFun (Object i,Object j)
{
  return (i.price>j.price);
}

double constrainAngle(double x){
  x = fmod(x + M_PI,2*M_PI);
  if (x < 0)
    x += 2*M_PI;
  return x - M_PI;
}

enum states{IDLE,
            SEARCH,PLAN_SEARCH_PATH,EXECUTE_SEARCH_PATH,GO_TO_NEXT_SEARCH_NODE,PARK, RECOVERY,
            RESCUE,PLAN_RESCUE_PATH,EXECUTE_RESCUE_PATH,GO_TO_NEXT_RESCUE_NODE,GO_BACK_TO_START,
            CATCH_OBJECT,VERIFY_OBJECT_CATCHED,LEAVE_OBJECT,GET_READY_START};




class Master
{
public:
  ros::Publisher pathPublisher;
  ros::Publisher startPointPublisher;
  ros::Publisher endPointPublisher;
  ros::Publisher talkerPublisher;
  ros::Publisher objectMarkerPublisher;
  ros::Publisher objectPositionPublisher;
  ros::Publisher startCollisionAvoidancePublisher;
  ros::Publisher goalPublisher;
  ros::Publisher classifyPub;
  ros::Publisher pathNotFoundPublisher;
  ros::Publisher espeakPublisher;
  ros::Publisher savePublisher;
  ros::Subscriber collisionSub;
  ros::Subscriber position_sub;
  ros::Subscriber object_position_sub;
  ros::Subscriber finishedPathSub;
  ros::Subscriber pathPlanSub;
  ros::Subscriber readyToGoSub;
  ros::Subscriber newMapSub;
  ros::Subscriber nextSearchNodeSub;
  ros::Subscriber batterySub;
  ros::Subscriber improvedGoalSub;
  ros::Subscriber setVelocitySub;
  ros::Subscriber posChangeSub;
  ros::ServiceClient gripperService;
  ros::Subscriber classifySub;
  ros::NodeHandle nh;
  vector<Object> objects;
  states state, previousState;
  bool inminentCollision;
  bool imminentCollision;
  bool severeCollision;
  bool finishedPath;
  bool readyToGo;
  bool updatedPosition;
  bool updatedPathPlan;
  bool initializedPosition;
  bool replanNeeded;
  bool improvedGoalFlag;
  int orientation;
  geometry_msgs::Pose2D improvedGoal;
  geometry_msgs::Pose searchNode;
  geometry_msgs::PoseArray pathPlan;
  int currentSearchNode;
  int currentRescueObject;
  int catchingAtempts;
  int objectClass;
  int objectBeingClassified;
  bool objectClassFlag;
  geometry_msgs::PoseArray currentPath;
  double catchingDistance;
  vector<string> objectType;
  bool goingSlow;
  double setVelocity;
  ros::Time begin;
  ros::Time begin_point_updating;

  Master()
  {
    nh = ros::NodeHandle("~"); //~ private node Handle
    state = IDLE;
    readyToGo = false;
    //inminentCollision = false;
    finishedPath = false;
    updatedPosition = false;
    initializedPosition = false;
    replanNeeded = false;
    catchingDistance = 0.32;//0,26
    catchingAtempts = 0;
    improvedGoalFlag = false;
    goingSlow = true;
    objectClassFlag = true;
    orientation = 0;
    setVelocity = 0.0;

    objectType.push_back("blueCube");
    objectType.push_back("blueTriangle");
    objectType.push_back("greenCube");
    objectType.push_back("greenCylinder");
    objectType.push_back("greenHollowCube");
    objectType.push_back("orangeCross");
    objectType.push_back("patric");
    objectType.push_back("purpleCross");
    objectType.push_back("purpleStar");
    objectType.push_back("redBall");
    objectType.push_back("redCube");
    objectType.push_back("redCylinder");
    objectType.push_back("yellowBall");
    objectType.push_back("yellowCube");


    //subs
    //collisionSub = nh.subscribe("/inminentColision", 1, &Master::collisionCallback, this);
    collisionSub = nh.subscribe("/imminentCollision", 5, &Master::collisionCallback, this);
    position_sub = nh.subscribe("/robot/position", 1, &Master::posCallback, this);
    object_position_sub = nh.subscribe("/object/position", 5, &Master::objCallback, this);
    finishedPathSub = nh.subscribe("/robot/finishedPath", 5, &Master::finishedPathCallback, this);
    readyToGoSub = nh.subscribe("/robot/readyToGo", 5, &Master::readyToGoCallback, this);
    pathPlanSub = nh.subscribe("/path_plan", 5, &Master::pathPlanCallback, this);
    newMapSub = nh.subscribe("/map/change", 1, &Master::newMapCallback, this);
    nextSearchNodeSub = nh.subscribe("/master/next_search_node", 5 , &Master::nextSearchNodeCallback, this);
    batterySub = nh.subscribe("/battery", 5, &Master::batteryCallback,this);
    improvedGoalSub = nh.subscribe("/master/object_new_pos",10, &Master::improvedGoalCallback,this);
    classifySub = nh.subscribe("/object/classification", 100, &Master::classifyCallback,this);
    setVelocitySub = nh.subscribe("/set_velocity",1, &Master::setVelocityCallback, this);
    posChangeSub = nh.subscribe("/robot/position_change",1 ,&Master::posChangeCallback,this);

    //pubs & services
    goalPublisher = nh.advertise<geometry_msgs::Pose2D>("/master/object_pos", 10);
    endPointPublisher = nh.advertise<geometry_msgs::Pose>("/path_end_point", 1);
    startPointPublisher = nh.advertise<geometry_msgs::Pose>("/path_start_point", 1);
    pathPublisher = nh.advertise<geometry_msgs::PoseArray>("/path", 5);
    //talkerPublisher = nh.advertise<std_msgs::String>("/talker", 1);
    gripperService = nh.serviceClient<arduino_servo_control::SetServoAngles>("/servo_control/set_servo_angles");
    objectMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/object/markers",10);
    objectPositionPublisher = nh.advertise<geometry_msgs::Pose>("/object_position",10);
    startCollisionAvoidancePublisher = nh.advertise<std_msgs::Bool>("/start_col_avoidance", 5);
    classifyPub = nh.advertise<geometry_msgs::Point>("/classify", 5);
    pathNotFoundPublisher = nh.advertise<geometry_msgs::Pose2D>("/invalid_goal", 5);
    espeakPublisher = nh.advertise<std_msgs::String>("/espeak/string", 5);
    savePublisher = nh.advertise<std_msgs::Bool>("/master/save",5);
  }

  void setVelocityCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    orientation =  (msg->linear.x > -0.001? 1: -1);
    setVelocity = msg->linear.x;
  }

  void posChangeCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
  {
    if(msg->x < 0.005 && msg->y < 0.005 && msg->theta < 0.01 ){
      goingSlow = true;
    }
    else{
      goingSlow = false;
    }
  }

  void posCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
  {
    posX = msg->x;
    posY = msg->y;
    posTheta = msg->theta;
    if(!initializedPosition)
    {
      startX = posX;
      startY = posY;
    }

    initializedPosition = true;
  }
  
  void classifyCallback(const ras_msgs::classified_objectConstPtr& msg)
  {
    ROS_INFO("classify calback");
    if(msg->p > 0.7)//TODO change back to 0.8
    {
      objects.at(objectBeingClassified).name = objectType[msg->name];
      ROS_INFO("the object id: %d is a: %s",objectBeingClassified,objects.at(objectBeingClassified).name.c_str());

      geometry_msgs::Pose pose;
      pose.position.x = objects[objectBeingClassified].posX;
      pose.position.y = objects[objectBeingClassified].posY;
      pose.position.z = msg->name;
      pose.orientation.x = objects[objectBeingClassified].frameX; //Pos in frame x
      pose.orientation.y = objects[objectBeingClassified].frameY; //Pos in frame y
      objectPositionPublisher.publish(pose);
      objects.at(objectBeingClassified).price = 10;//if we have no false detection delete this
    }
    if(msg->p > 0.83)
    {
     if(msg->name == 9)
       objects.at(objectBeingClassified).price = 10000;
     if(msg->name== 12)
       objects.at(objectBeingClassified).price = 10000;

     if(msg->name == 6)
       objects.at(objectBeingClassified).price = 5000;
     if(msg->name == 8)
       objects.at(objectBeingClassified).price = 5000;
     if(msg->name == 1)
       objects.at(objectBeingClassified).price = 5000;
    }




    objectClassFlag = true;
  }

  void improvedGoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
  {
    improvedGoal.x = msg->x;
    improvedGoal.y = msg->y;
    improvedGoalFlag = true;
  }

  /*void collisionCallback(const std_msgs::BoolConstPtr& msg)
  {
    inminentCollision = msg->data;
    if(inminentCollision){//stop
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      ROS_INFO("i see an obstacle in front of me");
    }
  }*/
  
  void collisionCallback(const std_msgs::BoolConstPtr& msg)
  {
    imminentCollision = msg->data;
    if(imminentCollision)
    {
      ROS_INFO("Collision! -> I have to reverse");

      if(state != RECOVERY)
        previousState = state;
      ROS_INFO("Previous state: %d",previousState);
      state = RECOVERY;
      ROS_INFO("RECOVERY");
    }
  }

  void newMapCallback(const std_msgs::BoolConstPtr& msg)
  {
    if(state ==IDLE ||state ==SEARCH ||state ==PLAN_SEARCH_PATH ||state ==RESCUE ||state ==PLAN_RESCUE_PATH
       ||state ==CATCH_OBJECT ||state ==LEAVE_OBJECT ||state ==GET_READY_START ||state ==RECOVERY){
      ROS_INFO("im Remapping because I saw a new wall");//in this case we do nothing, it is safe to continue

    }else if(state == EXECUTE_SEARCH_PATH || state == EXECUTE_RESCUE_PATH){
      ROS_INFO("im Remapping and replanning because I saw a new wall");
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      replanNeeded = true;
    }else if( state == VERIFY_OBJECT_CATCHED){
      replanNeeded = true;


    }else if(state == PARK || state ==  GO_TO_NEXT_SEARCH_NODE){
      ROS_INFO("im Remapping and replanning because I saw a new wall");
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      replanNeeded = true;
      state = EXECUTE_SEARCH_PATH;

    }else if(state == GO_TO_NEXT_RESCUE_NODE){
      ROS_INFO("im Remapping and replanning because I saw a new wall");
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      replanNeeded = true;
      state = EXECUTE_RESCUE_PATH;
    }else if(state ==GO_BACK_TO_START){
      //TODO decide what to do here

      //VERIFY_OBJECT_CATCHED
    }

  }

  void batteryCallback(const geometry_msgs::PoseArrayConstPtr& msg)
  { //ROS_INFO(" Battery Callback Function");
    if(state ==IDLE ||state ==SEARCH ||state ==PLAN_SEARCH_PATH ||state ==RESCUE ||state ==PLAN_RESCUE_PATH
       ||state ==CATCH_OBJECT ||state ==LEAVE_OBJECT ||state ==GET_READY_START ||state ==RECOVERY){
      ROS_INFO("im Remapping because i saw a new battery");//in this case we do nothing, it is safe to continue

    }else if(state == EXECUTE_SEARCH_PATH || state == EXECUTE_RESCUE_PATH ){
      ROS_INFO("im Remapping and replanning because I saw a new battery");
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      replanNeeded = true;

    }else if( state == VERIFY_OBJECT_CATCHED){
      replanNeeded = true;

    }else if(state == PARK || state ==  GO_TO_NEXT_SEARCH_NODE){
      ROS_INFO("im Remapping and replanning because I saw a new battery");
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      replanNeeded = true;
      state = EXECUTE_SEARCH_PATH;

    }else if(state == GO_TO_NEXT_RESCUE_NODE){
      ROS_INFO("im Remapping and replanning because I saw a new battery");
      currentPath.poses.clear();
      pathPublisher.publish(currentPath);
      replanNeeded = true;
      state = EXECUTE_RESCUE_PATH;
    }else if(state ==GO_BACK_TO_START){
      //TODO decide what to do here

      //VERIFY_OBJECT_CATCHED
    }
  }

  void finishedPathCallback(const std_msgs::BoolConstPtr& msg)
  {
    finishedPath = msg->data;
  }

  void readyToGoCallback(const std_msgs::BoolConstPtr& msg)
  {
    readyToGo = msg->data;
  }

  void pathPlanCallback(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    pathPlan.poses = msg->poses;

    if (pathPlan.poses.size() < 2){
      float cause = pathPlan.poses[0].position.y;
      ROS_INFO("cause = %f", cause);
      if(state == EXECUTE_SEARCH_PATH){

        if(cause == -3.0 || cause == -2.0)
        {
          ROS_INFO("could not reach node");
          // Publish to explorer that we need the next search node
          geometry_msgs::Pose2D msg;
          msg.x = searchNode.position.x;
          msg.y = searchNode.position.y;
          pathNotFoundPublisher.publish(msg);
          replanNeeded = true;

          state = EXECUTE_SEARCH_PATH;

        }
        else if(cause == -1.0)
        {
          ROS_INFO("start pose invalid (expl)");
          replanNeeded = true;
          if(state != RECOVERY)
            previousState = state;
          state = RECOVERY;
        }

      }else if(state == EXECUTE_RESCUE_PATH){

        if(cause == -3.0 || cause == -2.0)
        {
          ROS_INFO("could not reach object");
          replanNeeded = true;
          currentRescueObject++;
          state = EXECUTE_RESCUE_PATH;
        }
        else if(cause == -1.0)
        {
          ROS_INFO("start pose invalid (objc)");
          replanNeeded = true;
          if(state != RECOVERY)
            previousState = state;
          state = RECOVERY;

        }

      }else if(state == VERIFY_OBJECT_CATCHED){
        if(cause == -3.0 || cause == -2.0){
          ROS_INFO("Can not plan way to start");
          replanNeeded = true;
          if(state != RECOVERY)
            previousState = state;
          state = RECOVERY;
        }
        else if(cause == -1.0)
        {
          ROS_INFO("start pose invalid (verify)");
          replanNeeded = true;
          if(state != RECOVERY)
            previousState = state;
          state = RECOVERY;

        }
      }
    }
    updatedPathPlan = true;

  }

  void nextSearchNodeCallback(const geometry_msgs::PoseConstPtr& msg)
  {
    searchNode.position.x = msg->position.x;
    searchNode.position.y = msg->position.y;
    searchNode.position.z = msg->position.z;

  }

  void objCallback(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    for(int k = 0; k < msg->poses.size(); ++k){
      bool mapObject = true;
      if(!isnan(msg->poses[k].position.x) && !isnan(msg->poses[k].position.y)){
        //if the position is none or the objest is in the start we do not map it

        if(objects.size()>currentRescueObject && !objects.empty())
        {
          if(fabs(msg->poses[k].position.x-objects[currentRescueObject].posX)<0.12 && fabs(msg->poses[k].position.x-objects[currentRescueObject].posX)<0.12)
          {
            updatedPosition = true;
          }
        }

        /*  save objcets when first seen  */
        if(msg->poses.size()==1)
        {
          for(int i=0;i<objects.size();++i)
          {
            if((fabs(msg->poses[k].position.x-objects[i].posX)<0.08 && fabs(msg->poses[k].position.x-objects[i].posX)<0.08)
               ||(fabs(startX-msg->poses[k].position.x)< 0.25 && fabs(startY-msg->poses[k].position.y)< 0.25))
            {//this means the object is known, then we do not map it
              mapObject = false;
            }
          }
        }
        else //multiple objects
        {
          for(int i=0;i<objects.size();++i)
          {
            if((fabs(msg->poses[k].position.x-objects[i].posX)<0.05 && fabs(msg->poses[k].position.x-objects[i].posX)<0.05)
               ||(fabs(startX-msg->poses[k].position.x)< 0.25 && fabs(startY-msg->poses[k].position.y)< 0.25))
            {//this means the object is known, then we do not map it
              mapObject = false;
            }
          }
        }

        if(mapObject)
        {
          /*  Add object in the internal list  */
          Object o; o.posX = msg->poses[k].position.x; o.posY = msg->poses[k].position.y;
          o.name = "unknown";
          o.price = 1;//TODO add a method to put the correct price here
          objects.push_back(o);
          ROS_INFO("New Object!, id: %d",(int)objects.size());

          geometry_msgs::Pose pose;
          pose.position.x = o.posX;
          pose.position.y = o.posY;
          pose.position.z = 20;
          pose.orientation.x = msg->poses[k].orientation.x; //Pos in frame x
          pose.orientation.y = msg->poses[k].orientation.y; //Pos in frame y
          objectPositionPublisher.publish(pose);

        }

        /* update position and querey classifier */
        if(goingSlow && setVelocity < 0.015)
        {
          if(msg->poses.size()==1)
          {
            for(int i=0;i<objects.size();++i)
            {
              if(fabs(msg->poses[k].position.x-objects[i].posX)<0.08 && fabs(msg->poses[k].position.x-objects[i].posX)<0.08)
              {//this means the object is known, then we want to update the pos
                objects[i].posX = msg->poses[k].position.x;
                objects[i].posY = msg->poses[k].position.y;
                //ROS_INFO("updating object n: %d position",k+1);

                if(objects[i].name == "unknown" && objectClassFlag)
                {
                  ROS_INFO("asking for classification of object");
                  objectBeingClassified = i;
                  geometry_msgs::Point pointAux;
                  pointAux.x = msg->poses[k].orientation.x;//Pos in frame x
                  pointAux.y = msg->poses[k].orientation.y;//Pos in frame y
                  objects[i].frameX = pointAux.x;
                  objects[i].frameY = pointAux.y;
                  classifyPub.publish(pointAux);

                  //classifyPub.publish(pointAux);
                  objectClassFlag = false;
                }
              }
            }
          }
          else if(msg->poses.size()==2 && k==0)
          {
            float score; //this is a clever way of pairing the two objects we are seeing with the ones in memory
            float minScore = 99999;
            float minI,minJ;
            for(int i=0;i<objects.size();++i){
              for(int j=0;j<objects.size();++j){
                if(i != j)
                {
                  score = pow(msg->poses[0].position.x-objects[i].posX,2.0)+
                      pow(msg->poses[0].position.y-objects[i].posY,2.0)+
                      pow(msg->poses[1].position.x-objects[j].posX,2.0)+
                      pow(msg->poses[1].position.y-objects[j].posY,2.0);
                  if(score<minScore && score < 0.01)
                  {
                    minScore = score;
                    minI = i;
                    minJ = j;
                  }
                  //ROS_INFO("score: %f min score: %f",score, minScore);
                }
              }
            }
            if(minScore < 0.01)
            {
              if(objects[minI].name == "unknown" && objectClassFlag)
              {
                ROS_INFO("asking for classification of object");
                objectBeingClassified = minI;
                geometry_msgs::Point pointAux;
                pointAux.x = msg->poses[0].orientation.x;
                pointAux.y = msg->poses[0].orientation.y;
                objects[minI].frameX = pointAux.x;
                objects[minI].frameY = pointAux.y;
                classifyPub.publish(pointAux);
                classifyPub.publish(pointAux);
                objectClassFlag = false;
              }
              else if(objects[minJ].name == "unknown" && objectClassFlag)
              {
                ROS_INFO("asking for classification of object");
                objectBeingClassified = minJ;
                geometry_msgs::Point pointAux;
                pointAux.x = msg->poses[1].orientation.x;
                pointAux.y = msg->poses[1].orientation.y;
                objects[minJ].frameX = pointAux.x;
                objects[minJ].frameY = pointAux.y;
                classifyPub.publish(pointAux);
                classifyPub.publish(pointAux);
                objectClassFlag = false;
              }
              objects[minI].posX = msg->poses[0].position.x;
              objects[minI].posY = msg->poses[0].position.y;
              objects[minJ].posX = msg->poses[1].position.x;
              objects[minJ].posY = msg->poses[1].position.y;
            }
          }
        }

        if(mapObject)
        {
          /*  does the apropiate action depending on the state  */

          if(state ==IDLE ||state ==SEARCH ||state ==PLAN_SEARCH_PATH ||state ==RESCUE ||state ==PLAN_RESCUE_PATH
             ||state ==CATCH_OBJECT ||state ==LEAVE_OBJECT ||state ==GET_READY_START ||state ==RECOVERY){
            ROS_INFO("im Remapping because i saw a new object");//in this case we do nothing, it is safe to continue

          }else if(state == EXECUTE_SEARCH_PATH || state == EXECUTE_RESCUE_PATH){
            ROS_INFO("im Remapping and replanning because I saw a new object");
            currentPath.poses.clear();
            pathPublisher.publish(currentPath);
            replanNeeded = true;

          }else if( state == VERIFY_OBJECT_CATCHED){
            replanNeeded = true;

          }else if(state == PARK || state ==  GO_TO_NEXT_SEARCH_NODE){
            ROS_INFO("im Remapping and replanning because I saw a new object");
            currentPath.poses.clear();
            pathPublisher.publish(currentPath);
            replanNeeded = true;
            state = EXECUTE_SEARCH_PATH;

          }else if(state == GO_TO_NEXT_RESCUE_NODE){
            ROS_INFO("im Remapping and replanning because I saw a new object");
            currentPath.poses.clear();
            pathPublisher.publish(currentPath);
            replanNeeded = true;
            state = EXECUTE_RESCUE_PATH;
          }else if(state ==GO_BACK_TO_START){
            //TODO decide what to do here

            //VERIFY_OBJECT_CATCHED
          }

        }



      }
    }

    /*  Print objects in rviz  */
    visualization_msgs::Marker objMarker;
    visualization_msgs::MarkerArray objMarkerArray;
    objMarkerArray.markers.clear();
    for(int i=0; i<objects.size(); ++i)
    {
      objMarker.pose.position.x = objects[i].posX;
      objMarker.pose.position.y = objects[i].posY;
      objMarker.header.frame_id = "/map";
      objMarker.header.stamp = ros::Time();
      objMarker.ns = "node";
      objMarker.type = visualization_msgs::Marker::SPHERE;
      objMarker.action = visualization_msgs::Marker::MODIFY;
      objMarker.pose.orientation.x = 0.0;
      objMarker.pose.orientation.y = 0.0;
      objMarker.pose.orientation.z = 0.0;
      objMarker.pose.orientation.w = 1.0;
      objMarker.scale.x = 0.05;
      objMarker.scale.y = 0.05;
      objMarker.scale.z = 0.05;
      objMarker.color.a = 1.0; // Don't forget to set the alpha!
      objMarker.color.r = 0.95;
      objMarker.color.g = 0.95;
      objMarker.color.b = 0.95;
      objMarker.id = i ;
      objMarkerArray.markers.push_back(objMarker);
    }
    objectMarkerPublisher.publish(objMarkerArray);
  }

  void handleObject(int desiredGripperState)
  {
    arduino_servo_control::SetServoAngles angles;
    if(desiredGripperState == 0)//0:Closed 1:Opened
    {
      angles.request.angle_servo_0= 60;
      angles.request.angle_servo_1 = 120;
    }
    else if(desiredGripperState == 1)
    {
      angles.request.angle_servo_0 = 0;
      angles.request.angle_servo_1 = 180;
    }
    else if(desiredGripperState == 2)
    {
      angles.request.angle_servo_0 = 40;
      angles.request.angle_servo_1 = 140;
    }
    else if(desiredGripperState == 3)
    {
      angles.request.angle_servo_0 = 30;
      angles.request.angle_servo_1 = 150;
    }
    gripperService.call(angles);
  }

  void callPathPlanner(double startX, double startY, double endX, double endY)//querys path planner
  {
    geometry_msgs::Pose nodeAux;
    nodeAux.position.x = startX;
    nodeAux.position.y = startY;
    startPointPublisher.publish(nodeAux);
    nodeAux.position.x = endX;
    nodeAux.position.y = endY;
    endPointPublisher.publish(nodeAux);
    updatedPathPlan = false;
  }

  void waitForPathPlanner(ros::Rate loop_rate)//blocking function that waits for message to arrive
  {
    while(!updatedPathPlan && ros::ok() && nh.ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("I recived a path plan");
    currentPath.poses = pathPlan.poses;
  }

  void callDriver(double mode)//1: explore 2:rescue 3:catch
  {
    currentPath.poses[0].orientation.w = mode;
    pathPublisher.publish(currentPath);//we call driver jut before going to next state
    finishedPath = false;
  }

  void stop(ros::Rate loop_rate)
  {
    currentPath.poses.clear();
    pathPublisher.publish(currentPath);
    readyToGo = false;
    while(!readyToGo && ros::ok() && nh.ok())//waits for it to stop
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");
  Master master;
  ros::Rate loop_rate(10);
  std_msgs::String start_msg;
  start_msg.data = "Start";

  /*   reading object from file if available   */
  std::ifstream object_file("/home/ras28/catkin_ws/src/ras_project/ras_data/master_objects.csv", std::ifstream::in);
  if(!object_file.fail()) {
    std::string var;
    for (std::string line; std::getline(object_file, var, ',');)
    {
      Object o;
      o.name = var;
      std::getline(object_file, var,',');
      o.posX = std::atof(var.c_str());
      std::getline(object_file, var,',');
      o.posY = std::atof(var.c_str());
      std::getline(object_file, var,',');
      o.price = std::atoi(var.c_str());
      master.objects.push_back(o);
      ROS_INFO("read obj x: %f y: %f",o.posX,o.posY);


    }
    object_file.close();
  }

  while (ros::ok() && master.nh.ok())
  {
    switch (master.state) {
    case IDLE:
    {

      int pressedKey = 0;
      ROS_INFO(" 1 for search         2 for rescue        3 to save state in file");
      while( pressedKey == 0 && ros::ok() && master.nh.ok() )
      {
        std::cin >> pressedKey;
        ros::spinOnce();
        loop_rate.sleep();
      }
      while(!master.initializedPosition  && ros::ok() && master.nh.ok() )//this waits for position to be initialized before doing anything
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

      if(pressedKey == 3)
      {

        ROS_INFO("---------------SAVING STATE---------------");
        std::ofstream outfile("/home/ras28/catkin_ws/src/ras_project/ras_data/master_objects.csv");
        for(int i = 0; i < master.objects.size(); ++i){
          outfile <<master.objects.at(i).name << ',';
          outfile <<master.objects.at(i).posX << ',';
          outfile <<master.objects.at(i).posY << ',';
          outfile <<master.objects.at(i).price<< ',';
        }
        outfile.close();

        std_msgs::Bool msgAux;
        msgAux.data = true;
        master.savePublisher.publish(msgAux);
        ROS_INFO("wait a sec.");

      }
      else if(pressedKey == 2)
      {
        master.state = RESCUE;
        ROS_INFO("-------------------RESCUE------------------");
        std_msgs::Bool start;
        start.data = true;
        std::stable_sort (master.objects.begin(), master.objects.end(),sortFun);
        master.startCollisionAvoidancePublisher.publish(start);

      }
      else if(pressedKey == 1)
      {
        master.state = SEARCH;
        ROS_INFO("-------------------SEARCH------------------");
        std_msgs::Bool start;
        start.data = true;
        master.startCollisionAvoidancePublisher.publish(start);

      }
    }
      break;

    case SEARCH:
    {

      master.state = PLAN_SEARCH_PATH;
      ROS_INFO("PLAN_SEARCH_PATH");

    }
      break;

    case PLAN_SEARCH_PATH://this is not used, now we have a on request explorer node
    {
      master.state = EXECUTE_SEARCH_PATH;
      ROS_INFO("EXECUTE_SEARCH_PATH");
      master.espeakPublisher.publish(start_msg);
      master.begin = ros::Time::now();

    }
      break;

    case EXECUTE_SEARCH_PATH:
    {
      master.handleObject(2); //0:closed 1:open



      if(fabs(master.searchNode.position.z + 1.0) < 0.001 || numberOfExploredNodes > 40)//this means that the explorer node is finished
      {
        master.replanNeeded = false;
        master.callPathPlanner(posX, posY, startX, startY);
        master.waitForPathPlanner(loop_rate);
        if(!master.replanNeeded){
          master.callDriver(1.0);//1: explore 2:rescue 3:catch
          master.state = PARK;
          ROS_INFO("PARK");
        }
      }
      else
      {
        master.replanNeeded = false;
        master.callPathPlanner(posX, posY, master.searchNode.position.x,  master.searchNode.position.y);
        master.waitForPathPlanner(loop_rate);
        /*geometry_msgs::PoseArray samplePath; //hardcoded path
        geometry_msgs::Pose node;
        samplePath.poses.clear();

        node.position.x = 2.15;
        node.position.y = 0.25;
        samplePath.poses.push_back(node);
        node.position.x = 0.62;
        node.position.y = 0.46;
        samplePath.poses.push_back(node);
        node.position.x = 0.62;
        node.position.y = 0.95;
        samplePath.poses.push_back(node);
        master.currentPath.poses = samplePath.poses;*/
        if(!master.replanNeeded)
        {
          master.callDriver(1.0);//1: explore 2:rescue 3:catch
          master.state = GO_TO_NEXT_SEARCH_NODE;
          ROS_INFO("GO_TO_NEXT_SEARCH_NODE");
        }
        numberOfExploredNodes ++;

      }
    }
      break;

    case PARK:
    {
      if(master.finishedPath)
      {
        master.state = IDLE;
      }
      else
      {
        master.state = PARK;
      }
    }
      break;


    case RECOVERY:
    {
      master.readyToGo = false;
      master.currentPath.poses.clear();
      master.pathPublisher.publish(master.currentPath);
      //ROS_INFO("resetting driver");
      while(!master.readyToGo && ros::ok() && master.nh.ok())//waits for it to stop
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

      //ROS_INFO("going back");

      double back_dist = 0.15;
      phi = (master.orientation > 0.0 ? (M_PI) : 0.0)  + posTheta + (master.catchingAtempts>0? ((float) (master.catchingAtempts %3))-1.5 :0.0);
      x = posX + back_dist * cos(phi);
      y = posY + back_dist * sin(phi);

      geometry_msgs::Pose node;
      master.currentPath.poses.clear();
      node.position.x = posX;
      node.position.y = posY;
      master.currentPath.poses.push_back(node);
      node.position.x = x;
      node.position.y = y;
      node.orientation.z = (master.orientation > 0.0? 180.0 : 0);
      master.currentPath.poses.push_back(node);
      master.pathPublisher.publish(master.currentPath);
      master.catchingAtempts ++;
      master.callDriver(1.0);//1: explore 2:rescue 3:catch
      ROS_INFO("went back");
      master.replanNeeded = true;

      while(!master.finishedPath && ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
      //ROS_INFO("resetted driver for the second time");
      master.state = master.previousState;

      if(master.state ==IDLE ||master.state ==SEARCH ||master.state ==PLAN_SEARCH_PATH ||master.state ==RESCUE
         ||master.state ==PLAN_RESCUE_PATH ||master.state ==LEAVE_OBJECT ||master.state ==GET_READY_START ||master.state ==RECOVERY){

        //ROS_INFO("");//in this case we do nothing, it is safe to continue

      }else if(master.state ==CATCH_OBJECT ){

        master.catchingAtempts ++;
        master.handleObject(0); //0:closed 1:open
        ROS_INFO("EXECUTE_RESCUE_PATH");
        master.state = EXECUTE_RESCUE_PATH;

      }else if(master.state == EXECUTE_SEARCH_PATH || master.state == EXECUTE_RESCUE_PATH || master.state == VERIFY_OBJECT_CATCHED){
        //ROS_INFO("");
        master.replanNeeded = true;

      }else if(master.state == PARK || master.state ==  GO_TO_NEXT_SEARCH_NODE){
        //ROS_INFO("");
        master.replanNeeded = true;
        ROS_INFO("EXECUTE_SEARCH_PATH");
        master.state = EXECUTE_SEARCH_PATH;

      }else if(master.state == GO_TO_NEXT_RESCUE_NODE){
        master.replanNeeded = true;
        ROS_INFO("EXECUTE_RESCUE_PATH");
        master.state = EXECUTE_RESCUE_PATH;

      }else if(master.state ==GO_BACK_TO_START){

        master.replanNeeded = false;
        master.callPathPlanner(posX, posY, startX, startY);
        master.waitForPathPlanner(loop_rate);
        if(!master.replanNeeded){
          master.callDriver(2.0);//1: explore 2:rescue 3:catch
          master.state = GO_BACK_TO_START;
          ROS_INFO("GO_BACK_TO_START");
        }

      }
      master.imminentCollision = false;

    }
      break;

    case GO_TO_NEXT_SEARCH_NODE:
    {
      if(master.finishedPath)
      {
        master.state = EXECUTE_SEARCH_PATH;
        ROS_INFO("EXECUTE_SEARCH_PATH");
      }
      else
      {
        if((ros::Time::now()-master.begin)>=ros::Duration(60*5-0.06))
        {
          ROS_INFO("Time is up! %f sec elapsed since start",(ros::Time::now()-master.begin).toSec());

          ROS_INFO("---------------SAVING STATE---------------");

          std::ofstream outfile("/home/ras28/catkin_ws/src/ras_project/ras_data/master_objects.csv");
          for(int i = 0; i < master.objects.size(); ++i){
            outfile <<master.objects.at(i).name << ',';
            outfile <<master.objects.at(i).posX << ',';
            outfile <<master.objects.at(i).posY << ',';
            outfile <<master.objects.at(i).price<< ',';
          }
          outfile.close();

          std_msgs::Bool msgAux;
          msgAux.data = true;
          master.savePublisher.publish(msgAux);
          ROS_INFO("wait a sec.");


          loop_rate.sleep();

          master.stop(loop_rate);
          while(ros::ok() && master.nh.ok())
          {
            loop_rate.sleep();
          }
        }
        master.state = GO_TO_NEXT_SEARCH_NODE;
      }
    }
      break;

    case RESCUE:
    {
      master.espeakPublisher.publish(start_msg);
      master.state = PLAN_RESCUE_PATH;
      ROS_INFO("PLAN_RESCUE_PATH");
    }
      break;

    case PLAN_RESCUE_PATH:
    {//here we will decide in what order to attempt the rescue, close and pricey object are more valuable of course

      /*
      //hard coded object
      objectX = 0.25;
      objectY = 1.19;
      master.updatedPosition = true;
      ROS_INFO("New Object!, id: %d",(int)master.objects.size()+1);
      Object o; o.posX = objectX; o.posY = objectY;
      o.name = "unknown object"; o.price = 1;
      master.objects.push_back(o);
*/

      master.currentRescueObject = 0;
      master.state = EXECUTE_RESCUE_PATH;
      ROS_INFO("EXECUTE_RESCUE_PATH");
    }
      break;

    case EXECUTE_RESCUE_PATH:
    {
      master.handleObject(2); //0:closed 1:open 2:half open(for driving safely)

      if(master.currentRescueObject == master.objects.size())
      {
        ROS_INFO("no more objects left to catch");
        master.replanNeeded = false;
        master.callPathPlanner(posX, posY, startX, startY);
        master.waitForPathPlanner(loop_rate);
        if(!master.replanNeeded){
          master.callDriver(2.0);//1: explore 2:rescue 3:catch
          master.state = PARK;
          ROS_INFO("PARK");
        }
      }
      else
      {

        while(!master.initializedPosition && ros::ok() && master.nh.ok())
        {
          ros::spinOnce();
          loop_rate.sleep();
        }
        master.replanNeeded = false;
        master.currentRescueObject--;

        do{

          master.currentRescueObject++;
          master.improvedGoalFlag = false;
          geometry_msgs::Pose2D auxPos;
          auxPos.x = master.objects[master.currentRescueObject].posX;
          auxPos.y = master.objects[master.currentRescueObject].posY;
          master.improvedGoal.x = auxPos.x;
          master.improvedGoal.y = auxPos.y;
          ROS_INFO("Current object id: %d at X: %f Y: %f",master.currentRescueObject+1, auxPos.x, auxPos.y);
          master.goalPublisher.publish(auxPos);
          master.begin_point_updating = ros::Time::now();


          while(!master.improvedGoalFlag && ros::ok() && master.nh.ok() && (ros::Time::now()-master.begin_point_updating)<ros::Duration(1.0))
          {
            ros::spinOnce();
            loop_rate.sleep();
          }

        }
        while((master.improvedGoal.x + 1.0)<0.001 && fabs(master.improvedGoal.y + 1.0)<0.001);//if the node can not find the position we go for another object

        ROS_INFO("Current object id: %d at X: %f Y: %f",master.currentRescueObject+1, master.objects[master.currentRescueObject].posX, master.objects[master.currentRescueObject].posY);
        master.improvedGoalFlag = false;
        master.callPathPlanner(posX, posY, master.improvedGoal.x, master.improvedGoal.y);
        master.waitForPathPlanner(loop_rate);
        if(!master.replanNeeded)
        {
          master.callDriver(2.0);//1: explore 2:rescue 3:catch
          master.state = GO_TO_NEXT_RESCUE_NODE;
          ROS_INFO("GO_TO_NEXT_RESCUE_NODE");

        }
      }


    }
      break;

    case GO_TO_NEXT_RESCUE_NODE:
    {
      if(fabs(posX - master.objects[master.currentRescueObject].posX) < master.catchingDistance && fabs(posY - master.objects[master.currentRescueObject].posY) < master.catchingDistance )
      {
        master.handleObject(1); //0:closed 1:open
        master.stop(loop_rate);
        master.updatedPosition = true;
        objectX = master.objects[master.currentRescueObject].posX;
        objectY = master.objects[master.currentRescueObject].posY;
        master.finishedPath = false;


        master.state = CATCH_OBJECT;
        ROS_INFO("CATCH_OBJECT");

      }
      else
      {
        master.state = GO_TO_NEXT_RESCUE_NODE;
      }
    }
      break;

    case CATCH_OBJECT:
    {
      float mode = 1.0;
      for(int i=0;i<master.objects.size();++i)
      {
        if(master.currentRescueObject!=i && fabs(master.objects[master.currentRescueObject].posX - master.objects[i].posX)< 0.08 && fabs(master.objects[master.currentRescueObject].posY - master.objects[i].posY)< 0.08 )
        {
          mode = 3.0;
          break;
        }
      }
      master.handleObject(mode);

      if(master.catchingAtempts >2)
      {
        master.currentRescueObject++;
        ROS_INFO("I lost interest in this object interest");
        master.state = EXECUTE_RESCUE_PATH;
        ROS_INFO("EXECUTE_RESCUE_PATH");
        master.catchingAtempts = 0;
      }
      else
      {


        if(master.finishedPath)
        {
          master.handleObject(0); //0:closed 1:open
          master.state = VERIFY_OBJECT_CATCHED;
          ROS_INFO("VERIFY_OBJECT_CATCHED");
          master.updatedPosition = false;
        }
        else
        {
          if(master.updatedPosition)
          {
            geometry_msgs::Pose node;
            master.currentPath.poses.clear();
            node.position.x = posX;
            node.position.y = posY;
            master.currentPath.poses.push_back(node);
            node.position.x = master.objects[master.currentRescueObject].posX;
            node.position.y = master.objects[master.currentRescueObject].posY;
            master.currentPath.poses.push_back(node);
            master.currentPath.poses[0].orientation.w = 3.0;//1: explore 2:rescue 3:catch
            master.pathPublisher.publish(master.currentPath);//we update the last call with the new more"precise" data
            master.updatedPosition = false;

          }
          else
          {//wait till driver arrives
            master.state = CATCH_OBJECT;
          }
        }
      }
    }
      break;

    case VERIFY_OBJECT_CATCHED://todo limit the amount of time this can be runned
    {

      master.stop(loop_rate);

      double back_dist = 0.10;//go backwards
      phi =  constrainAngle(M_PI + posTheta);
      x = posX + back_dist * cos(phi);
      y = posY + back_dist * sin(phi);

      geometry_msgs::Pose node;
      master.currentPath.poses.clear();
      node.position.x = posX;
      node.position.y = posY;
      master.currentPath.poses.push_back(node);
      node.position.x = x;
      node.position.y = y;
      node.orientation.z = 180.0;
      master.currentPath.poses.push_back(node);
      master.callDriver(1.0);//1: explore 2:rescue 3:catch
      while(!master.finishedPath && ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
      master.stop(loop_rate);

      if(!master.updatedPosition)
      {
        ROS_INFO("I catched object id: %d", master.currentRescueObject+1);
        master.replanNeeded = false;
        master.callPathPlanner(posX, posY, startX, startY);
        master.waitForPathPlanner(loop_rate);
        if(!master.replanNeeded){
          master.callDriver(2.0);//1: explore 2:rescue 3:catch
          master.state = GO_BACK_TO_START;
          ROS_INFO("GO_BACK_TO_START");
        }

      }
      else
      {
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();

        master.state = CATCH_OBJECT;
        ++ master.catchingAtempts ;
        master.finishedPath = false;
      }
    }
      break;

    case GO_BACK_TO_START:
    {
      if(master.finishedPath){
        master.state = LEAVE_OBJECT ;
        ROS_INFO("LEAVE_OBJECT");
      }
    }
      break;

    case LEAVE_OBJECT:
    {
      master.handleObject(1.0); //0:closed 1:open
      // go bakwards

      double back_dist = 0.15;
      phi = M_PI + posTheta;
      x = posX + back_dist * cos(phi);
      y = posY + back_dist * sin(phi);

      geometry_msgs::Pose node;
      master.currentPath.poses.clear();
      node.position.x = posX;
      node.position.y = posY;
      node.orientation.w = 2.0;//1: explore 2:rescue 3:catch
      master.currentPath.poses.push_back(node);
      node.position.x = x;
      node.position.y = y;
      node.orientation.z = 180.0;
      master.currentPath.poses.push_back(node);
      master.pathPublisher.publish(master.currentPath);
      master.finishedPath = false;

      master.state = GET_READY_START;
      ROS_INFO("GET_READY_START");
    }
      break;

    case GET_READY_START:
    {

      if(master.finishedPath){
        master.state = EXECUTE_RESCUE_PATH ;
        ROS_INFO("EXECUTE_RESCUE_PATH");
        master.currentRescueObject++;
        master.catchingAtempts = 0;
      }
      else{
        master.state =  GET_READY_START ;
      }
    }
      break;

    default:
    {
      ROS_INFO("something has gone wrong I am at the default case in the master state machine, going to IDLE");
      master.state = IDLE;
      ROS_INFO("IDLE");
    }
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
