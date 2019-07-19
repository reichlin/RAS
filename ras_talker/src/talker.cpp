#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <ras_msgs/RAS_Evidence.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


float pos_x, pos_y, image_x, image_y;
int top_x, top_y, bottom_x, bottom_y;
bool object_received = false;
bool image_received = false;
int name_temp;
std::string name;
std::string ob;
std::vector<std::string> objects;
std::vector<std::string> convert_objects;
sensor_msgs::Image image_p;


void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (object_received)
  {
    //ROS_INFO("Getting image");
    cv_bridge::CvImagePtr cv_ptr;

    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception is %s:", e.what());
      return;
    }

    top_x = int(image_x) - 100;
    top_y = int(image_y) - 100;

    if (top_x >= 440)
      top_x = 439;
    if (top_x < 0)
      top_x = 0;
    if (top_y >= 480)
      top_y = 479;
    if (top_y < 0)
      top_y = 0;

    cv::Mat dImg;
    dImg = cv_ptr->image(cv::Rect(top_x, top_y, 200, 200));

    cv_bridge::CvImage resizeRos;
    resizeRos.encoding = "bgr8";
    resizeRos.image = dImg;
    image_p = *resizeRos.toImageMsg();
    //image_p.width = 200;
    //image_p.height = 200;
    //image_p.step  = 8*200*3;

    image_received = true;

  }

}

void ObjectCallback(const geometry_msgs::Pose::ConstPtr& msg){

    //ROS_INFO("Getting position");
    pos_x = msg->position.x;
    pos_y = msg->position.y;
    image_x = msg->orientation.x;
    image_y = msg->orientation.y;
    name_temp = msg->position.z;

    if(name_temp == 20){
      ob = "an_object";
      name = "An Object";
    }
    else{
      ob = objects[name_temp];
      name = convert_objects[name_temp];
    }

    object_received = true;
}

int main(int argc, char **argv)
{
  objects.push_back("blue_cube");
  objects.push_back("blue_triangle");
  objects.push_back("green_cube");
  objects.push_back("green_cylinder");
  objects.push_back("green_hollow_cube");
  objects.push_back("orange_cross");
  objects.push_back("patric");
  objects.push_back("purple_cross");
  objects.push_back("purple_star");
  objects.push_back("red_cube");
  objects.push_back("red_cylinder");
  objects.push_back("red_ball");
  objects.push_back("yellow_ball");
  objects.push_back("yellow_cube");

  convert_objects.push_back("Blue Cube");
  convert_objects.push_back("Blue Triangle");
  convert_objects.push_back("Green Cube");
  convert_objects.push_back("Green Cylinder");
  convert_objects.push_back("Green Hollow Cube");
  convert_objects.push_back("Orange Cross");
  convert_objects.push_back("Patric");
  convert_objects.push_back("Purple Cross");
  convert_objects.push_back("Purple Star");
  convert_objects.push_back("Red Cube");
  convert_objects.push_back("Red Cylinder");
  convert_objects.push_back("Red Ball");
  convert_objects.push_back("Yellow Ball");
  convert_objects.push_back("Yellow Cube");


  ros::init(argc, argv, "talker_node");
  ros::NodeHandle n_;
  image_transport::ImageTransport it_(n_);
  image_transport::Subscriber image_sub;
  ros::Rate loop_rate(5);

  image_sub = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCallback);
  ros::Subscriber position_sub = n_.subscribe<geometry_msgs::Pose>("/object_position",10 , ObjectCallback);

  ros::Publisher espeak_pub = n_.advertise<std_msgs::String>("/espeak/string", 1);
  ros::Publisher evidence_pub = n_.advertise<ras_msgs::RAS_Evidence>("/evidence", 2);
  //ros::Publisher image_evidence_pub = n_.advertise<sensor_msgs::Image>("/image_evidence", 1);


  std_msgs::String msg;
  //Location of the object in the /map frame
  //we only care about X,Y position coordinate
  geometry_msgs::TransformStamped object_location;

  
  while (ros::ok() && n_.ok())
  {	

    if (image_received && object_received)
    {
      std::string message = "I see a " + name;
      msg.data = message;
      espeak_pub.publish(msg);
      // publish evidence
      ras_msgs::RAS_Evidence evidence;
      evidence.stamp = ros::Time::now();

      evidence.group_number = 8;
      evidence.object_id = ob;
      evidence.object_location.transform.translation.x = pos_x;
      evidence.object_location.transform.translation.y = pos_y;
      evidence.image_evidence = image_p;
      evidence.image_evidence.header.frame_id = "base_link";
      evidence_pub.publish(evidence);
      //image_evidence_pub.publish(image_p);

      image_received = false;
      object_received = false;
  	}

  	ros::spinOnce();
  	loop_rate.sleep();
  }
 
return 0;

}
