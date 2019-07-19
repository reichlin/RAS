#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <ras_msgs/detected_objects.h>

int MAX_NUMBER_OBJECTS_PER_FRAME = 5;
int RADIUS_PCL_WINDOW = 5;

class Object {
public:
  int px;
  int py;
  float x, y, z;
  bool set;

};

std::vector<Object> objects;


void receiveObjectsCallback(const ras_msgs::detected_objects::ConstPtr& msg)
{
  if(msg->N_points > MAX_NUMBER_OBJECTS_PER_FRAME) {
    //ROS_INFO("found too many objects: %d", msg->N_points);
    //ROS_INFO("frame ignored");
    return;
  }

  for(int i = 0; i < msg->N_points; i++) {
    if(isnan(msg->objects.at(i).x) || isnan(msg->objects.at(i).y))
      continue;
    
    int x = (int) msg->objects.at(i).x;
    int y = (int) msg->objects.at(i).y;
    if(x >= 0 && x < 640 && y >= 0 && y < 480) {
      Object obj;
      obj.px = x;
      obj.py = y;
      obj.set = true;
      objects.push_back(obj);
    }
  }

}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  for(int i = 0; i < objects.size(); i++) {

    if(objects.at(i).set) {

      float x_m=0.0, y_m=0.0, z_m=0.0;
      int N_pcl=0;

      for(int j = -RADIUS_PCL_WINDOW; j <= RADIUS_PCL_WINDOW; j++) {
        for(int k = -RADIUS_PCL_WINDOW; k <= RADIUS_PCL_WINDOW; k++) {

          float x, y, z;
          int pos = msg->row_step * (objects.at(i).py+j) + 32 * (objects.at(i).px+k); // 16
          memcpy(&x, &msg->data[pos], sizeof(float));
          pos += 4;
          memcpy(&y, &msg->data[pos], sizeof(float));
          pos += 4;
          memcpy(&z, &msg->data[pos], sizeof(float));

          if(!isnan(x) && !isnan(y) && !isnan(z)) {
            //ROS_INFO("something is not nan");
            x_m = (x_m * N_pcl + x) / (float) (N_pcl + 1);
            y_m = (y_m * N_pcl + y) / (float) (N_pcl + 1);
            z_m = (z_m * N_pcl + z) / (float) (N_pcl + 1);
            N_pcl++;
          } //else
          //ROS_INFO("x: %f, y: %f, z: %f", x, y, z);
        }
      }

      if(N_pcl > 0) {
        objects.at(i).x = x_m;
        objects.at(i).y = y_m;
        objects.at(i).z = z_m;
        objects.at(i).set = false;
      } else {
        objects.erase(objects.begin() + i);
        i--;
        //ROS_INFO("all nan inside the window");
      }

    }
  }
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "place_object");
  ros::NodeHandle n;

  //n.getParam("/frequency", frequency);

  ros::Rate loop_rate(10.0);

  ros::Subscriber sub_object = n.subscribe("/object/detection", 100, receiveObjectsCallback);
  ros::Subscriber sub_cloud = n.subscribe("/camera/depth_registered/points", 100, cloudCallback);
  ros::Publisher object_pub = n.advertise<geometry_msgs::PoseArray>("/object/position",5);

  tf::TransformListener listener;
  geometry_msgs::PointStamped camera_point;
  geometry_msgs::PointStamped base_point;


  float x, y, z;
  
  bool classify = false;
  geometry_msgs::Point point;
  geometry_msgs::PoseArray object_message;

  while(n.ok()) {

    //ROS_INFO("start");
    
    classify = false;
    object_message.poses.clear();
    
    ros::spinOnce();
    int marker_count = 0;

    for(int i = 0; i < objects.size(); i++) {

      // for all setted (found a position in x, y, z camera frame) object found
      if(!objects.at(i).set) {

        x = objects.at(i).x;
        y = objects.at(i).y;
        z = objects.at(i).z;
        float N = 1.0;

        // cluster them with other points if close enough, 5 cm
        for(int j = i+1; j < objects.size(); j++) {
          if(!objects.at(j).set) {
            if(fabs(x - objects.at(j).x) < 0.04 && fabs(y - objects.at(j).y) < 0.04 && fabs(z - objects.at(j).z) < 0.04) {
              x = (N * x + objects.at(j).x)/(N + 1.0);
              y = (N * y + objects.at(j).y)/(N + 1.0);
              z = (N * z + objects.at(j).z)/(N + 1.0);
              N++;
              objects.erase(objects.begin() + j);
              j--;
            }
          }
        }

        //ROS_INFO("point in camera coordinates: %f %f %f", x, y, z);

        // try to conver the coordinates in the map reference frame
        try {

          camera_point.header.frame_id = "/camera_link";
          camera_point.header.stamp = ros::Time(0);

          camera_point.point.x = x;
          camera_point.point.y = y;
          camera_point.point.z = z;
          listener.transformPoint("/map", camera_point, base_point);

          // if height is in the correct range consider the object to be valid and save it in the next message

          geometry_msgs::Pose pose;

          if(base_point.point.z > 0.015 && base_point.point.z < 0.05) {
            pose.position.x = base_point.point.x;
            pose.position.y = base_point.point.y;
            pose.position.z = base_point.point.z;
            pose.orientation.x = objects.at(i).px;
            pose.orientation.y = objects.at(i).py;
            //this line is for debugging, tells me if there is more than one object in frame
            pose.orientation.z = object_message.poses.size();
            object_message.poses.push_back(pose);


          } else {
            ROS_INFO("object not published X: %f Y: %f Z: %f", base_point.point.x, base_point.point.y, base_point.point.z);
          }

          objects.erase(objects.begin() + i);
          i--;

        } catch(tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep(); // Questo va cambiato
        }


      }

    }


    if(!object_message.poses.empty())
      object_pub.publish(object_message);

    loop_rate.sleep();
  }

}

