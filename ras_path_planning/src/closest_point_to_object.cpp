#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include "std_msgs/Bool.h"


// global variables

int cell_h, cell_w;
float map_granularity;
int** map = NULL;
bool mapRead = false;
bool readNewMap = false;
double frequency = 30.0;

float goal[2];
bool haveGoal = false;


// read extended map callback
void readMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {

    if(!mapRead) {
        cell_h = msg->info.height;
        cell_w = msg->info.width;
        map_granularity = msg->info.resolution;
        
        int c = 0;
        
        map = new int*[cell_h];
        for(int i = 0; i < cell_h; i++) {
            map[i] = new int[cell_w];
            for(int j = 0; j < cell_w; j++) {
                map[i][j] = msg->data[c];
                c++;
            }
        }
        
        mapRead = true;
        
    } else if(readNewMap) {
        int c = 0;
        
        for(int i = 0; i < cell_h; i++) {
        
            for(int j = 0; j < cell_w; j++) {
                map[i][j] = msg->data[c];
                c++;
            }
        }
        
        readNewMap = false;
    }
}

// map has changed, read it again
void changeMapCallback(const std_msgs::Bool::ConstPtr &msg) {
    if(mapRead)
        readNewMap = true;
}

void goalCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
    goal[0] = msg->x;
    goal[1] = msg->y;
    haveGoal = true;
}

int main(int argc, char** argv) {

    int counter;
    double forceX=0.0, forceY=0.0;
    
    int Fx=0, Fy=0;
    int max=0;
    int dx, dy;
    int goalx, goaly, newgoalx, newgoaly;
    int threshold;
    
    
    
    ros::init(argc, argv, "closest_point_to_object");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    nh.getParam("/frequency", frequency);
    nh.getParam("/closest_point_to_object/threshold", threshold);
    
    ros::Rate r(frequency);
    
    ros::Subscriber sub_map = n.subscribe("/map/extended", 1, readMapCallback);
    ros::Subscriber sub_map_change = n.subscribe("/map/change", 10, changeMapCallback);
    ros::Subscriber sub_goal = n.subscribe("/master/object_pos", 10, goalCallback);
    
    ros::Publisher pos_pub = n.advertise<geometry_msgs::Pose2D>("/master/object_new_pos", 10);
        
    geometry_msgs::Pose2D new_pos;
    
    
    
    while(n.ok()) {
    
        // read messages and update believed position based on odometry
        
        ros::spinOnce();

        if(haveGoal && mapRead) {
        
          haveGoal = false;

          goalx = (int) round((float) goal[0]/map_granularity) + floor((float) cell_w/4.0);
          goaly = (int) round((float) goal[1]/map_granularity) + floor((float) cell_h/4.0);

          //ROS_INFO("goalx: %d, goaly: %d", goalx, goaly);

          if(map[goaly][goalx] > 98 || (goalx < ((float) cell_w/4.0) || goalx >= ((float) (3.0*cell_w)/4.0) || goaly < ((float) cell_h/4.0) || goaly >= ((float) (3.0*cell_h)/4.0))) {
              new_pos.x = -1;
              new_pos.y = -1;
          } else {

              counter = 0;

              while(map[goaly][goalx] >= threshold) {

                  counter++;
                  Fx=0;
                  Fy=0;
                  max=0;

                  for(dx = -1; dx <= 1; dx++) {
                      for(dy = -1; dy <= 1; dy++) {
                          if(goaly+dy >= 0 && goaly+dy < cell_h && goalx+dx >= 0 && goalx+dx < cell_w) {
                              if(max < map[goaly+dy][goalx+dx]) {
                                  max = map[goaly+dy][goalx+dx];
                                  Fx = dx;
                                  Fy = dy;
                              } else if(max == map[goaly+dy][goalx+dx]) {
                                  Fx += dx;
                                  Fy += dy;
                              }
                          }
                      }
                  }

                  if(abs(Fx) > 0)
                      Fx = abs(Fx)/Fx;
                  if(abs(Fy) > 0)
                      Fy = abs(Fy)/Fy;

                  goalx -= Fx;
                  goaly -= Fy;

                  if(counter > 10)
                      break;
              }

              if(counter <= 10) {
                  new_pos.x = (goalx - floor((float) cell_w/4.0))*map_granularity;
                  new_pos.y = (goaly - floor((float) cell_h/4.0))*map_granularity;
              } else {
                  new_pos.x = -1;
                  new_pos.y = -1;
              }
          }

          pos_pub.publish(new_pos);

        }
        
        r.sleep();
        
        
    }
    
    
    
}











