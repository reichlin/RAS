#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <algorithm>
#include <limits>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <cstdlib>
#include <string>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include <visualization_msgs/MarkerArray.h>


// global variables

int cell_h_E, cell_w_E;
int cell_h, cell_w;
float map_granularity;
int** map = NULL;
int** mapE = NULL;
bool mapRead = false;
bool mapERead = false;
bool changeMap = false;
double frequency = 30.0;
float pos_x, pos_y, pos_theta;

bool localization_ready = false;

float dmax;
float dmin;
float alpha;


float d_p = 0.4;
int blue_weights = 1000;
int max_d_cluster = 20;
int wallCost = 100;
int frontier_threshold;

int invalidThreshold;

class coordinate {
public:
    int x;
    int y;
};

std::vector<coordinate> invalidTargets;


/*

    EXPLORATION NODE:
    
    DEFINES THE EXPLORED MAP
          0: EXPLORED AREA
         50: UNEXPLORED AREA
        100: WALL
        
    INITIALIZE SOME EXPLORATION TARGETS DISTRIBUTED AS UNIFORMELY AS POSSIBLE AND FAR AWAY FROM WALLS
    
    COMPUTES FRONTIERS WHILE THE ROBOT IS MOVING AND SEND TO THE MASTER THESE FRONTIERS AS SOON AS THE OTHER TARGETS ARE COMPLETELY EXPLORED

*/


// read base map callback
void readMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {

    if(changeMap) {
        int c = 0;
        
        
        for(int i = 0; i < cell_h; i++) {
            
            for(int j = 0; j < cell_w; j++) {
                if(msg->data[c] > 99)
                    map[i][j] = 100;
                
                c++;
            }
        }
        
        changeMap = false;
        return;
    }

    if(!mapRead) {
    
    
        cell_h = msg->info.height;
        cell_w = msg->info.width;
        map_granularity = msg->info.resolution;
        
        int c = 0;
        
        map = new int*[cell_h];
        for(int i = 0; i < cell_h; i++) {
            map[i] = new int[cell_w];
            for(int j = 0; j < cell_w; j++) {
                if(msg->data[c] > 99)
                    map[i][j] = 100;
                else
                    map[i][j] = 50;
                c++;
            }
        }
        
        mapRead = true;
    }
    
    
}

// read extended map callback
void readMapECallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {

    if(!mapERead) {
    
    
        cell_h_E = msg->info.height;
        cell_w_E = msg->info.width;
        map_granularity = msg->info.resolution;
        
        int c = 0;
        
        mapE = new int*[cell_h_E];
        for(int i = 0; i < cell_h_E; i++) {
            mapE[i] = new int[cell_w_E];
            for(int j = 0; j < cell_w_E; j++) {
                mapE[i][j] = msg->data[c];
                c++;
            }
        }
        
        mapERead = true;
        
    }
    
    
}


void changeMapCallback(const std_msgs::Bool::ConstPtr &msg) {
    //mapRead = false;
    mapERead = false;
    changeMap = true;
}

// read robot position and update unexplored map
void localizationCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {

    float distance;
    int dx, dy, x, y, x0, y0;
    
    localization_ready = true;

    pos_x = msg->x;
    pos_y = msg->y;
    pos_theta = msg->theta;
    
    
    // update unexplored map
    
    if(round(pos_y/map_granularity) < 0 || round(pos_y/map_granularity) >= cell_h || round(pos_x/map_granularity) < 0 || round(pos_x/map_granularity) >= cell_w)
        return;
        
    /*
    for(int i = 0; i < dmin; i++) {
    
        x0 = round(pos_x/map_granularity) + i * cos(pos_theta);
        y0 = round(pos_y/map_granularity) + i * sin(pos_theta);
        
        if(round(y0) >= 0 && round(y0) < cell_h && round(x0) >= 0 && round(x0) < cell_w) {
            if(map[(int) round(y0)][(int) round(x0)] == 100)
                return;
        }
    }
    */

    for(int i = -5; i <= 5; i++) {
      for(int j = -5; j <= 5; j++) {

        x = round(pos_x/map_granularity) + i;
        y = round(pos_y/map_granularity) + j;

        if(round(y) >= 0 && round(y) < cell_h && round(x) >= 0 && round(x) < cell_w) {
          if(map[(int) round(y)][(int) round(x)] < 100)
              map[(int) round(y)][(int) round(x)] = 0;
        }
      }
    }
    
    for(int i = 0; i < dmax+1; i++) {
        
        
        x0 = round(pos_x/map_granularity) + i * cos(pos_theta);
        y0 = round(pos_y/map_granularity) + i * sin(pos_theta);
        
        if(round(y0) >= 0 && round(y0) < cell_h && round(x0) >= 0 && round(x0) < cell_w) {
            if(map[(int) round(y0)][(int) round(x0)] == 100)
                return;
        }
        
        
        distance = round((2.0 * i * sin(alpha)) / cos(alpha));
        
        for(int j = 0; j < (2*distance); j++) {
            dx =   (j/4.0) * sin(pos_theta);
            dy = - (j/4.0) * cos(pos_theta);
            
            x = round(pos_x/map_granularity) + i * cos(pos_theta) + dx;
            y = round(pos_y/map_granularity) + i * sin(pos_theta) + dy;

            if(round(y) >= 0 && round(y) < cell_h && round(x) >= 0 && round(x) < cell_w) {
                if(map[(int) round(y)][(int) round(x)] < 100)
                    map[(int) round(y)][(int) round(x)] = 0;
                else
                    j = (2*distance+1);
            }
        }
        
        for(int j = 0; j < (2*distance); j++) {
            dx = - (j/4.0) * sin(pos_theta);
            dy =   (j/4.0) * cos(pos_theta);
            
            x = round(pos_x/map_granularity) + i * cos(pos_theta) + dx;
            y = round(pos_y/map_granularity) + i * sin(pos_theta) + dy;

            if(round(y) >= 0 && round(y) < cell_h && round(x) >= 0 && round(x) < cell_w) {
                if(map[(int) round(y)][(int) round(x)] < 100)
                    map[(int) round(y)][(int) round(x)] = 0;
                else
                    j = (2*distance+1);
            }
        }
    }
    
}

class Point {
public:
    int x;
    int y;
    int weight;
};

// distribute uniformely target points on the map and let them slip away from walls
std::vector<Point> setTargets() {
    int Fx, Fy, max, dx, dy, x_meas, y_meas;
    std::vector<Point> nodes;
    int count;
    
    int N_p_x = round((float) (cell_w*map_granularity) / d_p);
    int N_p_y = round((float) (cell_h*map_granularity) / d_p);
    
    
    for(int i = 1; i < N_p_y; i++) {
        for(int j = 1; j < N_p_x; j++) {
        
            Point p;
            p.x = round((float) (d_p/map_granularity) * (j));
            p.y = round((float) (d_p/map_granularity) * (i));
            p.weight = blue_weights;
            
            count = 0;
            
            
            while(count < 50) {
            
                Fx=0;
                Fy=0;
                max=0;

                for(dx = -1; dx <= 1; dx++) {
                    for(dy = -1; dy <= 1; dy++) {
                        x_meas = (int) p.x+dx+round((float) cell_w_E/4.0);
                        y_meas = (int) p.y+dy+round((float) cell_h_E/4.0);
                        if(y_meas >= 0 && y_meas < cell_h_E && x_meas >= 0 && x_meas < cell_w_E) {
                        
                            if(max < mapE[y_meas][x_meas]) {
                                max = mapE[y_meas][x_meas];
                                Fx = dx;
                                Fy = dy;
                            } else if(max == mapE[y_meas][x_meas]) {
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
                    
                p.x -= Fx;
                p.y -= Fy;
                
                count++;
                
            
            }
            
            if(mapE[p.y+(int)round((float) cell_h_E/4.0)][p.x+(int)round((float) cell_w_E/4.0)] < 96)
                nodes.push_back(p);
        }
    }
    
    
    return nodes;
}


// cluster target points together
std::vector<Point> cleanTargets(std::vector<Point> targets) {
    
    std::vector<Point> new_targets;
    
    for(int i = 0; i < targets.size(); i++) {
        new_targets.push_back(targets.at(i));
        
        for(int k = 0; k < new_targets.size(); k++) {
            for(int j = i+1; j < targets.size(); j++) {
                if(abs(targets.at(j).x - new_targets.at(k).x) < max_d_cluster && abs(targets.at(j).y - new_targets.at(k).y) < max_d_cluster) {
                    targets.erase(targets.begin()+j);
                    j--;
                }
            }
        }
    }
    
    return new_targets;
}

// if there is a wall in between two points the distance should be considered higher
int wallInBetween(int x1, int y1, int x2, int y2) {
    
    int cx, cy;
    double distance = 2 * round(sqrt(pow((x1-x2), 2) + pow((y1-y2), 2)));
        
    for(int i = 0; i <= distance; i++) {
        cx = round(x1 + (double) i/distance * (x2 - x1));
        cy = round(y1 + (double) i/distance * (y2 - y1));

        if(map[cy][cx] == 100)
            return 1;
    }
    
    return 0;
}

// sort target points so to have the closest to the robot position first
std::vector<Point> sortTargets(std::vector<Point> targets) {
    
    std::vector<Point> new_targets;
    Point temp;
    int threshold = 10, index;
    bool found_start = false;
    float d;
    
    int robot_x, robot_y;
    
    robot_x = round(pos_x/map_granularity);
    robot_y = round(pos_y/map_granularity);
    
    for(int i = 0; i < targets.size(); i++) {
        if(abs(robot_y-targets.at(i).y) < threshold && abs(robot_x-targets.at(i).x) < threshold) {
            found_start = true;
            index = i;
        }
    }
    
    if(found_start) {
        targets.erase(targets.begin()+index);
    }
    
    while(targets.size() > 0) {
        temp.x = targets.at(0).x;
        temp.y = targets.at(0).y;
        d = sqrt(pow(temp.x-robot_x,2)+pow(temp.y-robot_y,2)) + wallInBetween(robot_x, robot_y, temp.x, temp.y) * wallCost;
        index = 0;
        for(int i = 1; i < targets.size(); i++) {
            if((sqrt(pow(targets.at(i).x-robot_x,2) + pow(targets.at(i).y-robot_y,2)) + wallInBetween(robot_x, robot_y, targets.at(i).x, targets.at(i).y) * wallCost) < d) {
            
                d = (sqrt(pow(targets.at(i).x-robot_x,2) + pow(targets.at(i).y-robot_y,2)) + wallInBetween(robot_x, robot_y, targets.at(i).x, targets.at(i).y) * wallCost);
                temp.x = targets.at(i).x;
                temp.y = targets.at(i).y;
                temp.weight = targets.at(i).weight;
                index = i;
                
            }
        }
        new_targets.push_back(temp);
        targets.erase(targets.begin()+index);
    }
    
    return new_targets;
}

// set new frontiers
std::vector<Point> frontiers() {

    std::vector<Point> frontiers;
    std::vector<Point> new_targets;
    Point temp;
    temp.x = 0;
    temp.y = 0;
    
    for(int i = 0; i < cell_h; i++) {
        for(int j = 0; j < cell_w; j++) {
            
            if(map[i][j] == 0) {
                for(int a = -1; a < 2; a++) {
                    for(int b = -1; b < 2; b++) {
                        if((i+a) >= 0 && (i+a) < cell_h && (j+b) >= 0 && (j+b) < cell_w) {
                            if(map[i+a][j+b] == 50) {
                                Point p;
                                p.x = j;
                                p.y = i;
                                frontiers.push_back(p);
                                a = 2;
                                b = 2;
                            }
                        }
                    }
                }
            }
            
        }
    }
    
    bool stable;
    std::vector<Point> cluster;
    
    for(int i = 0; i < frontiers.size(); i++) {
        Point target;
        target.x = frontiers.at(i).x;
        target.y = frontiers.at(i).y;
        cluster.push_back(target);
        
        stable = false;
        
        while(!stable) {
            stable = true;
            for(int j = (i+1); j < frontiers.size(); j++) {
                
                for(int k = 0; k < cluster.size(); k++) {
                    
                    if(abs(frontiers.at(j).x - cluster.at(k).x) < 2 && abs(frontiers.at(j).y - cluster.at(k).y) < 2) {
                        Point point;
                        point.x = frontiers.at(j).x;
                        point.y = frontiers.at(j).y;
                        cluster.push_back(point);
                        target.x += point.x;
                        target.y += point.y;
                        frontiers.erase(frontiers.begin()+j);
                        j--;
                        k = cluster.size();
                        stable = false;
                    }
                }
            }
        }
        
        // TODO: cluster in a smarter way
        
        
        
        target.x = round(target.x / cluster.size());
        target.y = round(target.y / cluster.size());
        target.weight = cluster.size();
        new_targets.push_back(target);
        cluster.clear();
        
        
    }
    
    
    
    
    return new_targets;
}


std::vector<Point> deleteFrontiers(std::vector<Point> frontier_nodes) {

    std::vector<Point> new_frontiers;
    
    int robot_x = round(pos_x/map_granularity);
    int robot_y = round(pos_y/map_granularity);
    
    for(int i = 0; i < frontier_nodes.size(); i++) {
        
        if(frontier_nodes.at(i).weight >= frontier_threshold) {
        
            if(!(abs(frontier_nodes.at(i).x - robot_x) < 5 && abs(frontier_nodes.at(i).y - robot_y) < 5))
                new_frontiers.push_back(frontier_nodes.at(i));
        }
    }
    
    return new_frontiers;

}

void invalidTargetCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {

    coordinate next_invalid;
    
    next_invalid.x = round(msg->x / map_granularity);
    next_invalid.y = round(msg->y / map_granularity);
    
    invalidTargets.push_back(next_invalid);

}


int main(int argc, char** argv) {

    std::vector<Point> nodes;
    std::vector<Point> frontier_nodes;
    
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray marker_array2;
    
    
    ros::init(argc, argv, "explorer");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    nh.getParam("/frequency_mc", frequency);
    nh.getParam("/camera/dmax", dmax);
    nh.getParam("/camera/dmin", dmin);
    nh.getParam("/camera/alpha", alpha);
    
    nh.getParam("/exploration/d_p", d_p);
    nh.getParam("/exploration/weights", blue_weights);
    nh.getParam("/exploration/cluster", max_d_cluster);
    nh.getParam("/exploration/wallCost", wallCost);
    nh.getParam("/exploration/frontier_threshold", frontier_threshold);
    nh.getParam("/exploration/invalidThreshold", invalidThreshold);
    
    
    
    ros::Rate r(frequency);
        
    ros::Subscriber sub_map = n.subscribe("/map/base", 1, readMapCallback);
    ros::Subscriber sub_mapE = n.subscribe("/map/extended", 1, readMapECallback);
    ros::Subscriber sub_odom = n.subscribe("/robot/position", 1, localizationCallback);
    ros::Subscriber sub_map_change = n.subscribe("/map/change", 10, changeMapCallback);
    ros::Subscriber sub_invalid_target = n.subscribe("/invalid_goal", 1, invalidTargetCallback);
    

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map/unexplored", 1);
    ros::Publisher next_action = n.advertise<geometry_msgs::Pose>("/master/next_search_node", 1);
    
    ros::Publisher targets_pub = n.advertise<visualization_msgs::MarkerArray>("nodes", 1);
    ros::Publisher front_pub = n.advertise<visualization_msgs::MarkerArray>("frontiers", 1);
        
    geometry_msgs::Pose2D robot_pose;
    std_msgs::Float32 request;
    geometry_msgs::Pose next_node;
    
    
        
    // loop until maps are read
    while(n.ok() && ((!mapRead || !mapERead) || !localization_ready)) {
        
        ros::spinOnce();
        
        
        
        r.sleep();
    }
    
    
    dmax = round(dmax/map_granularity);
    dmin = round(dmin/map_granularity);
    
    
    // generate nodes and put them in a list

    nodes = setTargets();
    nodes = cleanTargets(nodes);
    
    nodes = sortTargets(nodes);

    /*
    Point p;
    p.x = 35;
    p.y = 15;
    nodes.push_back(p);
    */

    int x, y;


    for(int i = -12; i <= 12; i++) {
      for(int j = -12; j <= 12; j++) {

        x = round(pos_x/map_granularity) + i;
        y = round(pos_y/map_granularity) + j;

        if(round(y) >= 0 && round(y) < cell_h && round(x) >= 0 && round(x) < cell_w) {
          if(map[(int) round(y)][(int) round(x)] < 100)
              map[(int) round(y)][(int) round(x)] = 0;
        }
      }
    }
    
    
    
    geometry_msgs::Pose pose;
    nav_msgs::OccupancyGrid map_msg;
    int count;
    int grid_mapBase[cell_h * cell_w];
    int grid_mapExt[cell_h * cell_w];
    int i, j;
    
    
    
    while(n.ok()) {
    
        
        ros::spinOnce();


        
        
        
        // check if some of the nodes lie in an already explored part of the map
        // if yes remove them
        if(!nodes.empty()) {
          for(int a = 0; a < nodes.size(); a++) {
            if(map[nodes.at(a).y][nodes.at(a).x] == 0) {
              nodes.erase(nodes.begin()+a);
              a--;
            }
          }
          
          
          if(!invalidTargets.empty()) {
            for(int a = 0; a < invalidTargets.size(); a++) {
                if(!nodes.empty()) {
                
                    if(abs(nodes.at(0).x - invalidTargets.at(a).x) < invalidThreshold && abs(nodes.at(0).y - invalidTargets.at(a).y) < invalidThreshold) {
                        nodes.erase(nodes.begin());
                        //invalidTargets.erase(invalidTargets.begin()+a);
                        //break;
                        a = -1;
                    }
                } else {
                    break;
                }
              }
          }
        }


        if(nodes.empty()) {
        
            // start sending to the master the frontiers if they have a value grater than threshold
            frontier_nodes = frontiers();

            frontier_nodes = deleteFrontiers(frontier_nodes);
            
            if(!frontier_nodes.empty()) {
            
                
                for(int a = 0; a < invalidTargets.size(); a++) {
                    if(!invalidTargets.empty()) {
                        if(abs(frontier_nodes.at(0).x - invalidTargets.at(a).x) < invalidThreshold && abs(frontier_nodes.at(0).y - invalidTargets.at(a).y) < invalidThreshold) {
                            frontier_nodes.erase(frontier_nodes.begin());
                            a = -1;
                        }
                    } else {
                        break;
                    }
                }
            
            }
            
            if(!frontier_nodes.empty()) {
            
                frontier_nodes = sortTargets(frontier_nodes);
            
                next_node.position.x = (float) frontier_nodes.at(0).x*map_granularity;
                next_node.position.y = (float) frontier_nodes.at(0).y*map_granularity;
                next_node.position.z = 0.0;
                next_action.publish(next_node);
                
            } else {
            
                // tell the master the exploration is done
                next_node.position.z = -1.0;
                next_action.publish(next_node);
                //return 0;
            }
            
        } else {
        
            nodes = sortTargets(nodes);

            //ROS_INFO("my position: %f, %f", pos_x, pos_y);


            // send to the master next node
            next_node.position.x = (float) nodes.at(0).x*map_granularity;
            next_node.position.y = (float) nodes.at(0).y*map_granularity;
            next_node.position.z = 0.0;
            next_action.publish(next_node);

            
            // FIND NEW NODES BASED ON FRONTIERS
            frontier_nodes = frontiers();
            
            
            
            // PRINT MARKERS
            // ---------------------------------------------------------------------------------------------------
            
            marker_array.markers.resize(nodes.size());

        
            for(int a = 0; a < nodes.size(); a++) {
                //ROS_INFO("target %d -> %d, %d", a, nodes.at(a).y, nodes.at(a).x);
                //map[nodes.at(a).y][nodes.at(a).x] = 1;
                
                visualization_msgs::Marker marker;
                marker.header.frame_id = "/map";
                marker.header.stamp = ros::Time();
                marker.ns = "node";
                marker.id = a;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::MODIFY;
                marker.pose.position.x = nodes.at(a).x*map_granularity;
                marker.pose.position.y = nodes.at(a).y*map_granularity;
                marker.pose.position.z = 0.05;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = ((float) a/nodes.size());
                marker.color.g = 0.0;
                marker.color.b = 1.0 - ((float) a/nodes.size());
                
                marker_array.markers[a] = marker;
            }
            
            targets_pub.publish( marker_array );
        
        }
        
        if(!frontier_nodes.empty()) {
          marker_array2.markers.resize(frontier_nodes.size());


          for(i = 0; i < frontier_nodes.size(); i++) {
              visualization_msgs::Marker marker;
              marker.header.frame_id = "/map";
              marker.header.stamp = ros::Time();
              marker.ns = "frontier";
              marker.id = i;
              marker.type = visualization_msgs::Marker::SPHERE;
              marker.action = visualization_msgs::Marker::MODIFY;
              marker.pose.position.x = frontier_nodes.at(i).x*map_granularity;
              marker.pose.position.y = frontier_nodes.at(i).y*map_granularity;
              marker.pose.position.z = 0.05;
              marker.pose.orientation.x = 0.0;
              marker.pose.orientation.y = 0.0;
              marker.pose.orientation.z = 0.0;
              marker.pose.orientation.w = 1.0;
              marker.scale.x = 0.05 * ((float) frontier_nodes.at(i).weight / 100);
              marker.scale.y = 0.05 * ((float) frontier_nodes.at(i).weight / 100);
              marker.scale.z = 0.05 * ((float) frontier_nodes.at(i).weight / 100);
              marker.color.a = 1.0; // Don't forget to set the alpha!
              marker.color.r = 0.0;
              marker.color.g = 1.0;
              marker.color.b = 0.0;
              marker_array2.markers[i] = marker;

              //map[frontier_nodes.at(i).y][frontier_nodes.at(i).x] = 100;
          }


          front_pub.publish( marker_array2 );
        }
        
        // ---------------------------------------------------------------------------------------------------
        
        
        
        // publish unexplored map
        
        map_msg.info.resolution = map_granularity;
        map_msg.info.width = cell_w;
        map_msg.info.height = cell_h;

        pose.position.x = -map_granularity/2.0;
        pose.position.y = -map_granularity/2.0;
        pose.position.z = 0.0;
        map_msg.info.origin = pose;
        
        count = 0;
        for(i = 0; i < cell_h; i++) {
            for(j = 0; j < cell_w; j++) {
                grid_mapBase[count] = map[i][j];
                count++;
            }
        }
            
        std::vector<signed char> v1(grid_mapBase, grid_mapBase + sizeof(grid_mapBase) / sizeof(grid_mapBase[0]) );


        map_msg.data = v1;
        map_pub.publish(map_msg);
        
                
        r.sleep();
    }
    
    
    
}










