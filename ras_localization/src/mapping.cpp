#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/LaserScan.h"

#include <fstream>
#include <istream>
#include <string>

#include <iostream>

float square_size;
int d_outliers;
float lidar_distance;
int lidar_non_consecutive;
float max_w_velocity;
int outliers_min_cluster;
int counter_for_new_wall;
int counter_for_mapping;

double frequency = 20.0;

bool save = false;


/*
 
 INTERNAL REPRESENTATIONS FOR THE MAP OF THE MAZE
 
 map[cell_h][cell_w] -> EACH NUMBER REPRESENT A SQUARE OF THE MAZE WHOSE DIMENTIONS ARE DEFINED A PRIORI
 
 IN THE MAP THE Y IS THE NUMBER OF ROWS AND X THE NUMBER OF COLOUMNS: map[y][x]
 ALL METHODS USED IN THE CLASS SPECIFY FIRST X AND THEN Y: Map.method(int x, int y)
 
   0: FREE SPACE THE ROBOT CAN GO
 100: WALL
 
*/

class Point {
public:
    int x;
    int y;
    int lidar_n;
};

class WallCell {
public:
    int x;
    int y;
};

class Map {
public:
    
    int** map = NULL;
    bool map_read; // true if the map has been read
    float cell_size;
    int cell_h, cell_w;
    double max_x, max_y;
    
    float lidar_data[360];
    bool lidarReady = false;
    float x_r, y_r, theta_r;
    std::vector<Point> outliers;
    double angular_velocity = 100.0;
    bool newWallReady = false;
    
    std::vector<WallCell> new_walls;

    
    // set the map
    void createMap(float c_size, double x, double y)
    {
        double intpart;

        max_x = x;
        max_y = y;
        
        cell_size = c_size;

        cell_h = (int) round((float) max_y/cell_size);
        cell_w = (int) round((float) max_x/cell_size);

        map = new int*[cell_h];
        for(int i = 0; i < cell_h; i++) {
            map[i] = new int[cell_w];
            for(int j = 0; j < cell_w; j++) {
                map[i][j] = 0;
            }
        }
    }
    
    // add a wall given the two vertices
    void setWall(double px1, double py1, double px2, double py2)
    {
        int x1, x2, y1, y2;
        double distance;
        double x_new, y_new;
        
        x1 = (int) round(px1 / cell_size);
        y1 = (int) round(py1 / cell_size);
        x2 = (int) round(px2 / cell_size);
        y2 = (int) round(py2 / cell_size);

        if(x1 < 0)
	    x1 = 0;
        if(x1 >= cell_w)
            x1 = cell_w-1;
        if(y1 < 0)
	        y1 = 0;
        if(y1 >= cell_h)
            y1 = cell_h-1;
        if(x2 < 0)
	        x2 = 0;
        if(x2 >= cell_w)
            x2 = cell_w-1;
        if(y2 < 0)
	        y2 = 0;
        if(y2 >= cell_h)
            y2 = cell_h-1;
        
        distance = 2 * round(sqrt(pow((x1-x2), 2) + pow((y1-y2), 2)));
        
        if(floor(distance) == 0) {
            map[y1][x1] = 100;
            return;
        }
        
        
        for(int i = 0; i <= distance; i++) {
            x_new = round(x1 + (double) i/distance * (x2 - x1));
            y_new = round(y1 + (double) i/distance * (y2 - y1));

            map[(int) y_new][(int) x_new] = 100;
        }
        
    }
    
    // callback called every time a /maze_map message is received
    void mapCreationCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
        if(!map_read) {
            
            double p[4][4], x_max = 0.0, y_max = 0.0;
            double x_min = std::numeric_limits<double>::infinity();
            double y_min = std::numeric_limits<double>::infinity();
            double x_m, y_m, length, angle;
            double roll, pitch, yaw;

            int N_walls = msg->markers.size();
            
            for(int i = 0; i < 4; i++) {
                
                x_m = msg->markers[i].pose.position.x;
                y_m = msg->markers[i].pose.position.y;
                length = msg->markers[i].scale.x;
                
                if(isinf(x_m) || isinf(y_m) || isinf(length) || length == 0.0) {
                    ROS_INFO("skip marker");
                    return;
                }
                

                tf::Quaternion q2(msg->markers[i].pose.orientation.x, msg->markers[i].pose.orientation.y, msg->markers[i].pose.orientation.z, msg->markers[i].pose.orientation.w);
                tf::Matrix3x3 m(q2);
                m.getRPY(roll, pitch, yaw);
                
                
                p[i][0] = x_m - 0.5 * length * cos(yaw);
                p[i][1] = y_m - 0.5 * length * sin(yaw);
                p[i][2] = x_m + 0.5 * length * cos(yaw);
                p[i][3] = y_m + 0.5 * length * sin(yaw);
                
                x_max = (( std::max(p[i][0],p[i][2]) > x_max) ? std::max(p[i][0],p[i][2]) : x_max );
                x_min = (( std::min(p[i][0],p[i][2]) < x_min) ? std::min(p[i][0],p[i][2]) : x_min );
                y_max = (( std::max(p[i][1],p[i][3]) > y_max) ? std::max(p[i][1],p[i][3]) : y_max );
                y_min = (( std::min(p[i][1],p[i][3]) < y_min) ? std::min(p[i][1],p[i][3]) : y_min );
                
            }
            
            ROS_INFO("%f %f %f %f", x_min, x_max, y_min, y_max);
            
            createMap(square_size, x_max, y_max);

	    
            

            setWall(p[0][0], p[0][1], p[0][2], p[0][3]);
            setWall(p[1][0], p[1][1], p[1][2], p[1][3]);
            setWall(p[2][0], p[2][1], p[2][2], p[2][3]);
            setWall(p[3][0], p[3][1], p[3][2], p[3][3]);
            
            
            
            for(int i = 4; i < N_walls; i++) {
                
                x_m = msg->markers[i].pose.position.x;
                y_m = msg->markers[i].pose.position.y;
                length = msg->markers[i].scale.x;
                
                if(isinf(x_m) || isinf(y_m) || isinf(length) || length == 0.0)
                    continue;
                
                tf::Quaternion q2(msg->markers[i].pose.orientation.x, msg->markers[i].pose.orientation.y, msg->markers[i].pose.orientation.z, msg->markers[i].pose.orientation.w);
                tf::Matrix3x3 m(q2);
                m.getRPY(roll, pitch, yaw);;
                
                
                p[0][0] = x_m - 0.5 * length * cos(yaw);
                p[0][1] = y_m - 0.5 * length * sin(yaw);
                p[0][2] = x_m + 0.5 * length * cos(yaw);
                p[0][3] = y_m + 0.5 * length * sin(yaw);
                
                
                setWall(p[0][0], p[0][1], p[0][2], p[0][3]);
            }
            
            
            map_read = true;
        }
    }
    
    
    void modifyCallback(const std_msgs::Float32::ConstPtr& msg) {
        
        double phi, angle, alpha=0.041;
        int j, i;
        double pred_x, pred_y;
        
        if(!outliers.empty())
            outliers.clear();

       // ROS_INFO("mapping callback called");
        
        
        for(j = 0; j < 360; j++) { // TODO: take into consideration 0.1 seconds delay of LIDAR measurements ????
            if(lidar_data[j] != 0) {
            
                phi = - ((double) j/360.0)*6.2832 - 0.01745;
                angle = theta_r + M_PI/2.0 - phi + alpha;
                angle = fmod(angle, 2.0 * M_PI);
                    
                pred_x = x_r + lidar_data[j] * cos(angle);
                pred_y = y_r + lidar_data[j] * sin(angle);
                    
                pred_x = round(pred_x / cell_size );
                pred_y = round(pred_y / cell_size );
            
                
                if(pred_y >= 0 && pred_y < cell_h && pred_x >= 0 && pred_x < cell_w) {
                    
                    
                    
                    
                    if(!isCloseToWall(pred_x, pred_y) && lidar_data[j] < lidar_distance ) {
                        Point p;
                        p.x = pred_x;
                        p.y = pred_y;
                        p.lidar_n = j;
                        outliers.push_back(p);
                    }
                }
            }
        }

        //ROS_INFO("number of outliers: %d", outliers.size());
        
        if(!outliers.empty()) {
        
            int count = 1;
            int last = outliers.at(0).lidar_n;
            int begin = 0;
            std::vector<Point> points;
            points.push_back(outliers.at(0));
            bool changed = false;
            
            for(int i = 1; i < outliers.size(); i++) {
                if(abs((outliers.at(i).lidar_n % 360) - (last % 360)) < lidar_non_consecutive) {
                    count++;
                    points.push_back(outliers.at(i));
                } else {
                    if(count > outliers_min_cluster && fabs(angular_velocity) < max_w_velocity) {
                        
			//ROS_INFO("number of points to map: %d", points.size());
                        if(!points.empty()) {
                            addNewWall(points);
                            changed = true;
                        }
                    } else {
                        //ROS_INFO("count: %d, angular velocity %f", count, angular_velocity);
                    }
                    
                    count = 1;
                    begin = i;
                    points.clear();
                    points.push_back(outliers.at(i));
                }
                last = outliers.at(i).lidar_n;

                
                
            }

		if(count > outliers_min_cluster && fabs(angular_velocity) < max_w_velocity) {
			if(!points.empty()) {
				addNewWall(points);
				changed = true;
			}
		}
        
        }
        
    
    }
    
    bool isCloseToWall(int x, int y) {
    
        int d = 100 - d_outliers - 1;
    
        for(int i = -d; i <= d; i++) {
            for(int j = -d; j <= d; j++) {
                if((y+i) >= 0 && (y+i) < cell_h && (x+j) >= 0 && (x+j) < cell_w) {
                    if(map[y+i][x+j] == 100 && !newWall(x, y))
                        return true;
                }
            }
        }
        
        return false;
    }
    
    bool newWall(int x, int y) {
        for(int i = 0; i < new_walls.size(); i++) {
            if(x == new_walls.at(i).x && y == new_walls.at(i).y)
                return true;
        }
        
        return false;
    }
    
    void addNewWall(std::vector<Point> points) {

        for(int i = 0; i < points.size(); i++) {
        
            if(map[points.at(i).y][points.at(i).x] < counter_for_mapping) {
                map[points.at(i).y][points.at(i).x]++;
            } else if(map[points.at(i).y][points.at(i).x] == counter_for_mapping) {
                map[points.at(i).y][points.at(i).x] = 100;
                WallCell wall;
                wall.x = points.at(i).x;
                wall.y = points.at(i).y;
                new_walls.push_back(wall);
            }

            if(map[points.at(i).y][points.at(i).x] == counter_for_new_wall) {
                newWallReady = true;
            }

        }
    }
    
    
    void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg) {

        int mCount = 0;
        for(int i = 0; i < 360; i++)
        {
            if(!isinf(msg->ranges[i])) {
                lidar_data[i] = 1.013 * msg->ranges[i];//Magic number to correct measurement
                mCount++;
            } else {
                lidar_data[i] = 0;
            }
        }
        if(mCount > 0)
            lidarReady = true;
    }
    
    void positionCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
        x_r = msg->x;
        y_r = msg->y;
        theta_r = msg->theta;
    }

    void odomCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {

        angular_velocity = msg->theta;
        //ROS_INFO("ang_vel: %f",angular_velocity);
    }
};

/*
 MORE OR LESS SIMILAR TO THE CLASS MAP, IT´S DOUBLE THE SIZE AND EACH CELL´S VALUE IS DIRECTLY PROPORTIONAL TO HOW FAR THE CLOSEST WALL IS
*/

class ExtendedMap {
public:
    
    int** map = NULL;
    float cell_size;
    int cell_h, cell_w;
    double max_x, max_y;
    int origin_x, origin_y, end_x, end_y;
    
    // set the extended map
    ExtendedMap(Map mapBase)
    {

        cell_h = 2*mapBase.cell_h;
        cell_w = 2*mapBase.cell_w;
        max_x = 2*mapBase.max_x;
        max_y = 2*mapBase.max_y;
        cell_size = mapBase.cell_size;
        
        origin_x = (int) floor((float) cell_w/4.0);
        origin_y = (int) floor((float) cell_h/4.0);
        end_x = (int) floor((float) cell_w*3.0/4.0);
        end_y = (int) floor((float) cell_h*3.0/4.0);
        

        map = new int*[cell_h];
        for(int i = 0; i < cell_h; i++) {
            map[i] = new int[cell_w];
            for(int j = 0; j < cell_w; j++) {
                map[i][j] = 0;
            }
        }
        
        for(int i = origin_y; i < end_y; i++) {
            for(int j = origin_x; j < end_x; j++) {
                map[i][j] = mapBase.map[(i-origin_y)][(j-origin_x)];
            }
        }
        
        
        ExtendMap();
    }
    
    void resetMap(Map mapBase) {
    
        for(int i = origin_y; i < end_y; i++) {
            for(int j = origin_x; j < end_x; j++) {
                map[i][j] = mapBase.map[(i-origin_y)][(j-origin_x)];
            }
        }
    }
    
    void ExtendMap() {
    
        for(int i = 0; i < cell_h; i++) {
            for(int j = 0; j < cell_w; j++) {
                int v=0;
                
                for(int a = -1; a <= 1; a++) {
                    for(int b = -1; b <= 1; b++) {
                        if((i+a) >= 0 && (i+a) < cell_h && (j+b) >= 0 && (j+b) < cell_w) {
                            if(v < map[i+a][j+b])
                                v = map[i+a][j+b];
                        }
                    }
                }
                
                if(v > 0)
                    map[i][j] = v-1;
                else
                    map[i][j] = 0;
            }
        }
        
        for(int i = cell_h-1; i >= 0; i--) {
            for(int j = cell_w-1; j >= 0; j--) {
                int v=0;
                
                for(int a = -1; a <= 1; a++) {
                    for(int b = -1; b <= 1; b++) {
                        if((i+a) >= 0 && (i+a) < cell_h && (j+b) >= 0 && (j+b) < cell_w) {
                            if(v < map[i+a][j+b])
                                v = map[i+a][j+b];
                        }
                    }
                }
                
                if(v > 0)
                    map[i][j] = v-1;
                else
                    map[i][j] = 0;
            }
        }
    }
};

void saveCallback(const std_msgs::BoolConstPtr& msg) {
    save = true;
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    nh.getParam("/frequency_mc", frequency);
    nh.getParam("/map/cell_size", square_size);
    nh.getParam("/mapping/d_outliers", d_outliers);
    nh.getParam("/mapping/lidar_distance", lidar_distance);
    nh.getParam("/mapping/lidar_non_consecutive", lidar_non_consecutive);
    nh.getParam("/mapping/max_w_velocity", max_w_velocity);
    nh.getParam("/mapping/outliers_min_cluster", outliers_min_cluster);
    nh.getParam("/mapping/counter_for_new_wall", counter_for_new_wall);
    nh.getParam("/mapping/counter_for_mapping", counter_for_mapping);
    
    Map maze_map;
    maze_map.map_read = false;
    
    
    ros::Rate r(frequency);
    
    
    ros::Subscriber marker_map = n.subscribe("/maze_map", 100, &Map::mapCreationCallback, &maze_map);
    ros::Subscriber modification_sub = n.subscribe("/map/request_modification", 10, &Map::modifyCallback, &maze_map);
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, &Map::lidarCallback, &maze_map);
    ros::Subscriber position_sub = n.subscribe("/robot/position", 10, &Map::positionCallback, &maze_map);
    ros::Subscriber odom_sub = n.subscribe("/robot/position_change", 10, &Map::odomCallback, &maze_map);
    ros::Subscriber save_sub = n.subscribe("/master/save", 1, saveCallback);
  
    ros::Publisher mapBase_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map/base", 10);
    ros::Publisher mapExtended_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map/extended", 10);
    ros::Publisher change_publisher = n.advertise<std_msgs::Bool>("/map/change", 10);
    
    
    
    
    
    std::ifstream map_file("/home/ras28/catkin_ws/src/ras_project/ras_data/base_map.csv", std::ifstream::in);
    
    if(!map_file.fail()) {
    
        
        std::string c_x, c_y;
        std::string val;

        getline(map_file, c_x, ',');
        getline(map_file, c_y, ',');
        
        int map_x, map_y;
        
        map_x = std::atoi(c_x.c_str());
        map_y = std::atoi(c_y.c_str());

        maze_map.max_x = (float) map_x * square_size;
        maze_map.max_y = (float) map_y * square_size;
        
        maze_map.cell_size = square_size;

        maze_map.cell_h = map_y;
        maze_map.cell_w = map_x;

        maze_map.map = new int*[maze_map.cell_h];
        for(int i = 0; i < map_y; ++i){
            maze_map.map[i] = new int[maze_map.cell_w];
            for(int j=0; j < map_x; ++j){
                getline(map_file, val, ',');
                maze_map.map[i][j] = std::atoi( val.c_str());
            }
        }
        
        maze_map.map_read = true;
        
        map_file.close();
        
    } else {
    
        // loop until the base map is ready
        while(n.ok() && !maze_map.map_read){
            
            ros::spinOnce();
            
            r.sleep();
        }
    
    }

	

    
    
    
    
    
    ExtendedMap Emap(maze_map);
    
    
    
    geometry_msgs::Pose pose;
    nav_msgs::OccupancyGrid map_msg;
    int count;
    int grid_mapBase[maze_map.cell_h * maze_map.cell_w];
    int grid_mapExt[Emap.cell_h * Emap.cell_w];
    int i, j;

    std_msgs::Bool notification;
    
    while(n.ok()) {

    
        ros::spinOnce();

        if(maze_map.newWallReady) {
          notification.data = true;
          change_publisher.publish(notification);
          Emap.resetMap(maze_map);
          Emap.ExtendMap();
          maze_map.newWallReady = false;
        }
        
        
        
        if(save) {
        
            std::ofstream outfile("/home/ras28/catkin_ws/src/ras_project/ras_data/base_map.csv");
            
            outfile << maze_map.cell_w << ',';
            outfile << maze_map.cell_h << ',';

            for(int i = 0; i < maze_map.cell_h; ++i){
                for(int j=0; j < maze_map.cell_w; ++j){
                    outfile << maze_map.map[i][j] << ',';
                }
            }

            outfile.close();
        
            save = false;
        
        }
        
        

        
        // publish base map
        
        
        
        map_msg.info.resolution = (maze_map.cell_size);
        map_msg.info.width = maze_map.cell_w;
        map_msg.info.height = maze_map.cell_h;

        pose.position.x = - (float) (maze_map.cell_size)/2.0;
        pose.position.y = - (float) (maze_map.cell_size)/2.0;
        pose.position.z = 0.0;
        map_msg.info.origin = pose;
        
        count = 0;
        for(i = 0; i < maze_map.cell_h; i++) {
            for(j = 0; j < maze_map.cell_w; j++) {
                grid_mapBase[count] = maze_map.map[i][j];
                count++;
            }
        }
            
        std::vector<signed char> v1(grid_mapBase, grid_mapBase + sizeof(grid_mapBase) / sizeof(grid_mapBase[0]) );


        map_msg.data = v1;
        mapBase_publisher.publish(map_msg);
            
            
        // publish extended map
            
        map_msg.info.resolution = (Emap.cell_size);
        map_msg.info.width = Emap.cell_w;
        map_msg.info.height = Emap.cell_h;

        
        pose.position.x = - (float) (Emap.cell_size)/2.0 - Emap.origin_x*Emap.cell_size;
        pose.position.y = - (float) (Emap.cell_size)/2.0 - Emap.origin_y*Emap.cell_size;
        pose.position.z = 0.0;
        map_msg.info.origin = pose;
        
        count = 0;
        for(i = 0; i < Emap.cell_h; i++) {
            for(j = 0; j < Emap.cell_w; j++) {
                grid_mapExt[count] = Emap.map[i][j];
                count++;
            }
        }
            
        std::vector<signed char> v2(grid_mapExt, grid_mapExt + sizeof(grid_mapExt) / sizeof(grid_mapExt[0]) );


        map_msg.data = v2;
        mapExtended_publisher.publish(map_msg);
             
        
    
        r.sleep();
    
    }
    
    
    
}











