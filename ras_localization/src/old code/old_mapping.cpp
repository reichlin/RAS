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

int cell_x = 128;
int cell_y = 128;

double frequency = 30.0;


/*
 
 INTERNAL REPRESENTATIONS FOR THE MAP OF THE MAZE
 
 map[cell_h][cell_w] -> EACH NUMBER REPRESENT A SQUARE OF THE MAZE WHOSE DIMENTIONS ARE SPECIFIED BY THE RATIO max_x / cell_w
 
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

class Map {
public:
    
    int** map = NULL;
    bool map_read; // true if the map has been read
    int cell_h, cell_w;
    double max_x, max_y;
    
    float lidar_data[360];
    bool lidarReady = false;
    float x_r, y_r, theta_r;
    std::vector<Point> outliers;
    double angular_velocity = 100.0;
    bool newWallReady = false;

    
    // set the map
    void createMap(int N_x, int N_y, double x, double y)
    {
        double intpart;

        max_x = x;
        max_y = y;

        cell_h = N_y;
        cell_w = N_x;

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
        
        x1 = (int) round((cell_w * px1) / max_x);
        y1 = (int) round((cell_h * py1) / max_y);
        x2 = (int) round((cell_w * px2) / max_x);
        y2 = (int) round((cell_h * py2) / max_y);

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
            
            createMap(cell_x, cell_y, x_max, y_max);

	    
            

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
        
        
        for(j = 0; j < 360; j++) {
            if(lidar_data[j] != 0) {
            
                phi = - ((double) j/360.0)*6.2832 - 0.01745;
                angle = theta_r + M_PI/2.0 - phi + alpha;
                angle = fmod(angle, 2.0 * M_PI);
                    
                pred_x = x_r + lidar_data[j] * cos(angle);
                pred_y = y_r + lidar_data[j] * sin(angle);
                    
                pred_x = round(pred_x / (max_x / cell_w) );
                pred_y = round(pred_y / (max_x / cell_w) );
            
                //x_meas = pred_x+(cell_w/4);
                //y_meas = pred_y+(cell_h/4);
                
                if(pred_y >= 0 && pred_y < cell_h && pred_x >= 0 && pred_x < cell_w) {
                    
                    if(map[(int) pred_y][(int) pred_x] < 98 && lidar_data[j] < 0.7 ) {
                        Point p;
                        p.x = pred_x;
                        p.y = pred_y;
                        p.lidar_n = j;
                        outliers.push_back(p);
                    }
                }
            }
        }
        
        if(!outliers.empty()) {
        
            int count = 0;
            int last = outliers.at(0).lidar_n;
            int begin = 0;
            std::vector<Point> points;
            points.push_back(outliers.at(0));
            bool changed = false;
            
            for(int i = 1; i < outliers.size(); i++) {
                if(abs((outliers.at(i).lidar_n % 360) - (last % 360)) < 3) {
                    count++;
                    points.push_back(outliers.at(i));
                } else {
                    if(count > 5 && fabs(angular_velocity) < 0.002) {
                        //ransac(points);
                        if(!points.empty()) {
                            addNewWall(points);
                            changed = true;
                        }
                    }
                   // ROS_INFO("count: %d", count);
                    count = 0;
                    begin = i;
                    points.clear();
                }
                last = outliers.at(i).lidar_n;

                
                
            }
        
        }
        
    
    }
    
    void addNewWall(std::vector<Point> points) {

      for(int i = 0; i < points.size(); i++) {
        if(map[points.at(i).y][points.at(i).x] < 10) {
          map[points.at(i).y][points.at(i).x]++;
        }
        else if(map[points.at(i).y][points.at(i).x] == 10)
          map[points.at(i).y][points.at(i).x] = 100;

        if(map[points.at(i).y][points.at(i).x] == 5) {
          newWallReady = true;
        }

      }
    }

    /*
    void ransac(std::vector<Point> points) {
        int index1, index2;
        float m, q, m_best, q_best;
        int votes = 0, best_votes = 0;
        float error = 0.0, smaller_error = 9999999.0;
        int K = 1000;
        int predicted;
        float threshold = 1.0;
        
        for(int k = 0; k < K; k++) {
            index1 = rand() % points.size();
            index2 = rand() % points.size();
            while(index == index1)
                index2 = rand() % points.size();
                
            m = (float) (points.at(index2).y - points.at(index1).y) / (points.at(index2).x - points.at(index1).x);
            q = (float) points.at(index1).y - m * points.at(index1).x;
            
            for(int i = 0; i < points.size(); i++) {
                predicted = (int) m * points.at(i).x + q;
                error+= abs(points.at(i).y - predicted);
                if(abs(points.at(i).y - predicted) < threshold)
                    votes++;
            }
            
            if(votes > best_votes) {
                if(error < smaller_error) {
                    smaller_error = error;
                    best_votes = votes;
                    m_best = m;
                    q_best = q;
                }
            }
            
        }
        
        
    }
    */
    
    
    void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg) {

        int mCount = 0;
        for(int i = 0; i < 360; i++)
        {
            if(!isinf(msg->ranges[i])) {
                lidar_data[i] = msg->ranges[i];
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
    int cell_h, cell_w;
    double max_x, max_y;
    
    // set the extended map
    ExtendedMap(Map mapBase)
    {

        cell_h = 2*mapBase.cell_h;
        cell_w = 2*mapBase.cell_w;
        max_x = 2*mapBase.max_x;
        max_y = 2*mapBase.max_y;

        map = new int*[cell_h];
        for(int i = 0; i < cell_h; i++) {
            map[i] = new int[cell_w];
            for(int j = 0; j < cell_w; j++) {
                map[i][j] = 0;
            }
        }
        
        for(int i = (cell_h/4); i < (3*cell_h/4); i++) {
            for(int j = (cell_w/4); j < (3*cell_w/4); j++) {
                map[i][j] = mapBase.map[(i-cell_h/4)][(j-cell_w/4)];
            }
        }
        
        
        ExtendMap();
    }
    
    void resetMap(Map mapBase) {
    
        for(int i = (cell_h/4); i < (3*cell_h/4); i++) {
            for(int j = (cell_w/4); j < (3*cell_w/4); j++) {
                map[i][j] = mapBase.map[(i-cell_h/4)][(j-cell_w/4)];
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


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    nh.getParam("/frequency_mc", frequency);
    
    Map maze_map;
    maze_map.map_read = false;
    
    
    ros::Rate r(frequency);
    
    
    ros::Subscriber marker_map = n.subscribe("/maze_map", 100, &Map::mapCreationCallback, &maze_map);
    ros::Subscriber modification_sub = n.subscribe("/map/request_modification", 10, &Map::modifyCallback, &maze_map);
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, &Map::lidarCallback, &maze_map);
    ros::Subscriber position_sub = n.subscribe("/robot/position", 10, &Map::positionCallback, &maze_map);
    ros::Subscriber odom_sub = n.subscribe("/robot/position_change", 10, &Map::odomCallback, &maze_map);
  
    ros::Publisher mapBase_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map/base", 10);
    ros::Publisher mapExtended_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map/extended", 10);
    ros::Publisher change_publisher = n.advertise<std_msgs::Bool>("/map/change", 10);
    
    
    // loop until the base map is ready
    while(n.ok() && !maze_map.map_read){
        
        ros::spinOnce();
        
        r.sleep();
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
        
        

        
        // publish base map
        
        
        
        map_msg.info.resolution = (maze_map.max_x / maze_map.cell_w);
        map_msg.info.width = maze_map.cell_w;
        map_msg.info.height = maze_map.cell_h;

        pose.position.x = -(maze_map.max_x / maze_map.cell_w)/2.0;
        pose.position.y = -(maze_map.max_y / maze_map.cell_h)/2.0;
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
            
        map_msg.info.resolution = (Emap.max_x / Emap.cell_w);
        map_msg.info.width = Emap.cell_w;
        map_msg.info.height = Emap.cell_h;

        
        pose.position.x = -(Emap.max_x / Emap.cell_w)/2.0 - (Emap.cell_w/4.0)*(Emap.max_x / Emap.cell_w);
        pose.position.y = -(Emap.max_y / Emap.cell_h)/2.0 - (Emap.cell_h/4.0)*(Emap.max_y / Emap.cell_h);
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











