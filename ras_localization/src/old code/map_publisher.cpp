#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>

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

class Map {
public:
    
    int** map = NULL;
    bool map_read; // true if the map has been read
    int cell_h, cell_w;
    double max_x, max_y;
    
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
  
    ros::Publisher mapBase_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map/base", 10);
    ros::Publisher mapExtended_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map/extended", 10);
    
    
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
    
    while(n.ok()) {

    
        ros::spinOnce();

        
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











