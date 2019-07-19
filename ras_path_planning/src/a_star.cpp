#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <set>
#include <vector>
#include <algorithm>
#include <std_msgs/Int8.h>
#include <queue>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <istream>
#include <string>

int size_x, size_y, start_x, start_y, goal_x, goal_y, exp_rad, smooth_rad;
float rad, resolution, rec;
bool map_created = false;
bool have_start = false;
bool have_goal = false;
bool read_map = false;
bool save = false;
float cause = 0.0; // Cause for not finding a path
int** map_for_planning = NULL;
int** map_for_smooth = NULL;

struct GridLocation{
  int x, y;
};

struct Point{
  int x, y;
};

std::vector<Point> batteries;
std::vector<Point> objects;

struct SquareGrid{
public:
  static std::array<GridLocation, 8>  DIRS;
  int width, height;
  std::set<GridLocation> walls;

  SquareGrid(int width_, int height_) : width(width_), height(height_){}
  bool in_bounds(GridLocation id) const{
    return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
  }

  bool passable(GridLocation id) const{
    return walls.find(id) == walls.end();
  }

  std::vector<GridLocation> neighbors (GridLocation id) const {
    std::vector<GridLocation> results;

    for (GridLocation dir : DIRS){
      GridLocation next{id.x + dir.x, id.y + dir.y};
      if (in_bounds(next) && (passable(next))) {
        results.push_back(next);
      }
    }

    if ((id.x + id.y) %2 == 0 ){
      std::reverse(results.begin(), results.end());
    }
    return results;
  }

};
// Array containing in what directions we can find neighbours to a cell
std::array<GridLocation, 8> SquareGrid::DIRS =
{
    GridLocation{1, 0},
    GridLocation{-1, 0},
    GridLocation{0, 1},
    GridLocation{0, -1},
    GridLocation{1, 1},
    GridLocation{-1, 1},
    GridLocation{1, -1},
    GridLocation{-1, -1}
};

// Adding walls as obstacles to the list of obstacles
SquareGrid addObstacle(int radius, int** map){
  SquareGrid grid(size_y, size_x);
  //ROS_INFO("Grid created");
  int col, row;
  for (col = 0; col < size_y; col++){
    for (row = 0; row < size_x; row++){
      if (map[col][row] == 100){
        grid.walls.insert(GridLocation{col, row});

        for(int r_col = radius; r_col > -1; r_col--){
          if (col - r_col >= 0){
            for ( int r_row = radius; r_row > -1; r_row--){
              if(pow(r_col,2.0)+pow(r_row,2.0) <= pow(radius, 2.0)){
                if(grid.walls.find(GridLocation{col - r_col,row - r_row}) == grid.walls.end()){
                  grid.walls.insert(GridLocation{col - r_col,row - r_row});
                }
                if(grid.walls.find(GridLocation{col - r_col, row + r_row}) == grid.walls.end()){
                  grid.walls.insert(GridLocation{col - r_col, row + r_row});
                }
              }
            }
          }
        }

        for(int r_col = radius; r_col > -1; r_col--){
          if (col + r_col < size_y){
            for (int r_row = radius; r_row > -1; r_row--){
              if(pow(r_col, 2.0)+pow(r_row, 2.0) <= pow(radius, 2.0)){
                if(grid.walls.find(GridLocation{col + r_col, row + r_row}) == grid.walls.end()){
                  grid.walls.insert(GridLocation{col + r_col, row + r_row});
                }
                if(grid.walls.find(GridLocation{col + r_col, row - r_row}) == grid.walls.end()){
                  grid.walls.insert(GridLocation{col + r_col, row - r_row});
                }
              }
            }
          }
        }
      }
    }
  }
  return grid;
}
// Adding batteries to the list of obstacles
SquareGrid addBatteries(SquareGrid grid, int radius){
  int col, row;
  //ROS_INFO("Adding Batteries");
  for (int i = 0; i < batteries.size(); i++){
      col = batteries[i].x;
      row = batteries[i].y;
      //ROS_INFO("battery at: %d, %d" , col, row);
      grid.walls.insert(GridLocation{col, row});

      for(int r_col = radius; r_col > -1; r_col--){
        if (col - r_col >= 0){
          for ( int r_row = radius; r_row > -1; r_row--){
            if(pow(r_col,2.0)+pow(r_row,2.0) <= pow(radius, 2.0)){
              if(grid.walls.find(GridLocation{col - r_col,row - r_row}) == grid.walls.end()){
                grid.walls.insert(GridLocation{col - r_col, row - r_row});
              }
              if(grid.walls.find(GridLocation{col - r_col, row + r_row}) == grid.walls.end()){
                grid.walls.insert(GridLocation{col - r_col, row + r_row});
              }
            }
          }
        }
      }
      for(int r_col = radius; r_col > -1; r_col--){
        if (col + r_col < size_y){
          for (int r_row = radius; r_row > -1; r_row--){
            if(pow(r_col, 2.0)+pow(r_row, 2.0) <= pow(radius, 2.0)){
              if(grid.walls.find(GridLocation{col + r_col, row + r_row}) == grid.walls.end()){
                grid.walls.insert(GridLocation{col + r_col, row + r_row});
              }
              if(grid.walls.find(GridLocation{col + r_col, row - r_row}) == grid.walls.end()){
                grid.walls.insert(GridLocation{col + r_col, row - r_row});
              }
            }
          }
        }
      }
    }

  return grid;
}

// Weighing cells in the 2D array map after the distance to walls/batteries also adding objects as weights
void map_weighing(int** map, SquareGrid grid){
  //ROS_INFO("Weighing Map");
  int col, row;
  for (int i = 0; i < size_y; i++){
    for (int j = 0; j < size_x; j++){
      if ((grid.walls.find(GridLocation{i, j}) != grid.walls.end())){
        for (int k = -2; k < 3; k++){
          for (int l = -2; l < 3; l++){
            if (i+k >= 0 && i+k < size_y && j+l >=0 && j+l < size_x){
              if (std::abs(k) < 2 && std::abs(l) < 2){
                if(map[i+k][j+l] < 7){
                  map[i+k][j+l] = 7;
                }
              }
              else{
                if(map[i+k][j+l] < 3){
                  map[i+k][j+l] = 3;
                }
              }
            }
          }
        }
      }
    }
  }

  for(int i = 0; i < objects.size(); i++){
    col = objects[i].x;
    row = objects[i].y;
    for (int k = -9; k < 10; k++){
      for (int l = -9; l < 10; l++){
        if (col+k >= 0 && col+k < size_y && row+l >=0 && row+l < size_x){
          if(pow(k,2.0)+pow(l,2.0) <= pow(9, 2.0)){
            if(pow(k,2.0)+pow(l,2.0) <= pow(5, 2.0)){
              if(map[col+k][row+l] < 7){
                map[col+k][row+l] = 7;
              }
            }
            else{
              if(map[col+k][row+l] < 3){
                map[col+k][row+l] = 3;
              }
            }
          }
        }
      }
    }
  }
}

bool operator == (Point a, Point b)
{
  return a.x == b.x && a.y == b.y;
}
// Callback functions
void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  if(!map_created) {

    resolution = msg->info.resolution;
    size_x = msg->info.height; // number of cells in x dir
    size_y = msg->info.width; // number of cells in y dir

    //ROS_INFO("width %i, height %i", msg->info.height, msg->info.width);

    exp_rad = round(rad/resolution)+1;
    smooth_rad = round(rad/resolution);

    map_for_planning = new int*[size_y];
    map_for_smooth = new int*[size_y];

    int c = 0;
    for(int i = 0; i < size_y; i++) {
      map_for_planning[i] = new int[size_x];
      map_for_smooth [i] = new int[size_x];
    }

    for(int i = 0; i < size_x; i++) {

      for(int j = 0; j < size_y; j++) {
        map_for_planning[j][i] = (msg->data[c]==100?100:1);
        map_for_smooth[j][i] = (msg->data[c]==100?100:1);
        c++;

      }

    }
    map_created = true;
  }

  else if(read_map)
  {
    //ROS_INFO("Reading Map Again");
    int c = 0;
    for(int i = 0; i < size_x; i++) {
      //c = i;
      for(int j = 0; j < size_y; j++) {
        map_for_planning[j][i] = (msg->data[c]==100?100:1);
        map_for_smooth[j][i] = (msg->data[c]==100?100:1);
        c++;
       }
    }
    read_map = false;
  }
}

void ObjectCallback(const geometry_msgs::Pose::ConstPtr& msg){
  int x, y;
  if(map_created){
    x = round(msg->position.x/resolution);
    y = round(msg->position.y/resolution);
    objects.push_back(Point {x, y});
    std::cout << "object at: "<< x <<"," << y << std::endl;
  }
}

void EndPointCallback(const geometry_msgs::Pose::ConstPtr& msg){
  if(map_created)
  {
    goal_x = round(msg->position.x/resolution);
    goal_y = round(msg->position.y/resolution);
    std::cout << "GOAL: " << goal_x << "," << goal_y << std::endl;
    have_goal = true;
  }
}

void StartPointCallback(const geometry_msgs::Pose::ConstPtr& msg){
  if(map_created)
  {
    start_x = round((msg->position.x)/resolution);
    start_y = round(msg->position.y/resolution);
    //std::cout << "START: " << start_x << "," << start_y << std::endl;
    ROS_INFO("STart %d, %d", start_x, start_y);
    have_start = true;
  }
}

void saveCallback(const std_msgs::BoolConstPtr& msg) {
    save = true;
}

void BatteryCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
  if(map_created){

    int x, y;

    for (int i = 0; i < msg->poses.size(); i++){
      x = round(msg->poses[i].position.x/resolution);
      y = round(msg->poses[i].position.y/resolution);
      Point id {x, y};
      std::vector<Point>::iterator it = find(batteries.begin(), batteries.end(), id);
      if (it == batteries.end()){
        batteries.push_back(id);
      }
    }

  }
}

bool operator == (GridLocation a, GridLocation b)
{
  return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b)
{
  return !( a == b );
}

bool operator < (GridLocation a, GridLocation b)
{
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}



template<class Graph>
void draw_grid(const Graph& grid, int field_width, int** map, std::vector<GridLocation>* path=nullptr) {


  ROS_INFO("width %i, height %i", size_x, size_y);

  std::cout << std::endl;
  std::cout << "   ";
  for (int y = 0; y != size_x; y++)
    std::cout << "_" ;
  std::cout << std::endl;
  std::cout << "   ";
  for (int y = 0; y != size_x; y++)
    std::cout << ((int)((float)y/10.0)>9?(int)((float)y/100.0):(int)((float)y/10.0)) ;
  std::cout << std::endl;
  std::cout << "   ";
  for (int y = 0; y != size_x; y++)
    std::cout << y%10 ;
  for (int x = 0; x != size_y; x++){
    std::cout << '\n'<<(x<10?80+x:x) << "|";
    for (int y = 0; y != size_x; y++){
      GridLocation id {x, y};
      std::cout << std::left << std::setw(field_width);
      if (grid.walls.find(id) != grid.walls.end()){
        std::cout << std::string(field_width, '#');
      }
      else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()){
        std::cout << '@';
      }
      else if(map[x][y] == 7){
          std::cout << 7;
      }
      else if(map[x][y] == 3){
          std::cout << 3;
      }
      else{
        std::cout << '.';
      }
    }

  }
}

template<typename T, typename priority_t>
struct PriorityQueue{
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;
  inline bool empty() const{
    return elements.empty();
  }
  inline void put(T item, priority_t priority){
    elements.emplace(priority, item);
  }
  T get(){
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

GridLocation find_new_point(GridLocation id, int radius, SquareGrid grid){

  for (int i = 1; i <= radius; i++){
    for (GridLocation dir : SquareGrid::DIRS){
      GridLocation next{id.x + i*dir.x, id.y + i*dir.y};
      if ((grid.walls.find(next) == grid.walls.end()) && 0 <= next.x && next.x < size_x && 0 <= next.y && next.y < size_y){

        return next;
      }
    }
  }
  GridLocation default_val{0,0};
  return default_val;
}


template<typename Location>
std::vector<Location> reconstruct_path(std::map<Location, Location> came_from) {
 // ROS_INFO("Reconstructing Path");
  GridLocation start{start_x, start_y};
  GridLocation goal{goal_x, goal_y};
  std::vector<Location> path;
  Location current = goal;
  while (current != start){
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

inline int heuristic(GridLocation a, GridLocation b) {
  return (std::abs(a.x - b.x) + (std::abs(a.y - b.y)));
}

template<typename Location, typename Graph>
bool a_star_search(const Graph& grid,
                  std::map<Location,Location>& came_from,
                  std::map<Location, float>& cost_so_far,
                  int radius,
                  int** map){
 // ROS_INFO("Started A star");
  PriorityQueue<Location, float> frontier;

  GridLocation start{start_x, start_y};
  GridLocation goal{goal_x, goal_y};

  if (grid.walls.find(start) != grid.walls.end()){

    GridLocation new_start = find_new_point(start, radius, grid);

    if (new_start.x != 0 && new_start.y != 0){
      start_x = new_start.x;
      start_y = new_start.y;
      start = new_start;
      std::cout << "New valid start: " << start_x << "," << start_y << std::endl;
    }

    else{
      std::cout << "Could not find valid start" << std::endl;
      cause = -1.0;
      return false;
    }
  }

  if (grid.walls.find(goal) != grid.walls.end()){

    GridLocation new_goal = find_new_point(goal, radius, grid);

    if (new_goal.x != 0 && new_goal.y != 0){
      goal_x = new_goal.x;
      goal_y = new_goal.y;
      goal = new_goal;
      std::cout << "New valid goal: " << goal.x << "," << goal.y << std::endl;
    }

    else{
      std::cout << "Could not find valid goal" << std::endl;
      cause = -2.0;
      return false;
    }
  }

  frontier.put(start, 0.0);
  came_from[start] = start;
  cost_so_far[start] = 0.0;
  float new_cost;

  while (!frontier.empty()){
    Location current = frontier.get();
    if (current == goal){
      ROS_INFO(" Path Found ");
      return true;
    }
    for (Location next : grid.neighbors(current)){
      new_cost = 1.0 * (float)map[next.x][next.y] + (float)cost_so_far[current];
      if((std::abs(current.x - next.x) + std::abs(current.y - next.y)) > 1)
      {
        new_cost = 1.414 * (float)map[next.x][next.y] + (float)cost_so_far[current];
      }
      if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
      {
        cost_so_far[next] = new_cost;
        float priority = new_cost + (float)heuristic(next, goal);
        frontier.put(next, priority);
        came_from[next] = current;
      }
      //}
    }
  }
  std::cout << "Path not found" << std::endl;
  cause = -3.0;
  return false;
}
std::vector<GridLocation> smoothing(std::vector<GridLocation> path, SquareGrid grid, int** map) {
  //ROS_INFO("Smoothing");
  std::vector<GridLocation> new_path;
  GridLocation cur_node{start_x, start_y};
  int cur_weight = map[start_x][start_y];
    //ROS_INFO("init weight %d",cur_weight);
  GridLocation last_seen_node{start_x, start_y};
  new_path.push_back(cur_node);
  int index = 1;
  bool ended;
 // std::cout << path.size() << std::endl;

  while(last_seen_node.x != path[path.size()-1].x || last_seen_node.y != path[path.size()-1].y ) {
         //ROS_INFO("inside path loop");

    for(int i = index ; i < path.size() ; i++) {
      //ROS_INFO("weight': %d  curr weight: %d",map[path[i-1].x][path[i-1].y],cur_weight);
      int cell_x, cell_y;
      double d, theta, alpha;

      d = sqrt(pow(path[i].x - cur_node.x, 2) + pow(path[i].y - cur_node.y, 2));
      //if(path[i].x != cur_node.x){
        theta = atan2((path[i].y-cur_node.y) , (path[i].x-cur_node.x));
     // }
    //  else{
      //  theta = M_PI/2;
     // }
      alpha = 0.5;
      ended = false;

      for(double j = alpha; j < d; j+=alpha){
        cell_x = cur_node.x + round(j*cos(theta));
        cell_y = cur_node.y + round(j*sin(theta));
        //ROS_INFO("inside the wall detector loop");
        //if(cell_x==32&&cell_y==72)
        //ROS_INFO("thetea: %f j: %f d: %f currN X: %d currN Y: %d",theta,j,d,cur_node.x,cur_node.y);
        if(grid.walls.find(GridLocation{cell_x, cell_y}) != grid.walls.end() || map[cell_x][cell_y] != cur_weight || (abs(cur_node.x-cell_x)+abs(cur_node.y-cell_y)) > 12 )
        {
          //ROS_INFO("previous node whe: %d next node weight: %d target weight: %d cur_weight: %d X: %d Y: %d",map[path[i-1].x][path[i-1].y], map[path[i].x][path[i].y], map[cell_x][cell_y],cur_weight , cell_x,cell_y);
          //ROS_INFO("Hit a wall");
          j = d/alpha;
          cur_node = GridLocation{path[i-1].x, path[i-1].y};
          new_path.push_back(cur_node);
          if(map[path[i-1].x][path[i-1].y] != map[path[i].x][path[i].y] || (i==index && map[cell_x][cell_y] != cur_weight)){
            cur_node = GridLocation{path[i].x, path[i].y};
            new_path.push_back(cur_node);
            cur_weight = map[path[i].x][path[i].y];
          }
          index = i;
          i = path.size();
          ended = true;
        }
      }

    if(!ended) {
      last_seen_node = GridLocation{path[i].x, path[i].y};
    }
  }
}
new_path.push_back(GridLocation{path[path.size()-1].x, path[path.size()-1].y});
return new_path;
}

int main(int argc, char **argv)
{
   ROS_INFO("1");
  ros::init(argc, argv, "path_planning_node");
  ros::NodeHandle n_;
   ROS_INFO("2");

  float frequency;
  std::vector<GridLocation> path;
  std::vector<GridLocation> smoothed_path;
 ROS_INFO("3");
  n_.getParam("/frequency", frequency);
  n_.getParam("/robot/radius", rad);
 ROS_INFO("4");
  ros::Rate loop_rate(frequency);

  ros::Subscriber map_sub = n_.subscribe<nav_msgs::OccupancyGrid>("/map/base", 1, MapCallback);
  ros::Subscriber start_sub = n_.subscribe("/path_start_point", 1, StartPointCallback);
  ros::Subscriber end_sub = n_.subscribe("/path_end_point", 1, EndPointCallback);
  ros::Subscriber battery_sub = n_.subscribe<geometry_msgs::PoseArray>("/battery", 10, BatteryCallback);
  ros::Subscriber object_sub = n_.subscribe<geometry_msgs::Pose>("/object_position",10 , ObjectCallback);
  ros::Subscriber save_sub = n_.subscribe("/master/save", 1, saveCallback);
  ros::Publisher path_pub = n_.advertise<geometry_msgs::PoseArray>("/path_plan", 1);
 ROS_INFO("5");
  std::ifstream battery_file("/home/ras28/catkin_ws/src/ras_project/ras_data/path_planner_batteries.csv", std::ifstream::in);
  std::ifstream object_file("/home/ras28/catkin_ws/src/ras_project/ras_data/path_planner_objects.csv", std::ifstream::in);
  ROS_INFO("file");
  if(!battery_file.fail() && !object_file.fail()) {

      std::string c_x1, c_x2;
      int x, y;

      for (std::string line; std::getline(battery_file, line, ',');)
          {
            c_x1 = line;
            std::getline(battery_file, c_x2,',');
            x = std::atoi(c_x1.c_str());
            y = std::atoi(c_x2.c_str());
            batteries.push_back(Point{x, y});
          }

      for (std::string line; std::getline(object_file, line, ','); )
          {
            c_x1 = line;
            std::getline(object_file, c_x2, ',');
            x = std::atoi(c_x1.c_str());
            y = std::atoi(c_x2.c_str());
            objects.push_back(Point{x, y});
          }

      battery_file.close();
      object_file.close();
  }


  while(!map_created && (ros::ok() && n_.ok())){
    ROS_INFO("Mspinning");
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("Map not created");
  }

  while (ros::ok() && n_.ok())
  {
   // ROS_INFO("Map created");
    ros::spinOnce();

    if(save) {

        std::ofstream outfile1("/home/ras28/catkin_ws/src/ras_project/ras_data/path_planner_batteries.csv");
        std::ofstream outfile2("/home/ras28/catkin_ws/src/ras_project/ras_data/path_planner_objects.csv");


        for(int j=0; j < objects.size(); ++j){
          outfile2 << objects[j].x << ',';
          outfile2 << objects[j].y << ',';
        }

        outfile2.close();


        for(int i = 0; i < batteries.size(); ++i){
                outfile1 << batteries[i].x << ',';
                outfile1 << batteries[i].y << ',';
            }


        outfile1.close();

        save = false;

    }


    if (have_start && have_goal ){
      ROS_INFO("start and goal");

      read_map = true;

      while(read_map && (ros::ok() && n_.ok())){
        ros::spinOnce();
        loop_rate.sleep();
      }

      have_start = false;
      have_goal = false;

      SquareGrid exp_batteries = addObstacle(exp_rad, map_for_planning);
      SquareGrid smooth_batteries = addObstacle(smooth_rad, map_for_smooth);

      exp_batteries = addBatteries(exp_batteries, exp_rad);
      smooth_batteries = addBatteries(smooth_batteries, smooth_rad);

      map_weighing(map_for_planning, exp_batteries);
      map_weighing(map_for_smooth, smooth_batteries);

      //draw_grid(exp_batteries, 1, map_for_planning);

      std::map<GridLocation, GridLocation> came_from;
      std::map<GridLocation, float> cost_so_far;

      bool foundpath = a_star_search(exp_batteries, came_from, cost_so_far, exp_rad, map_for_planning);

      if (foundpath){

        path = reconstruct_path(came_from);
        smoothed_path = smoothing(path, smooth_batteries, map_for_smooth);
        draw_grid(exp_batteries, 1, map_for_planning, &path);
       // draw_grid(smooth_batteries, 1, map_for_smooth);

        //draw_grid(smooth_batteries, 1, map_for_smooth, &smoothed_path);

        geometry_msgs::PoseArray pose_array;
        for (int i = 0; i < smoothed_path.size(); i++){
          geometry_msgs::Pose pose;
          pose.position.x = resolution * smoothed_path[i].x;
          pose.position.y = resolution * smoothed_path[i].y;
          pose_array.poses.push_back(pose);
        }
        path_pub.publish(pose_array);
      }

     if(!foundpath){
        // -1.0 could not find valid start, -2.0 could not find valid goal, -3.0 Path not found
        geometry_msgs::PoseArray pose_array;
        geometry_msgs::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = cause;
        pose_array.poses.push_back(pose);
        path_pub.publish(pose_array);
        }
    }
    loop_rate.sleep();
  }
  return 0;
}
