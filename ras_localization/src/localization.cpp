#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <algorithm>
#include <limits>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


// global variables
int N_particles = 1000;
float HI = 0.02;
float LO = -0.02;
float HI_T = M_PI/180.0;
float LO_T = -M_PI/180.0;
float threshold_t = 5.0;
float threshold_fx = 5.0;
float threshold_fy = 5.0;
float smoothing_factor = 0.3;
int T = 1;
float lidar_data[360];
bool lidarReady = false;
int cell_h, cell_w;
float map_granularity;
int** map = NULL;
bool mapRead = false;
bool readNewMap = false;
float delta_x, delta_y, delta_theta;
double frequency = 30.0;


// output variables
float pos_x, pos_y, pos_theta;

// smoothed output variables
float pos_x_s, pos_y_s, pos_theta_s;


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

// read odometry data callback
void odomCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {

    delta_x = msg->x;
    delta_y = msg->y;
    delta_theta = msg->theta;
}

// read lidar data callback
void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg) { // TODO: scale lidar measurements

    int mCount = 0;
    for(int i = 0; i < 360; i++)
    {
        if(!isinf(msg->ranges[i])) {
            lidar_data[i] = 1.013 *msg->ranges[i];//Magic number to correct measurement
            mCount++;
        } else {
            lidar_data[i] = 0;
        }
    }
    if(mCount > 0)
        lidarReady = true;
}

// map has changed, read it again
void changeMapCallback(const std_msgs::Bool::ConstPtr &msg) {
    readNewMap = true;
}

class Particle {
    public:
        float x, y, theta;
        float value;
};

int main(int argc, char** argv) {

    int i, j, k;
    float sum_value;
    double torque=0.0, forceX=0.0, forceY=0.0;
    float pred_x, pred_y;
    double angle, phi, alpha=0.041;
    int Fx=0, Fy=0;
    int max=0;
    int dx, dy;
    int x_meas, y_meas;
    float cdf[N_particles];
    float u, c;
    int index;
    double mean_sin, mean_cos;
    
    
    int min_outliers_map = 20;
    int min_distance_for_outlier = 98;
    
    
    
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    nh.getParam("/frequency_mc", frequency);
    nh.getParam("/localization/N_particles", N_particles);
    nh.getParam("/localization/T", T);
    nh.getParam("/localization/x_error", HI);
    nh.getParam("/localization/theta_error", HI_T);
    nh.getParam("/localization/F_threshold", threshold_fx);
    nh.getParam("/localization/T_threshold", threshold_t);
    nh.getParam("/robot/base_link_x_bias", pos_x);
    nh.getParam("/robot/base_link_y_bias", pos_y);
    nh.getParam("/robot/base_link_yaw_bias", pos_theta);
    
    nh.getParam("/mapping/N_outliers", min_outliers_map);
    nh.getParam("/mapping/d_outliers", min_distance_for_outlier);
    
    pos_x_s = pos_x;
    pos_y_s = pos_y;
    pos_theta_s = pos_theta;

    LO = -HI;
    LO_T = -HI_T;
    threshold_fy = threshold_fx;
    
    ros::Rate r(frequency);
        
    ros::Subscriber sub_lidar = n.subscribe("/scan", 1, lidarCallback);
    ros::Subscriber sub_map = n.subscribe("/map/extended", 1, readMapCallback);
    ros::Subscriber sub_odom = n.subscribe("/robot/position_change", 1, odomCallback);
    ros::Subscriber sub_map_change = n.subscribe("/map/change", 10, changeMapCallback);
    
    ros::Publisher position_pub = n.advertise<geometry_msgs::Pose2D>("/robot/position", 10);
    ros::Publisher map_pub = n.advertise<std_msgs::Float32>("/map/request_modification", 10);
        
    geometry_msgs::Pose2D robot_pose;
    std_msgs::Float32 request;
        
    // loop until lidar data is reliable, update robot position based on odometry data
    while(n.ok() && !lidarReady) {
        
        ros::spinOnce();
        
        pos_x += delta_x;
        pos_y += delta_y;
        pos_theta += delta_theta;
        
        pos_x_s = pos_x;
        pos_y_s = pos_y;
        pos_theta_s = pos_theta;

        robot_pose.x = pos_x_s;
        robot_pose.y = pos_y_s;
        robot_pose.theta = pos_theta_s;
        robot_pose.theta = fmod(robot_pose.theta, 2.0 * M_PI);

        position_pub.publish(robot_pose);
        
        r.sleep();
    }
    
    
    
    // create set of particles and spread them around the odometry position
    Particle particles[N_particles];
    Particle particles_new[N_particles];
    
    srand(static_cast <unsigned> (1000));
    
    for(i = 0; i < N_particles; i++) {
        Particle p;
        p.x = pos_x + (LO + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(HI-LO))));
        p.y = pos_y + (LO + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(HI-LO))));
        p.theta = pos_theta + (LO_T + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(HI_T-LO_T))));
        p.theta = fmod(p.theta, 2.0 * M_PI);
        p.value = 1.0;
        particles[i] = p;
    }
    
    // number of outliers for every particle, needed for mapping
    float outliers[N_particles];
    
    
    while(n.ok()) {
    
        // read messages and update believed position based on odometry
        ros::spinOnce();
        
        
        sum_value = 0;
            
        for(i = 0; i < N_particles; i++) {
        
            outliers[i] = 0.0;
        
            // update each particle pose based on odometry and add noise on x, y, theta
            particles[i].x += delta_x + (LO + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(HI-LO))));
            particles[i].y += delta_y + (LO + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(HI-LO))));
            particles[i].theta += delta_theta + (LO_T + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(HI_T-LO_T))));
            particles[i].theta = fmod(particles[i].theta, 2.0 * M_PI);
            
            //ROS_INFO("particle is in x: %f, y: %f, theta: %f", particles[0].x, particles[0].y, particles[0].theta);
        
            for(k = 0; k < T; k++) {
        
                torque=0.0;
                forceX=0.0;
                forceY=0.0;
            
                for(j = 0; j < 360; j++) {
                            
                    if(lidar_data[j] != 0) {
                        
                        phi = - ((double) j/360.0)*6.2832 - 0.01745;
                        angle = particles[i].theta + M_PI/2.0 - phi + alpha;
                        angle = fmod(angle, 2.0 * M_PI);
                        
                        pred_x = particles[i].x + lidar_data[j] * cos(angle);
                        pred_y = particles[i].y + lidar_data[j] * sin(angle);
                        
                        pred_x = (int) round((float) pred_x/map_granularity);
                        pred_y = (int) round((float) pred_y/map_granularity);
                        
                        
                        //ROS_INFO("lidar %d, pred_x = %f, pred_y = %f", j, pred_x, pred_y);
                        
                        Fx=0;
                        Fy=0;
                        max=0;
                        
                        
                        // compute the force this measurement is subject to, it is attracted by walls
                        for(dx = -1; dx <= 1; dx++) {
                            for(dy = -1; dy <= 1; dy++) {
                                x_meas = (int) pred_x + dx + floor((float) cell_w/4.0);
                                y_meas = (int) pred_y + dy + floor((float) cell_h/4.0);
                                if(y_meas >= 0 && y_meas < cell_h && x_meas >= 0 && x_meas < cell_w) {
                                
                                    if(max < map[y_meas][x_meas]) {
                                        max = map[y_meas][x_meas];
                                        Fx = dx;
                                        Fy = dy;
                                    } else if(max == map[y_meas][x_meas]) {
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
                        
                        //ROS_INFO("Fx: %d, Fy: %d", Fx, Fy);
                        
                        torque += ((pred_x*map_granularity-particles[i].x)*Fy - (pred_y*map_granularity-particles[i].y)*Fx);
                        forceX += Fx;
                        forceY += Fy;
                    }
                                
                }
                
                //ROS_INFO("torque: %lf, force on x: %lf, force on y: %lf", torque, forceX, forceY);
                
                // theta needs to be rotated
                if(abs(torque) > threshold_t) {
                    particles[i].theta += (torque/abs(torque)) * M_PI/180.0;
                    particles[i].theta = fmod(particles[i].theta, 2.0 * M_PI);

                }
                
                // x needs to be shifted
                if(abs(forceX) > threshold_fx) {
                    particles[i].x += (forceX/abs(forceX)) * map_granularity;
                }
                
                // y needs to be shifted
                if(abs(forceY) > threshold_fy) {
                    particles[i].y += (forceY/abs(forceY)) * map_granularity;
                }
            
            }
            
            // give value to the particle after it has been adjusted
            for(j = 0; j < 360; j++) {
                if(lidar_data[j] != 0) {
                
                    phi = - ((double) j/360.0)*6.2832 - 0.01745;
                    angle = particles[i].theta + M_PI/2.0 - phi + alpha;
                    angle = fmod(angle, 2.0 * M_PI);
                        
                    pred_x = particles[i].x + lidar_data[j] * cos(angle);
                    pred_y = particles[i].y + lidar_data[j] * sin(angle);
                        
                    pred_x = round(pred_x/map_granularity);
                    pred_y = round(pred_y/map_granularity);
                
                    x_meas = (int) pred_x + dx + floor((float) cell_w/4.0);
                    y_meas = (int) pred_y + dy + floor((float) cell_h/4.0);
                    
                    if(y_meas >= 0 && y_meas < cell_h && x_meas >= 0 && x_meas < cell_w) {
                                
                        particles[i].value += std::pow((double) map[y_meas][x_meas]/100.0, (double) 40.0);
                        if(map[y_meas][x_meas] < 100) {
                            outliers[i]++;
                        }
                    }
                }
            }
            
            sum_value += particles[i].value;
        
        }
        
        // resample the particles based on the values given preaviously and estimate position after resample
        cdf[0] = 0.0;
        for(i = 1; i < N_particles; i++) {
            cdf[i] = cdf[i-1] + particles[i-1].value / sum_value;
        }
                
                
        u = static_cast <float> (rand()) / ( static_cast <float> (RAND_MAX/((float) 1.0/N_particles)) );
        //c = u;
        
        pos_x = 0.0;
        pos_y = 0.0;
        pos_theta = 0.0;
        mean_sin = 0.0;
        mean_cos = 0.0;
                
        index = 0;
                
        
        for(i = 0; i < N_particles; i++) {
            for(j = index; j < N_particles; j++) {
                    
                if(cdf[j] > u) {
                    index = j - 1;
                    j = N_particles;
                }
            }
                    
            particles_new[i].x = particles[index].x;
            particles_new[i].y = particles[index].y;
            particles_new[i].theta = particles[index].theta;
            u += (float) 1.0/N_particles;
                    
            pos_x += particles_new[i].x;
            pos_y += particles_new[i].y;
            //pos_theta += particles_new[i].theta;
            mean_sin += sin(particles_new[i].theta);
            mean_cos += cos(particles_new[i].theta);
        }
        
        for(i = 0; i < N_particles; i++) {
            particles[i].x = particles_new[i].x;
            particles[i].y = particles_new[i].y;
            particles[i].theta = particles_new[i].theta;
            particles[i].value = 1.0;
        }
                
        pos_x = pos_x / N_particles;
        pos_y = pos_y / N_particles;
        pos_theta = atan2((double) mean_sin / N_particles, (double) mean_cos / N_particles);
        pos_theta = fmod(pos_theta, 2.0 * M_PI);

        //ROS_INFO("theta: %f",pos_theta);

        //update estimates with odometry
        pos_x_s += delta_x;
        pos_y_s += delta_y;
        pos_theta_s += delta_theta;

        //add some of the new estimate to the runing estimate
        pos_x_s = (1.0 - smoothing_factor)*pos_x_s + smoothing_factor * pos_x;
        pos_y_s = (1.0 - smoothing_factor)*pos_y_s + smoothing_factor * pos_y;
        //pos_theta_s = (1.0 - smoothing_factor)*pos_theta_s + smoothing_factor * pos_theta;
        pos_theta_s = pos_theta;

        // published smoothed computed pose of the robot
        robot_pose.x = pos_x_s;
        robot_pose.y = pos_y_s;
        robot_pose.theta = pos_theta_s;
        robot_pose.theta = fmod(robot_pose.theta, 2.0 * M_PI);

    
        position_pub.publish(robot_pose);
        
        pos_x = pos_x_s;
        pos_y = pos_y_s;
        pos_theta = pos_theta_s;
        
        // if there are too many outliers call mapping
        if(*std::min_element(outliers, outliers+N_particles) > min_outliers_map) {
            request.data = *std::min_element(outliers, outliers+N_particles);
            map_pub.publish(request);
        }
        
                
        r.sleep();
    }
    
    
    
}











