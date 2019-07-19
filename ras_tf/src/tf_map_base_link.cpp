#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include "phidgets/motor_encoder.h"
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


double x_b = 0.0;
double y_b = 0.0;
double yaw_b = 0.0;
int frequency = 30;

void trCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    x_b = msg->x;
    y_b = msg->y;
    yaw_b = msg->theta;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "robot_map_base_link_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.getParam("/robot/base_link_x_bias", x_b);
    nh.getParam("/robot/base_link_y_bias", y_b);
    nh.getParam("/robot/base_link_yaw_bias", yaw_b);
    nh.getParam("/frequency", frequency);

    ros::Rate r(frequency);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;
    
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";

    ros::Subscriber sub_motor1 = n.subscribe("/robot/position", 10, trCallback);


    while(n.ok()) {

        ros::spinOnce();
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = x_b;
        transformStamped.transform.translation.y = y_b;
        transformStamped.transform.translation.z = 0.0;
        q.setRPY(0, 0, yaw_b);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
        
        r.sleep();
    }
}
