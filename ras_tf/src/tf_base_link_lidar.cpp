#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

double x_l = 0.0;
double y_l = 0.0;
double yaw_l = 0.0;
int frequency = 30;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_base_link_lidar_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.getParam("/robot/lidar_x_bias", x_l);
    nh.getParam("/robot/lidar_y_bias", y_l);
    nh.getParam("/robot/lidar_yaw_bias", yaw_l);
    nh.getParam("/frequency", frequency);

    ros::Rate r(frequency);

    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion quaternion;

    while(n.ok()){

        transform.setOrigin( tf::Vector3(x_l, y_l, 0.0) );
        quaternion.setRPY(0.0, 0.0, yaw_l);
        transform.setRotation(quaternion);

        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/base_link", "/laser"));
        r.sleep();
    }
}
