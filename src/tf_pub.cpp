#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
// #include <tf/tf_
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
// tf base to lidar 0 0 82
// tf base to camera 0 0 72
using namespace std;

ros::Subscriber odom_sub;


geometry_msgs::PoseStamped robot_pose;


void tf_pub(geometry_msgs::PoseStamped robot_pose, double yaw)
{
    
    tf::TransformBroadcaster br; 
    tf::Transform transform;

    // // odom subscribe 받아야할듯
    // transform.setOrigin( tf::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, 0.0) );
    // transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0,yaw));
    transform.setOrigin( tf::Vector3(0.0,0.0, 0.0) );
    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0,0.0));    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

    transform.setOrigin( tf::Vector3(0.0,0.0, 0.0) );
    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0,0.0));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.82) );
    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0,0.0));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "velodyne"));
}


void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{


    // robot_pose.header.frame_id = "map";
    // robot_pose.header.stamp = ros::Time::now();

    robot_pose.pose.position.x = msg -> pose.pose.position.x;
    robot_pose.pose.position.y = msg -> pose.pose.position.y;
    robot_pose.pose.position.z = 0;

    robot_pose.pose.orientation.w = msg -> pose.pose.orientation.w;
    robot_pose.pose.orientation.x = msg -> pose.pose.orientation.x;
    robot_pose.pose.orientation.y = msg -> pose.pose.orientation.y;
    robot_pose.pose.orientation.z = msg -> pose.pose.orientation.z;

    tf::Quaternion q(
        robot_pose.pose.orientation.w,
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z   
        );

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);

    cout << robot_pose << endl;

    tf_pub(robot_pose, yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_pub_node");

    ros::NodeHandle nh("~");

    odom_sub = nh.subscribe("/odom", 1000, odom_cb);
    
    ros::spin();

    return 0;
}