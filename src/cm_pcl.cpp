#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;
void topiccb(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{

    ROS_INFO("=============");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ point_xyz;
    ROS_INFO("num %d", msg->point_num);
    for (int i = 0; i < msg->point_num; i++)
    {
        point_xyz.x = msg->points[i].x;
        point_xyz.y = msg->points[i].y;
        point_xyz.z = msg->points[i].z;
        cloud.push_back(point_xyz);
        ROS_INFO("%.3f %.3f %.3f", point_xyz.x, point_xyz.y, point_xyz.z);
    }
    // cout << cloud << endl;
    // printf(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub;
    topic_sub = nh.subscribe("/livox/lidar", 1000, topiccb);
 
    cv::Mat image;
    ros::spin();
    return 0;
}