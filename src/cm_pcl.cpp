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
    }
    // ROS_INFO("%.3f %.3f %.3f", point_xyz.x, point_xyz.y, point_xyz.z);

    // cout << cloud.at(msg->point_num-1).x << " " << cloud.at(msg->point_num-1).y << " " << cloud.at(msg->point_num-1).z << endl;
    // cloud and point_xyz match complete
    cloud.clear();
}

void asdad()
{

}

void cm_matrix()
{
    // realsense camera K matrix -> fx fy cx cy
    Eigen::Matrix3d K_matrix;
    
    K_matrix << 601.654608, 0.000000, 326.296932, 0.000000, 603.568857, 244.054371, 0.000000, 0.000000, 1.000000; 
    cout << K_matrix(0,1) << " " << K_matrix(1,1) << " " <<K_matrix(2,1) << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub;
    topic_sub = nh.subscribe("/livox/lidar", 1000, topiccb);
    cm_matrix();
    cv::Mat image;
    ros::spin();
    return 0;
}