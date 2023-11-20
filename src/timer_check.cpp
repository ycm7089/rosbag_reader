#include <ros/time.h>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef Eigen::Matrix<float, 3, 4> Matrix3x4;
typedef Eigen::Matrix<float, 4, 1> Matrix4x1;

ros::Subscriber odom_sub;
ros::Subscriber camera_sub;
ros::Subscriber ouster_sub;

ros::Publisher lidar_pub;

nav_msgs::Odometry odom;
tf::TransformBroadcaster br;
cv::Mat sub_image;

bool is_in_img(int u, int v)
{
    if ( 0 <= u && u <640 && 0 <= v && v <480)

        return true;
    else

        return false;
}

void repub(pcl::PointCloud<pcl::PointXYZI>& cloud){
    pcl::PointXYZI point_rgb;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 cloud_out;
    // std::cout << cloud.points.size() << std::endl;
    for (int i = 0; i < cloud.points.size(); i++)
    {  
        if (cloud.points[i].z > 1.0 ) continue;

        point_rgb.x = cloud.points[i].x;
        point_rgb.y = cloud.points[i].y;
        point_rgb.z = cloud.points[i].z;
        point_rgb.intensity = cloud.points[i].intensity;


        output_cloud -> points.push_back(point_rgb);
    }

    pcl::toROSMsg(*output_cloud, cloud_out);

    cloud_out.header.frame_id = "os_sensor";

    cloud_out.header.stamp = ros::Time::now();

    lidar_pub.publish(cloud_out);

    output_cloud -> clear(); 
    cloud_out.data.clear();
    // point_rgb.x = cloud.points[0].x;

}

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    sub_image = cv_bridge::toCvShare(msg, "bgr8")-> image;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom.pose.pose.position.x  = msg -> pose.pose.position.x;
    odom.pose.pose.position.y  = msg -> pose.pose.position.y;
    odom.pose.pose.position.z  = msg -> pose.pose.position.z;

    odom.pose.pose.orientation.x  = msg -> pose.pose.orientation.x;
    odom.pose.pose.orientation.y  = msg -> pose.pose.orientation.y;
    odom.pose.pose.orientation.z  = msg -> pose.pose.orientation.z;
    odom.pose.pose.orientation.w  = msg -> pose.pose.orientation.w;
}

void ouster_cb(const sensor_msgs::PointCloud2::ConstPtr& msg ){
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_ouster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*new_ouster_cloud);

    // tf::Quaternion q(
    //     odom.pose.pose.orientation.w,
    //     odom.pose.pose.orientation.x,
    //     odom.pose.pose.orientation.y,
    //     odom.pose.pose.orientation.z   
    // );

    // cout << odom.pose << endl;
    // repub(*new_ouster_cloud);
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "intensity_pub");
    ros::NodeHandle nh;
    
    odom_sub = nh.subscribe("/ranger_base_node/odom", 1000, odom_cb);
    ouster_sub = nh.subscribe("/ouster/points", 1000, ouster_cb);
    camera_sub = nh.subscribe("/camera/color/image_raw", 1000, image_cb);
    
    // lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("ouster/points2",1);
    // ros::Timer timer1 = nh.createTimer(ros::Duration(0.1),cb);

    ros::spin();
    return 0;
}