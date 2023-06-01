#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "loam_pcd_read_node");
    ros::NodeHandle nh ("~");

    ros::Publisher PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_pub",1);
    ros::Publisher seg_PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lego_loam_pub",1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("new_odom",1);

    sensor_msgs::PointCloud2 cloud_out;
    sensor_msgs::PointCloud2 seg_cloud_out;
    nav_msgs::Odometry odom;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    
    pcl::PointXYZRGB point_rgb;  
    pcl::PointXYZRGB seg_point_rgb; 

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 
    tf::TransformBroadcaster br; 

    int num_pcd = 2537;
    FILE* file;
    file = fopen("/home/kimm/pcd_img_data/txt/re_0413_txt.txt","r");
    
    string rostime;
    float pose_x,pose_y,pose_z,orien_x,orien_y,orien_z,orien_w;
    
    for(int i = 0; i < num_pcd; i ++)
    {
        pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/kimm/pcd_img_data/trav_pcd/" + std::to_string(i) + ".pcd", cloud);
        
        tf::Transform transform;

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne"));
        
        transform.setOrigin( tf::Vector3(0.0, 0.0, -0.82) );
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0,0.0));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "base_link"));
        
        odom.header.frame_id = "map";
        odom.header.stamp = ros::Time::now();
        
        odom_pub.publish(odom);
    
        for(int j = 0; j < cloud.size(); j ++)
        {

            seg_point_rgb.x = cloud.points[j].x;
            seg_point_rgb.y = cloud.points[j].y;
            seg_point_rgb.z = cloud.points[j].z;
            seg_point_rgb.r = cloud.points[j].r;
            seg_point_rgb.g = cloud.points[j].g;
            seg_point_rgb.b = cloud.points[j].b;
            
            seg_output_cloud -> points.push_back(seg_point_rgb);   
            // cout << seg_point_rgb.x << " " << seg_point_rgb.y << " " << seg_point_rgb.z << " " << endl;
            printf("r is %d g is %d b is %d \n", seg_point_rgb.r,seg_point_rgb.g,seg_point_rgb.b);
        }
        pcl::toROSMsg(*seg_output_cloud, seg_cloud_out);
        
        seg_cloud_out.header.frame_id = "velodyne";
        seg_cloud_out.header.stamp = ros::Time::now();
        seg_PointCloud_pub.publish(seg_cloud_out);
        
        seg_cloud_out.data.clear();
        
        seg_output_cloud -> clear();
        
        for (const auto& point_rgb: cloud)
        {
            output_cloud -> points.push_back(point_rgb);
        }

        pcl::toROSMsg(*output_cloud, cloud_out);
        
        cloud_out.header.frame_id = "velodyne";
        cloud_out.header.stamp = ros::Time::now();
        
        PointCloud_pub.publish(cloud_out);

        output_cloud -> clear();

        cloud_out.data.clear();
        std::cout << i << "th publish complete" << std::endl;
    }
    ros::spinOnce();

    return 0;
}