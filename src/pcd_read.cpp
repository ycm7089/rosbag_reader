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

typedef Eigen::Matrix<float, 3, 4> Matrix3x4;
typedef Eigen::Matrix<float, 4, 1> Matrix4x1;

bool is_in_img(int u, int v)
{
    if ( 0 <= u && u <640 && 0 <= v && v <480)

        return true;
    else

        return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_read_node");
    ros::NodeHandle nh ("~");

    ros::Publisher PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_pub",1);
    ros::Publisher seg_PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("seg_pointcloud_pub",1);
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

    Eigen::Matrix4f Extrinsic_matrix;

    Matrix3x4 KE_Matrix;

    Matrix4x1 xyz_result;

    KE_Matrix = Matrix3x4::Zero();

    Extrinsic_matrix << 0.0, -1.0, 0.0, 0.1284,
                        0.0,  0.0,-1.0, -0.063,
                        1.0,  0.0, 0.0, -0.055,
                        0,0,0,1;

    float fx = 603.5733;
    float fy = 603.8386;
    float cx = 316.1940;
    float cy = 246.2663;
    
    for(int j = 0; j <4; j++)
    {
        KE_Matrix(0,j) = fx * Extrinsic_matrix(0,j) + cx * Extrinsic_matrix(2,j);
        KE_Matrix(1,j) = fy * Extrinsic_matrix(1,j) + cy * Extrinsic_matrix(2,j);
        KE_Matrix(2,j) = Extrinsic_matrix(2,j);
    }

    int num_pcd = 5667;

    FILE* file;
    file = fopen("/home/cm/pcd_img_data/txt/re_0413_txt.txt","r");
    
    string rostime;
    float pose_x,pose_y,pose_z,orien_x,orien_y,orien_z,orien_w;
    
    for(int i = 0; i < num_pcd; i ++)
    {
        pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/cm/pcd_img_data/resaved_pcd/" + std::to_string(i) + ".pcd", cloud); //* load the file
        // cv::Mat image = cv::imread("/home/cm/pcd_img_data/resaved_img/" + std::to_string(i) + ".png",cv::IMREAD_COLOR);
        cv::Mat image = cv::imread("/home/cm/pcd_img_data/seg_img/" + std::to_string(i) + ".png",cv::IMREAD_COLOR);
        // cout << image.
        fscanf(file,"%f %f %f %f %f %f %f", &pose_x, &pose_y, &pose_z, &orien_x, &orien_y, &orien_z, &orien_w);
        
        odom.pose.pose.position.x = pose_x;
        odom.pose.pose.position.y = pose_y;
        odom.pose.pose.position.z = pose_z;

        odom.pose.pose.orientation.x = orien_x;
        odom.pose.pose.orientation.y = orien_y;
        odom.pose.pose.orientation.z = orien_z;
        odom.pose.pose.orientation.w = orien_w;

        tf::Quaternion q(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z   
        );

        tf::Quaternion cm_q(orien_x, orien_y, orien_z, orien_w);

        // tf::Matrix3x3 m(q);
        tf::Matrix3x3 ms(cm_q);
        double roll, pitch, yaw;
        // m.getRPY(roll,pitch,yaw);
        ms.getRPY(roll,pitch,yaw);
        
        tf::Transform transform;

        transform.setOrigin( tf::Vector3(pose_x,pose_y, pose_z) );
        transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne"));
        
        transform.setOrigin( tf::Vector3(0.0, 0.0, -0.82) );
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0,0.0));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "base_link"));
        
        odom.header.frame_id = "map";
        odom.header.stamp = ros::Time::now();

        odom_pub.publish(odom);
    
        for(int j = 0; j < cloud.size(); j ++)
        {
            xyz_result = Matrix4x1::Zero();
            xyz_result(3,0) = 1.0;
            
            if (cloud.points[j].x < 0.25) continue; 
            
            xyz_result[0] = KE_Matrix(0,0) * cloud.points[j].x +
                            KE_Matrix(0,1) * cloud.points[j].y +
                            KE_Matrix(0,2) * cloud.points[j].z +
                            KE_Matrix(0,3) * 1;

            xyz_result[1] = KE_Matrix(1,0) * cloud.points[j].x +
                            KE_Matrix(1,1) * cloud.points[j].y +
                            KE_Matrix(1,2) * cloud.points[j].z +
                            KE_Matrix(1,3) * 1;

            xyz_result[2] = KE_Matrix(2,0) * cloud.points[j].x +
                            KE_Matrix(2,1) * cloud.points[j].y +
                            KE_Matrix(2,2) * cloud.points[j].z +
                            KE_Matrix(2,3) * 1;

            int u = int(xyz_result[0] / xyz_result[2]);
            int v = int(xyz_result[1] / xyz_result[2]);
            int w = 1;

            if (is_in_img(u,v))
            {
                seg_point_rgb.x = cloud.points[j].x;
                seg_point_rgb.y = cloud.points[j].y;
                seg_point_rgb.z = cloud.points[j].z;
                
                cv::Vec3b rgb_val = image.at<cv::Vec3b>(v,u);
                
                // rgb_val needs color check
                seg_point_rgb.r = rgb_val(2);
                seg_point_rgb.g = rgb_val(1);
                seg_point_rgb.b = rgb_val(0);

                seg_output_cloud -> points.push_back(seg_point_rgb);   
            }
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