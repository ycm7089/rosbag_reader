#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
ros::Publisher odom_pub;

nav_msgs::Odometry odom;
cv::Mat sub_image;

bool is_in_img(int u, int v)
{
    if ( 0 <= u && u <640 && 0 <= v && v <480)

        return true;
    else

        return false;
}

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    sub_image = cv_bridge::toCvShare(msg, "bgr8")-> image;
}

void cm_matrix(pcl::PointCloud<pcl::PointXYZI>& cloud, int data_num,cv::Mat img, nav_msgs::Odometry odometry)
{
    Eigen::Matrix4f Extrinsic_matrix;

    Matrix3x4 KE_Matrix;

    Matrix4x1 xyz_result;

    KE_Matrix = Matrix3x4::Zero();
    xyz_result = Matrix4x1::Zero();
    xyz_result(3,0) = 1.0;

    Extrinsic_matrix << 
    -0.00318,  -0.99992,  -0.01222,  0.04439,   
    0.01545,   0.01217,   -0.99981,  -0.03315,  
    0.99988,   -0.00336,  0.01541,   -0.09533,  
    0.00000,   0.00000,   0.00000,   1.00000;
    
    float fx = 603.5733;
    float fy = 603.8386;
    float cx = 316.1940;
    float cy = 246.2663;
    
    pcl::PointXYZRGB point_rgb;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    sensor_msgs::PointCloud2 cloud_out;


    for(int j = 0; j <4; j++)
    {
        KE_Matrix(0,j) = fx * Extrinsic_matrix(0,j) + cx * Extrinsic_matrix(2,j);
        KE_Matrix(1,j) = fy * Extrinsic_matrix(1,j) + cy * Extrinsic_matrix(2,j);
        KE_Matrix(2,j) = Extrinsic_matrix(2,j);
    }

    cout <<"==================================" << endl;
    // cv::Mat tmp;
    // img.copyTo(tmp);

    for(int i = 0; i < data_num; i ++)
    {
        if (cloud.points[i].x < 0.25)   continue; 

        xyz_result[0] = KE_Matrix(0,0) * cloud.points[i].x +
                        KE_Matrix(0,1) * cloud.points[i].y +
                        KE_Matrix(0,2) * cloud.points[i].z +
                        KE_Matrix(0,3) * 1;

        xyz_result[1] = KE_Matrix(1,0) * cloud.points[i].x +
                        KE_Matrix(1,1) * cloud.points[i].y +
                        KE_Matrix(1,2) * cloud.points[i].z +
                        KE_Matrix(1,3) * 1;

        xyz_result[2] = KE_Matrix(2,0) * cloud.points[i].x +
                        KE_Matrix(2,1) * cloud.points[i].y +
                        KE_Matrix(2,2) * cloud.points[i].z +
                        KE_Matrix(2,3) * 1;

        int u = int(xyz_result[0] / xyz_result[2]);
        int v = int(xyz_result[1] / xyz_result[2]);
        int w = 1;

        if (is_in_img(u, v))
        {
            point_rgb.x = cloud.points[i].x;
            point_rgb.y = cloud.points[i].y;
            point_rgb.z = cloud.points[i].z;

            cv::Vec3b rgb_val = img.at<cv::Vec3b>(v,u);

            point_rgb.r = rgb_val(2);
            point_rgb.g = rgb_val(1);
            point_rgb.b = rgb_val(0);

            // tmp.at<cv::Vec3b>(v,u) = cv::Vec3b(cloud.points[i].intensity);
            // tmp.at<cv::Vec3b>(v,u) = cv::Vec3b(point_rgb.x,point_rgb.y,point_rgb.z);

            output_cloud -> points.push_back(point_rgb);
        }
    }
    // cv::namedWindow("tmp");
    // cv::imshow("tmp", tmp);
    // cv::waitKey(1);
    pcl::toROSMsg(*output_cloud, cloud_out);
    
    cloud_out.header.frame_id = "body";
    // cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.stamp = odometry.header.stamp;


    lidar_pub.publish(cloud_out);  
    output_cloud -> clear(); 
    cloud_out.data.clear();
    lidar_pub.publish(cloud_out);   
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom.pose.pose.position.x  = msg -> pose.pose.position.x;
    odom.pose.pose.position.y  = msg -> pose.pose.position.y;
    odom.pose.pose.position.z  = msg -> pose.pose.position.z;

    odom.pose.pose.orientation.x  = msg -> pose.pose.orientation.x;
    odom.pose.pose.orientation.y  = msg -> pose.pose.orientation.y;
    odom.pose.pose.orientation.z  = msg -> pose.pose.orientation.z;
    odom.pose.pose.orientation.w  = msg -> pose.pose.orientation.w;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    // transform.setOrigin(tf::Vector3(msg -> pose.pose.position.x,msg -> pose.pose.position.y,msg -> pose.pose.position.z));
    // transform.setRotation(tf::Quaternion(msg -> pose.pose.orientation.x,msg -> pose.pose.orientation.y,msg -> pose.pose.orientation.z,msg -> pose.pose.orientation.w));
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "os_sensor"));


}
void ouster_cb(const sensor_msgs::PointCloud2::ConstPtr& msg )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_ouster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*new_ouster_cloud);
    cv::Mat copy_image;
    sub_image.copyTo(copy_image);

    nav_msgs::Odometry cm_odom;

    cm_odom.pose.pose.position.x = odom.pose.pose.position.x;
    cm_odom.pose.pose.position.y = odom.pose.pose.position.y;
    cm_odom.pose.pose.position.z = odom.pose.pose.position.z;
    cm_odom.pose.pose.orientation.x = odom.pose.pose.orientation.x;
    cm_odom.pose.pose.orientation.y = odom.pose.pose.orientation.y;
    cm_odom.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    cm_odom.pose.pose.orientation.w = odom.pose.pose.orientation.w;
    
    cm_matrix( *new_ouster_cloud, new_ouster_cloud->size(), copy_image, cm_odom );
    new_ouster_cloud->clear();


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ouster_node");

    ros::NodeHandle nh("~");

    odom_sub = nh.subscribe("/Odometry", 1000, odom_cb);
    // ouster_sub = nh.subscribe("/cloud_registered", 1000, ouster_cb);
    // odom_sub = nh.subscribe("/ranger_base_node/odom", 1000, odom_cb);
    ouster_sub = nh.subscribe("/ouster/points", 1000, ouster_cb);
    camera_sub = nh.subscribe("/camera/color/image_raw", 1000, image_cb);
    
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("ouster/points2",1);
    // odom_pub = nh.advertise<nav_msgs::Odometry>("ranger_odom",1);
    // ros::Timer timer1 = nh.createTimer(ros::Duration(0.1),cb);

    ros::spin();
    return 0;
}