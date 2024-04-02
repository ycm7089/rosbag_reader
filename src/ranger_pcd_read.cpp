#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <unistd.h>

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
    ros::init(argc, argv, "ranger_pcd_read_node");
    ros::NodeHandle nh ("~");

    ros::Publisher PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_pub",1);
    ros::Publisher seg_PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("seg_pointcloud_pub",1);

    image_transport::ImageTransport it(nh);
    // image_transport::Publisher seg_image_pub = it.advertise("seg_img",1);
    image_transport::Publisher rgb_image_pub = it.advertise("rgb_img",1);

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

    Extrinsic_matrix << 
    -0.00318,  -0.99992,  -0.01222,  0.04439,   
    0.01545,   0.01217,   -0.99981,  -0.03315,  
    0.99988,   -0.00336,  0.01541,   -0.09533,  
    0.00000,   0.00000,   0.00000,   1.00000;

    // Extrinsic_matrix << 
    // 0.0,  -1.0,  0.0,  0.04439,   
    // 0.0,   0.0,   -1.0,  -0.03315,  
    // 1.0,   0.0,  0.0,   -0.09533,  
    // 0.00000,   0.00000,   0.00000,   1.00000;

    float fx = 606.682373046875;
    float fy = 605.8425903320312;
    float cx = 320.0584411621094;
    float cy = 240.04714965820312;
    
    for(int j = 0; j <4; j++)
    {
        KE_Matrix(0,j) = fx * Extrinsic_matrix(0,j) + cx * Extrinsic_matrix(2,j);
        KE_Matrix(1,j) = fy * Extrinsic_matrix(1,j) + cy * Extrinsic_matrix(2,j);
        KE_Matrix(2,j) = Extrinsic_matrix(2,j);
    }

    int num_pcd = 4419;
    // int num_pcd = 116;

    FILE* file;
    file = fopen("/home/kimm/new_ranger5/txt/new_fastlio_txt.txt","r");
    
    string rostime;
    float pose_x,pose_y,pose_z,orien_x,orien_y,orien_z,orien_w;
    
    for(int i = 0; i < num_pcd; i ++)
    {
        // pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/kimm/new_ranger5/for_rviz_pcd/" + std::to_string(i) + ".pcd", cloud); //* load the file
        pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/kimm/new_ranger5/for_rviz_pcd/" + std::to_string(i) + ".pcd", cloud); //* load the file

        // sleep(0.1);
        cv::Mat image = cv::imread("/home/kimm/new_ranger5/for_rviz_segimg/" + std::to_string(i) + ".png",cv::IMREAD_COLOR);
        cv::Mat rgb_image = cv::imread("/home/kimm/new_ranger5/for_rviz_rgbimg/" + std::to_string(i) + ".png",cv::IMREAD_COLOR);

        // sleep(0.1);
        fscanf(file,"%f %f %f %f %f %f %f", &pose_x, &pose_y, &pose_z, &orien_x, &orien_y, &orien_z, &orien_w);
        // sleep(0.1);
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

        tf::Quaternion kimm_q(orien_x, orien_y, orien_z, orien_w);

        tf::Matrix3x3 ms(kimm_q);
        double roll, pitch, yaw;
        ms.getRPY(roll,pitch,yaw);
        
        tf::Transform transform;

        transform.setOrigin( tf::Vector3(pose_x,pose_y, pose_z) );
        transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "body"));
                
        odom.header.frame_id = "map";
        odom.header.stamp = ros::Time::now();

        odom_pub.publish(odom);
        
        cv::Mat tmp;
        image.copyTo(tmp);
        
        for(int j = 0; j < cloud.size(); j ++)
        {
            // cout << cloud.size() << endl;
            xyz_result = Matrix4x1::Zero();
            xyz_result(3,0) = 1.0;
            
            if (cloud.points[j].x < 0.0) continue; 
            
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
                
                if(cloud.points[j].x > 50.0)  continue;
                point_rgb.x = cloud.points[j].x;
                point_rgb.y = cloud.points[j].y;
                point_rgb.z = cloud.points[j].z;

                cv::Vec3b rgb_val = image.at<cv::Vec3b>(v,u);

                seg_point_rgb.r = rgb_val(2);
                seg_point_rgb.g = rgb_val(1);
                seg_point_rgb.b = rgb_val(0);
                
                point_rgb.r = rgb_val(2);
                point_rgb.g = rgb_val(1);
                point_rgb.b = rgb_val(0);
                // }
                tmp.at<cv::Vec3b>(v,u) = cv::Vec3b(255.0,255.0,255.0);

                seg_output_cloud -> points.push_back(seg_point_rgb);                               
                output_cloud -> points.push_back(point_rgb);
            }
           
        }

        // pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
        // viewer1.addPointCloud<pcl::PointXYZRGB>(seg_output_cloud, "src_red");
        
        // pcl::visualization::CloudViewer viewer2("Cloud Viewer");
        // viewer2.showCloud(seg_output_cloud, "src_red");
        
        // // viewer1.spinOnce();

        // cv::namedWindow("tmp");
        // cv::imshow("tmp", tmp);
        // cv::waitKey(0);
        pcl::toROSMsg(*seg_output_cloud, seg_cloud_out);
        
        seg_cloud_out.header.frame_id = "body";
        seg_cloud_out.header.stamp = ros::Time::now();
        seg_PointCloud_pub.publish(seg_cloud_out);
        
        seg_cloud_out.data.clear();
        
        seg_output_cloud -> clear();
        
        pcl::toROSMsg(*output_cloud, cloud_out);
        
        cloud_out.header.frame_id = "body";
        cloud_out.header.stamp = ros::Time::now();
        
        PointCloud_pub.publish(cloud_out);

        // sensor_msgs::ImagePtr segimg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        // seg_image_pub.publish(segimg);
        sensor_msgs::ImagePtr rgbimg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",rgb_image).toImageMsg();
        rgb_image_pub.publish(rgbimg);

        output_cloud -> clear();

        cloud_out.data.clear();
        // std::cout << i << "th publish complete" << std::endl;
    }
    ros::spinOnce();

    return 0;
}