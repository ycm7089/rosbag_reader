#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef Eigen::Matrix<float, 3, 4> Matrix3x4;
typedef Eigen::Matrix<float, 4, 1> Matrix4x1;

ros::Subscriber lidar_sub;
ros::Subscriber odom_sub;
ros::Subscriber camera_sub;
ros::Publisher lidar_pub;

cv::Mat sub_image;

bool is_in_img(int u, int v)
{
    if ( 0 <= u && u <640 && 0 <= v && v <480)

        return true;
    else

        return false;
}

void cm_matrix(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int data_num,cv::Mat img)
{
    Eigen::Matrix4f Extrinsic_matrix;

    Matrix3x4 KE_Matrix;

    Matrix4x1 xyz_result;

    KE_Matrix = Matrix3x4::Zero();
    xyz_result = Matrix4x1::Zero();
    xyz_result(3,0) = 1.0;

    // Extrinsic_matrix << 0.0346, -0.9933, -0.1102, 0.1134,
    //                     0.0773,  0.1126, -0.9906,-0.5178,
    //                     0.9964, 0.0257, 0.0807, -0.0737,
    //                     0,0,0,1;

    Extrinsic_matrix << 0.0, -1.0, 0.0, 0.1284,
                        0.0,  0.0,-1.0, -0.063,
                        1.0,  0.0, 0.0, -0.055,
                        0,0,0,1;

    // Extrinsic_matrix << 0.0346, -0.9933, -0.1102, 0.04,
    //                     0.0773,  0.1126, -0.9906,-0.118,
    //                     0.9964, 0.0257, 0.0807, -0.055,
    //                     0,0,0,1;

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

    cv::Mat tmp;
    img.copyTo(tmp);

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

            // cout << cloud.points[i].z << endl;
            
            // double slope = atan2( sqrt( pow(cloud.points[i].x,2)+ pow(cloud.points[i].y,2)), cloud.points[i].z) * 180 / M_PI;
            // cout << "slope_x is "<< slope_x << " slope y is " << slope_y << " estimated_slope is " << slope << endl;
            // cout << " estimated_slope is " << slope << endl;
            // if (slope < 60)
            // {
            point_rgb.z = cloud.points[i].z;
            // }
            // if (cloud.points[i].z < 0)
            // {
            //     point_rgb.z = cloud.points[i].z;
            // }

            cv::Vec3b rgb_val = img.at<cv::Vec3b>(v,u);


            point_rgb.r = rgb_val(2);
            point_rgb.g = rgb_val(1);
            point_rgb.b = rgb_val(0);
            
            // BGR
            tmp.at<cv::Vec3b>(v,u) = cv::Vec3b(cloud.points[i].b,cloud.points[i].g,cloud.points[i].r);

            output_cloud -> points.push_back(point_rgb);
        }
    }

    cv::namedWindow("tmp");
    cv::imshow("tmp", tmp);
    cv::waitKey(1);

    pcl::toROSMsg(*output_cloud, cloud_out);
    
    cloud_out.header.frame_id = "map";

    cloud_out.header.stamp = ros::Time::now();
        
    // cv::namedWindow("test");
    // cv::imshow("test", img);
    // cv::waitKey(1);

    // tf_pub(robot_odom);

    lidar_pub.publish(cloud_out);  
    output_cloud -> clear(); 
    cloud_out.data.clear();
    lidar_pub.publish(cloud_out);   
}

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    sub_image = cv_bridge::toCvShare(msg, "bgr8")-> image;
}

void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_velodyne_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // *new_velodyne_cloud는 데이터 값 가져오는거고  new_velodyne_cloud는 주소 가져오는거임!
    
    pcl::fromROSMsg(*msg,*new_velodyne_cloud);
    cv::Mat copy_image;
    sub_image.copyTo(copy_image);
    
    /* visualization
    pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
    viewer1.addPointCloud<pcl::PointXYZRGB>(new_velodyne_cloud, "velodyn_cloud");

    cv::imshow("test", copy_image);
    cv::waitKey(1);*/

    cm_matrix( *new_velodyne_cloud, new_velodyne_cloud->size(), copy_image);

    new_velodyne_cloud->clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "real_time_node");

    ros::NodeHandle nh("~");

    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("xyzrgb",1);

    // camera_sub = nh.subscribe("/camera/color/image_raw", 1000, image_cb);
    
    // For jetson
    // camera_sub = nh.subscribe("/cm_segmentation", 1000, image_cb);
    // odom_sub = nh.subscribe("/odom", 1000, odom_cb);
    camera_sub = nh.subscribe("/camera/color/image_raw", 1000, image_cb);
    // lidar_sub = nh.subscribe("/velodyne_points", 1000, lidar_cb);
    lidar_sub = nh.subscribe("/pcd_read_node/seg_pointcloud_pub",1000, lidar_cb);
    
    ros::spin();

    return 0;
}


