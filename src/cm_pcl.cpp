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

typedef Eigen::Matrix<float, 3, 4> Matrix3x4;
typedef Eigen::Matrix<float, 4, 1> Matrix4x1;


void asdad()
{

}

void cm_matrix(pcl::PointCloud<pcl::PointXYZ>& cloud, int data_num) // & no copy chamjoja
{
    // cout << cloud.points[2399].x << endl;
    // realsense camera K matrix -> fx fy cx cy
    
    Eigen::Matrix3f K_matrix;

    Eigen::Matrix4f Extrinsic_matrix;
    Eigen::Matrix4f result1;

    Matrix3x4 result2;
    Matrix3x4 KE_Matrix;

    Matrix4x1 xyz_result;

    KE_Matrix = Matrix3x4::Zero();
    xyz_result = Matrix4x1::Zero();
    xyz_result(3,0) = 1.0;
    // xyz_result.ad
    // cout << xyz_result.matrix() << endl;
    // K_matrix << 601.654608, 0.000000, 326.296932, 0.000000, 603.568857, 244.054371, 0.000000, 0.000000, 1.000000; 
    Extrinsic_matrix << 0.0396624,-0.99725,-0.0626107,-0.406584,0.0162051,0.0632938,-0.997863,-0.00107753, 0.999082,0.0385631,0.0186709,-0.294709,0,0,0,1;

    float fx = 601.654608;
    float fy = 603.568857;
    float cx = 326.296932;
    float cy = 244.054371;

    cv::Mat img1(640, 480, CV_32FC3); // float 자료형 + 채널3개인 행렬

    // result1.block(0,0,3,4) = K_matrix * Extrinsic_matrix.block(0,0,3,4) ;
    // result2 =K_matrix * Extrinsic_matrix.block(0,0,3,4);
    
    // cout << result1(0,0) << " " << result1(1,0) << " " <<result1(2,0) << endl;
    // cout << Extrinsic_matrix(0,0) << " " << Extrinsic_matrix(1,0) << " " <<Extrinsic_matrix(2,0) << " "<< Extrinsic_matrix(3,0) << endl;

    // Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    // cout << transform_1.matrix() << endl;
    // Eigen::Affine3f transform_1(Extrinsic_matrix);


    for(int j = 0; j <4; j++)
    {
        KE_Matrix(0,j) = fx * Extrinsic_matrix(0,j) + cx * Extrinsic_matrix(2,j);
        KE_Matrix(1,j) = fy * Extrinsic_matrix(1,j) + cy * Extrinsic_matrix(2,j);
        KE_Matrix(2,j) = Extrinsic_matrix(2,j);
    }
    cout <<"==================================" << endl;
    for(int i = 0; i < data_num; i ++)
    {
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
        cout << u <<" " << v << endl;    
    }
    int w = 1;
    // cout << xyz_result.matrix() << endl; 
    //calculate final matrix

    
    // img1.at<Vec3f>(0,0) = float(u);
    // cout << img1.rows << " " << img1.cols << " " << img1.at<Vec3f>(0,0) << endl;

    // cout << KE_Matrix.matrix() << endl;
    cout << "" << endl;
    // cout << result1.matrix() << endl;
    // Eigen::Matrix3d res =  K_matrix * transform_1;
    // cout << "" << endl;
    // cout << K_matrix.matrix() << endl;
    // cout << "" << endl;
    // cout << transform_1.matrix() << endl;

    // cout << transform_1.translation().matrix() << endl;
    // cout << transform_1.rotation().matrix() << endl;
    
}
void topiccb(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{

    ROS_INFO("=============");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ point_xyz;
    // ROS_INFO("num %d", msg->point_num);  //2400
    int data_num = msg -> point_num;
    for (int i = 0; i < msg->point_num; i++)
    {
        point_xyz.x = msg->points[i].x;
        point_xyz.y = msg->points[i].y;
        point_xyz.z = msg->points[i].z;
        cloud.push_back(point_xyz);
    }
    cm_matrix(cloud, data_num);
    cloud.clear();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub;
    topic_sub = nh.subscribe("/livox/lidar", 1000, topiccb);

    ros::spin();
    return 0;
}