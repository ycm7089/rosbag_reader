// // #include <cv_bridge/cv_bridge.h>
// #include <iostream>
// #include <string>
// #include <vector>
// #include <ros/ros.h>

// #include <opencv2/highgui/highgui.hpp>

// #include <sensor_msgs/PointCloud2.h>

// // #include <livox_ros_driver/CustomMsg.h>

// #include <pcl_conversions/pcl_conversions.h>

// using namespace std;

// typedef Eigen::Matrix<float, 3, 4> Matrix3x4;
// typedef Eigen::Matrix<float, 4, 1> Matrix4x1;

// ros::Subscriber topic_sub;
// ros::Publisher lidar_pub;

// bool is_in_img(int u, int v)
// {
//     if ( 0 <= u && u <640 && 0 <= v && v <480)

//         return true;
//     else

//         return false;
// }

// void cm_matrix(pcl::PointCloud<pcl::PointXYZ>& cloud, int data_num) // & no copy chamjoja
// {
//     Eigen::Matrix4f Extrinsic_matrix;

//     Matrix3x4 KE_Matrix;

//     Matrix4x1 xyz_result;

//     KE_Matrix = Matrix3x4::Zero();
//     xyz_result = Matrix4x1::Zero();
//     xyz_result(3,0) = 1.0;

//     Extrinsic_matrix << 0.0396624,-0.99725,-0.0626107,-0.406584,0.0162051,0.0632938,-0.997863,-0.00107753, 0.999082,0.0385631,0.0186709,-0.294709,0,0,0,1;

//     float fx = 601.654608;
//     float fy = 603.568857;
//     float cx = 326.296932;
//     float cy = 244.054371;

//     cv::Mat img1= cv::imread("/home/kimm/calibration_data/data/calib/image/old/0.bmp", cv::IMREAD_COLOR);
    
//     pcl::PointXYZRGB point_rgb;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
//     sensor_msgs::PointCloud2 cloud_out;

//     for(int j = 0; j <4; j++)
//     {
//         KE_Matrix(0,j) = fx * Extrinsic_matrix(0,j) + cx * Extrinsic_matrix(2,j);
//         KE_Matrix(1,j) = fy * Extrinsic_matrix(1,j) + cy * Extrinsic_matrix(2,j);
//         KE_Matrix(2,j) = Extrinsic_matrix(2,j);
//     }

//     cout <<"==================================" << endl;

//     for(int i = 0; i < data_num; i ++)
//     {
//         xyz_result[0] = KE_Matrix(0,0) * cloud.points[i].x +
//                         KE_Matrix(0,1) * cloud.points[i].y +
//                         KE_Matrix(0,2) * cloud.points[i].z +
//                         KE_Matrix(0,3) * 1;

//         xyz_result[1] = KE_Matrix(1,0) * cloud.points[i].x +
//                         KE_Matrix(1,1) * cloud.points[i].y +
//                         KE_Matrix(1,2) * cloud.points[i].z +
//                         KE_Matrix(1,3) * 1;

//         xyz_result[2] = KE_Matrix(2,0) * cloud.points[i].x +
//                         KE_Matrix(2,1) * cloud.points[i].y +
//                         KE_Matrix(2,2) * cloud.points[i].z +
//                         KE_Matrix(2,3) * 1;

//         int u = int(xyz_result[0] / xyz_result[2]);
//         int v = int(xyz_result[1] / xyz_result[2]);
//         int w = 1;

//         if (is_in_img(u, v))
//         {
//             point_rgb.x = cloud.points[i].x;
//             point_rgb.y = cloud.points[i].y;
//             point_rgb.z = cloud.points[i].z;

//             cv::Vec3b rgb_val = img1.at<cv::Vec3b>(v,u);
            
//             // extract lidar pointcloud in image
//             // cv::Vec3b tmp_val(0, 0, 0);
//             // img1.at<cv::Vec3b>(v,u) = tmp_val;

//             point_rgb.r = rgb_val(0);
//             point_rgb.g = rgb_val(1);
//             point_rgb.b = rgb_val(2);

//             output_cloud -> points.push_back(point_rgb);
//         }
//     }

//     pcl::toROSMsg(*output_cloud, cloud_out);
    
//     cloud_out.header.frame_id = "map";
//     cloud_out.header.stamp = ros::Time::now();
        
//     cv::namedWindow("test");
//     cv::imshow("test", img1);
//     cv::waitKey(1);

//     lidar_pub.publish(cloud_out);   
// }

// void topiccb(const livox_ros_driver::CustomMsg::ConstPtr& msg)
// {
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::PointXYZ point_xyz;

//     int data_num = msg -> point_num;

//     for (int i = 0; i < msg->point_num; i++)
//     {
//         point_xyz.x = msg->points[i].x;
//         point_xyz.y = msg->points[i].y;
//         point_xyz.z = msg->points[i].z;

//         cloud.push_back(point_xyz);
//     }

//     cm_matrix(cloud, data_num);

//     cloud.clear();
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pcl_node");

//     ros::NodeHandle nh("~");

//     lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("xyzrgb",1);
//     topic_sub = nh.subscribe("/livox/lidar", 1000, topiccb);
        
//     ros::spin();

//     return 0;
// }