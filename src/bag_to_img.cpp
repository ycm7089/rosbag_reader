#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <vector>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <time.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_img_node");
    
    char buf[256];

    auto start = std::chrono::system_clock::now();

    cout << "Read image from bag file" << endl;
    cout << "Please set image_topic_name and bagfile_name" << endl;

    string image_topic_name = "/camera/color/image_raw";
    string pcl_topic_name = "/velodyne_points";

    string bagfile_name = "/home/cm/rosbag_file/no_light.bag";
    cout << "topic name: " << image_topic_name << " , " << pcl_topic_name << endl;

    cv::Mat image;    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/cm/rosbag_file/output_pcd/no_light/no_light.pcd", *cloud) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //     return (-1);
    // }
    // std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;
    // for (const auto& point: *cloud)
    //     std::cout << "    " << point.x
    //             << " "    << point.y
    //             << " "    << point.z << std::endl;

    rosbag::Bag bag;
    bag.open(bagfile_name);
    int count = 0;

    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        // fetch image topic name
        
        auto middle_time = std::chrono::system_clock::now();
     
        std::chrono::duration<double> elapsed_seconds = middle_time - start;

        std::string rosbag_topic = m.getTopic();
        if (image_topic_name == rosbag_topic) {
            try {
            sensor_msgs::ImageConstPtr imgMsgPtr = m.instantiate<sensor_msgs::Image>();
            image = cv_bridge::toCvCopy(imgMsgPtr)->image;


            } 
            catch (cv_bridge::Exception& e) {
            ROS_ERROR("Image convert error");
            }
            // cv::circle(image, cv::Point(image.cols/ 2, image.rows/ 2), 10, cv::Scalar(255, 0, 255), 3);
            
            cv::imshow("image", image);
            cv::waitKey(1);

            // if (elapsed_seconds.count() > 0.0005){
            //     cout << "WOW!" << endl;
            //     break;
            // }

            sprintf(buf,"/home/cm/rosbag_file/new/%d.png",count);       
            cv::imwrite(buf,image);
            count = count + 1;
     
            }

        std::cout << "Elapsed Time: " << elapsed_seconds.count() << " sec" << std::endl;

    }
    bag.close();
    return 0;
}