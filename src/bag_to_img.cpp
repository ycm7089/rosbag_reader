#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_img_node");
    
    char buf[256];

    std::cout << "Read image from bag file" << std::endl;
    std::cout << "Please set image_topic_name and bagfile_name" << std::endl;
    std::string image_topic_name = "/camera/color/image_raw";
    std::string bagfile_name = "/home/cm/rosbag_file/all_light_on.bag";
    std::cout << "topic name: " << image_topic_name << std::endl;
    cv::Mat image;
    rosbag::Bag bag;
    bag.open(bagfile_name);
    int count = 0;
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        // fetch image topic name
        std::string imgTopic = m.getTopic();
        if (image_topic_name == imgTopic) {
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
            // string output_path = "/home/cm/rosbag_file/image/";
            // stringstream counting;
            // counting << "img_" << count;
            // string output = output_path + counting + ".bmp";

            // cv::imwrite(output, image);

            sprintf(buf,"/home/cm/rosbag_file/all_light_image/%d.bmp",count);       
            cv::imwrite(buf,image);
            count = count + 1;

     
            }
    }
    bag.close();
    return 0;
}