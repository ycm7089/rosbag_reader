#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <livox_ros_driver/CustomMsg.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_pcd_node");

    cout << "Hello bag_to_pcd" << endl;
    rosbag::Bag bag;
    bag.open("/home/yshin/agricultural_ws/cm_livox_data.bag", rosbag::bagmode::Read);
    
    std::vector<string> topics;
    topics.push_back("/livox/lidar");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    for (const rosbag::MessageInstance &m : view) {
        livox_ros_driver::CustomMsg::ConstPtr livox_ptr = m.instantiate<livox_ros_driver::CustomMsg>();
        livox_ros_driver::CustomMsg custom_msg = *(livox_ptr); // message type
        
        cerr << custom_msg.point_num << endl;
        
        for (uint i = 0; i < custom_msg.point_num; ++i) {
            pcl::PointXYZI p;
            p.x = custom_msg.points[i].x;
            p.y = custom_msg.points[i].y;
            p.z = custom_msg.points[i].z;
            p.intensity = custom_msg.points[i].reflectivity;
            
            if(p.x > 0.01)
                cerr << p.x << " " << p.y << " " << p.z << " " << p.intensity << endl;
            //output_cloud.points.push_back(p);
        }
    }

    return 0;
}
