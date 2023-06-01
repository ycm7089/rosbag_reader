#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <chrono>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cm_gicp_node");
    ros::NodeHandle nh ("~");
    while (ros::ok())
    {
        ros::Publisher bef_trans_gicp_pub = nh.advertise<sensor_msgs::PointCloud2>("bef_trans_gicp",1);
        ros::Publisher trans_gicp_pub = nh.advertise<sensor_msgs::PointCloud2>("trans_gicp",1);
        ros::Publisher align_gicp_pub = nh.advertise<sensor_msgs::PointCloud2>("align_gicp",1);

        sensor_msgs::PointCloud2 bef_trans_gicp;
        sensor_msgs::PointCloud2 trans_gicp;
        sensor_msgs::PointCloud2 align_gicp;

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/kimm/pcd_img_data/resaved_pcd/0.pcd", *src); //* load the file
        

        Eigen::Matrix4f tf;
        tf << 1, 0, 0, 2.0,
                0, 1, 0, 0.0,
                0, 0, 1, 0.0,
                0, 0, 0, 1.0;
        
        pcl::transformPointCloud(*src,*tgt,tf);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
        gicp.setMaxCorrespondenceDistance(1.0);
        gicp.setTransformationEpsilon(0.001);
        gicp.setMaximumIterations(1000);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr align(new pcl::PointCloud<pcl::PointXYZRGB>);

        gicp.setInputSource(src);
        gicp.setInputTarget(tgt);
        gicp.align(*align);

        Eigen::Matrix4f src2tgt = gicp.getFinalTransformation();
        double score = gicp.getFitnessScore();

        cout << src2tgt << endl;
        cout << score << endl;


        pcl::toROSMsg(*tgt, trans_gicp);
            
        trans_gicp.header.frame_id = "map";
        trans_gicp.header.stamp = ros::Time::now();

        pcl::toROSMsg(*src, bef_trans_gicp);

        bef_trans_gicp.header.frame_id = "map";
        bef_trans_gicp.header.stamp = ros::Time::now();

        pcl::toROSMsg(*align, align_gicp);
        align_gicp.header.frame_id = "map";
        align_gicp.header.stamp = ros::Time::now();
        
        bef_trans_gicp_pub.publish(bef_trans_gicp);
        trans_gicp_pub.publish(trans_gicp);
        align_gicp_pub.publish(align_gicp);

        ros::spinOnce();
    }
    return 0;
}
