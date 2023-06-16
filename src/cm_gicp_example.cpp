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
        
        pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/kimm/pcd_img_data/room_scan1.pcd", *src); //* load the file
        
        // 앞으로 10m 전진 시킨 tgt를 만듬
        Eigen::Matrix4f tf;
        tf << 1, 0, 0, 10.0,
                0, 1, 0, 0.0,
                0, 0, 1, 0.0,
                0, 0, 0, 1.0;
        
        pcl::transformPointCloud(*src,*tgt,tf);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
        gicp.setMaxCorrespondenceDistance(0.1);
        gicp.setTransformationEpsilon(0.001);
        gicp.setMaximumIterations(3000);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr align(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 걸리는 시간 측정
        chrono::system_clock::time_point t_start = chrono::system_clock::now();

        // Registration 시행
        gicp.setInputSource(src);
        gicp.setInputTarget(tgt);
        gicp.align(*align);

        chrono::system_clock::time_point t_end = chrono::system_clock::now();
        /*******************************************/
        chrono::duration<double> t_reg = t_end - t_start;
        cout<<"Takes "<<t_reg.count()<<" sec..."<<endl;

        // Ouputs
        Eigen::Matrix4f src2tgt = gicp.getFinalTransformation();
        double score = gicp.getFitnessScore();
        bool is_converged = gicp.hasConverged(); // 수렴여부

        cout << src2tgt << endl;
        cout << score << endl;
        cout << is_converged << endl;

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
