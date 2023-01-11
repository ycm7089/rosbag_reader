#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, Image, PointCloud
import sensor_msgs.point_cloud2 as pc2

import rospy
import string

import pcl
import pcl_helper
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Convert:
    def __init__(self):
        self.cv_br = CvBridge()
        self.rgb_img = Image()
                
        self.lidar_sub  = rospy.Subscriber("/velodyne_points",PointCloud2,self.lidar_cb)
        self.camera_sub  = rospy.Subscriber("/camera/color/image_raw",Image,self.camera_cb)
            
    def lidar_cb(self, msg):

        self.lidar_points = pcl_helper.ros_to_pcl(msg)
        print("sss", self.lidar_points)
 
        
    def camera_cb(self, msg):
        print("s")
        self.cv2_img = self.cv_br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        
        cv2.imshow("s",self.cv2_img)
        cv2.waitKey(1)
    
    def simultaneously_convert(self):
        num_iter = 100
        for i in range(num_iter):
            input_key = input()

            if input_key == "":
                pcl.save(self.lidar_points,"/home/cm/rosbag_file/output_pcd/%d.pcd"%i)

                cv2.imwrite("/home/cm/rosbag_file/output_pcd/%d.png" %i, self.cv2_img)

def main():
    rospy.init_node("convert_node")

    test = Convert()
    test.simultaneously_convert()
    
    rospy.spin()

if __name__ == '__main__':
        main()

