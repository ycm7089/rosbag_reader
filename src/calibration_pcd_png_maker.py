#!/usr/bin/env python3

import numpy as np
import os
from sensor_msgs.msg import PointCloud2, Image, PointCloud
import tf
import rospy
import string
import time

import pcl
import pcl_helper
import cv2
from cv_bridge import CvBridge

save_dir = '/home/kimm/lidar_camera_calibration_data/1101'
lidar_save_dir = os.path.join(save_dir, 'pcd')
camera_save_dir = os.path.join(save_dir, 'png')

class Convert:
    def __init__(self):
        self.cv_br = CvBridge()
        self.color_cv2_img = Image()
        
        # ouster lidar points
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_cb)

        # RGB camera Information
        self.color_camera_sub  = rospy.Subscriber("/camera/color/image_raw",Image,self.color_camera_cb)

    def lidar_cb(self, msg):
        self.lidar_points = pcl_helper.ros_to_pcl(msg)
     
    def color_camera_cb(self, msg):
        self.color_cv2_img = self.cv_br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        
    def simultaneously_convert(self):

        num_iter = 15
        print("in save function")

        for i in range(num_iter):
            input_key = input()
            if input_key == "":
                pcl.save(self.lidar_points,os.path.join(lidar_save_dir,"%d.pcd"%i))
                cv2.imwrite(os.path.join(camera_save_dir,"%d.png" %i), self.color_cv2_img)
            print("img, pcd %dth file saved" %i)

def main():
    rospy.init_node("convert_node")

    test = Convert()
    
    test.simultaneously_convert()
    
    rospy.spin()

if __name__ == '__main__':
        main()

