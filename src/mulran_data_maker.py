#!/usr/bin/env python3

import numpy as np
import os
import csv

from sensor_msgs.msg import PointCloud2, Image, PointCloud
from nav_msgs.msg import Odometry

import tf
import rospy
import string
import time

import pcl
import pcl_helper
import cv2
from cv_bridge import CvBridge, CvBridgeError

save_dir = '/home/kimm/cm_mulran'
lidar_save_dir = os.path.join(save_dir, 'pcd/')
camera_save_dir = os.path.join(save_dir, 'png/')

class Convert:
    def __init__(self):
        self.cv_br = CvBridge()
        self.color_cv2_img = Image()
        self.seg_cv2_img = Image()
        self.robot_odom = Odometry()

        self.csv_lst = []
        self.ck = False        
        # Fast-LIO lidar points
        # self.lidar_sub = rospy.Subscriber("/cloud_registered", PointCloud2, self.lidar_cb)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.lidar_cb)

        
        # FAST-LIO Odometry
        # self.odom_sub = rospy.Subscriber("/Odometry",Odometry, self.odom_cb) # odom is odom to base_link
        self.odom_sub = rospy.Subscriber("/ranger_base_node/odom",Odometry, self.odom_cb) # odom is odom to base_link


        # RGB camera Information
        self.color_camera_sub  = rospy.Subscriber("/camera/color/image_raw",Image,self.color_camera_cb)
    
    def odom_cb(self, msg):
        
        self.csv_lst.append([str(rospy.get_rostime()),msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
        if not self.ck :
            self.txt_file = open("/home/kimm/cm_mulran/odom_txt.txt", 'w')
            self.ck = True
        else :
        # with open(save_dir + "/" + "odom_txt.txt", 'w') as txt_file :
            self.txt_file.write(str(rospy.get_rostime()))
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.position.x:0.10f}')
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.position.y:0.10f}')
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.position.z:0.10f}')
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.orientation.x:0.10f}')
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.orientation.y:0.10f}')
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.orientation.z:0.10f}')
            self.txt_file.write(" ")
            self.txt_file.write(f'{msg.pose.pose.orientation.w:0.10f}')
            self.txt_file.write("\n")

        # self.txt_file.close()

        # csv_writer = csv.writer(self.csv_lst)

        with open(save_dir + "/" +"odom_csv.csv", 'w') as f :
            writer = csv.writer(f)


            for row in self.csv_lst :
                writer.writerow(row)

        # close 포함
        # self.csv_file.close()
 

    def lidar_cb(self, msg):
        self.lidar_points = pcl_helper.ros_to_pcl(msg)
        pcl.save(self.lidar_points,lidar_save_dir + "%s.pcd"% str(rospy.get_rostime()))

    def color_camera_cb(self, msg):
        self.color_cv2_img = self.cv_br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        cv2.imwrite(camera_save_dir + "%s.png"% str(rospy.get_rostime()), self.color_cv2_img)


def main():
    rospy.init_node("cm_mulran_node")

    test = Convert()
        
    rospy.spin()

if __name__ == '__main__':
        main()

