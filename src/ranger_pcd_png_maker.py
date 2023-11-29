#!/usr/bin/env python3

import numpy as np
import os
from sensor_msgs.msg import PointCloud2, Image, CameraInfo,PointCloud
import tf
import rospy
import string
import time

import pcl
import pcl_helper
import cv2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

class Convert:
    def __init__(self):
        self.cv_br = CvBridge()
        self.color_cv2_img = Image()
        self.seg_cv2_img = Image()
        self.robot_odom = Odometry()
        self.lidar_points = PointCloud2()
        self.camera_info = CameraInfo()
        # ouster lidar points
        self.lidar_sub = rospy.Subscriber("/ouster_node/ouster/points2", PointCloud2, self.lidar_cb)

        # Segmentation Information
        # self.seg_camera_sub  = rospy.Subscriber("/cm_segmentation",Image,self.seg_camera_cb)
        
        # Fast LIO Odometry
        self.odom_sub = rospy.Subscriber("/Odometry",Odometry, self.odom_cb) # odom is odom to base_link

        # RGB camera Information
        self.color_camera_sub  = rospy.Subscriber("/camera/color/image_raw",Image,self.color_camera_cb)
        self.camera_camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo,self.color_info_cb)        
        # robot_pose txt
        self.f = open("/home/kimm/new_ranger/txt/fastlio_txt.txt", 'w')

        self.listener = tf.TransformListener()
        self.cnt = 0
        self.seg_cnt = 0

    def color_info_cb(self,msg) :
        self.camera_info.header.stamp = msg.header.stamp
        
    def odom_cb(self, msg):
        
        self.robot_odom.pose.pose.position.x = msg.pose.pose.position.x
        self.robot_odom.pose.pose.position.y = msg.pose.pose.position.y
        self.robot_odom.pose.pose.position.z = msg.pose.pose.position.z
        
        self.robot_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.robot_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.robot_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.robot_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w
        # self.robot_odom.header.stamp
        # trans, rot = self.listener.lookupTransform('/map','/body',msg.header.stamp)
        # print (trans, rot)
        # if self.cnt > 0 :
        #     self.f.write("\n")
        # print(self.robot_odom)

    def lidar_cb(self, msg):
        self.lidar_points = pcl_helper.ros_to_pcl(msg)
        # print(rospy.get_rostime().secs, rospy.get_rostime().nsecs)
        # print(rospy.get_rostime())
        if self.lidar_points :
            # pcl.save(self.lidar_points,"/home/kimm/new_ranger/pcd/%s.pcd"% str(rospy.get_rostime()))
            pcl.save(self.lidar_points,"/home/kimm/new_ranger/pcd/%s.pcd"% str(self.camera_info.header.stamp))

            # pcl.save(self.lidar_points,"/home/kimm/pcd_img_data/0502_trav_pcd/%d.pcd"% self.cnt)
            # print("2")
            # self.cnt+=1

            ## robot_pose txt 
            trans, rot = self.listener.lookupTransform('/map','/body',rospy.Time(0))
            
            print (trans[0], rot)
            self.f.write(str(self.camera_info.header.stamp))
            self.f.write(" ")
            self.f.write(f'{trans[0]:0.10f}')
            self.f.write(" ")
            self.f.write(f'{trans[1]:0.10f}')
            self.f.write(" ")
            self.f.write(f'{trans[2]:0.10f}')
            self.f.write(" ")
            self.f.write(f'{rot[0]:0.10f}')
            self.f.write(" ")
            self.f.write(f'{rot[1]:0.10f}')
            self.f.write(" ")
            self.f.write(f'{rot[2]:0.10f}')
            self.f.write(" ")
            self.f.write(f'{rot[3]:0.10f}')
            self.f.write("\n")
        else :
            self.f.close()

        # print("lidar points is : ", self.lidar_points)
        
    def seg_camera_cb(self, msg):
        # print("camera cb start")
        self.seg_cv2_img = self.cv_br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        
        pcl.save(self.lidar_points,"/home/kimm/pcd_img_data/0413_pcd/%d.pcd"%self.cnt)
        cv2.imwrite("/home/kimm/pcd_img_data/0413_seg/%d.png"%self.seg_cnt, self.seg_cv2_img)
        
        trans, rot = self.listener.lookupTransform('/map','/velodyne',rospy.Time(0))
        # print (trans[0], rot)

        self.f.write(str(trans[0]))
        self.f.write(" ")
        self.f.write(str(trans[1]))
        self.f.write(" ")
        self.f.write(str(trans[2]))
        self.f.write(" ")
        self.f.write(str(rot[0]))
        self.f.write(" ")
        self.f.write(str(rot[1]))
        self.f.write(" ")
        self.f.write(str(rot[2]))
        self.f.write(" ")
        self.f.write(str(rot[3]))
        self.f.write("\n")

        self.f.close()

        self.cnt+=1
        self.seg_cnt +=1
        print("pcd, img, txt %dth saved"%self.cnt)

    def color_camera_cb(self, msg):
        # print("camera cb start")
        self.color_cv2_img = self.cv_br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        # cv2.imwrite("/home/kimm/new_ranger/img/%s.png"% str(rospy.get_rostime()), self.color_cv2_img)
        cv2.imwrite("/home/kimm/new_ranger/img/%s.png"% str(self.camera_info.header.stamp), self.color_cv2_img)

def main():
    rospy.init_node("convert_node")

    test = Convert()
    
    # test.simultaneously_convert()
    
    rospy.spin()

if __name__ == '__main__':
        main()

