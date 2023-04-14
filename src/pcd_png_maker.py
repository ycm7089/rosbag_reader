#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, Image, PointCloud
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
        # self.lidar_points = PointCloud2()
        
        self.lidar_sub  = rospy.Subscriber("/velodyne_points",PointCloud2,self.lidar_cb)
        # self.seg_camera_sub  = rospy.Subscriber("/cm_segmentation",Image,self.seg_camera_cb)
        # self.odom_sub = rospy.Subscriber("/odom",Odometry, self.odom_cb) # odom is odom to base_link
        self.color_camera_sub  = rospy.Subscriber("/camera/color/image_raw",Image,self.color_camera_cb)

        self.f = open("/home/kimm/pcd_img_data/txt/0413_txt.txt", 'w')
        self.listener = tf.TransformListener()
        self.cnt = 0
        self.seg_cnt = 0

    def odom_cb(self, msg):
        
        self.robot_odom.pose.pose.position.x = msg.pose.pose.position.x
        self.robot_odom.pose.pose.position.y = msg.pose.pose.position.y
        self.robot_odom.pose.pose.position.z = msg.pose.pose.position.z
        
        self.robot_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.robot_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.robot_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.robot_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w
        # self.robot_odom.header.stamp
        trans, rot = self.listener.lookupTransform('/map','/velodyne',msg.header.stamp)
        print (trans, rot)
        # if self.cnt > 0 :
        #     self.f.write("\n")
        # print(self.robot_odom)

    def lidar_cb(self, msg):

        self.lidar_points = pcl_helper.ros_to_pcl(msg)
        # print(rospy.get_rostime().secs, rospy.get_rostime().nsecs)
        # print(rospy.get_rostime())
        pcl.save(self.lidar_points,"/home/kimm/pcd_img_data/pcd/%s.pcd"% str(rospy.get_rostime()))

        trans, rot = self.listener.lookupTransform('/map','/velodyne',rospy.Time(0))
        
        # print (trans[0], rot)
        self.f.write(str(rospy.get_rostime()))
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

        # self.f.close()

        self.cnt+=1
        self.seg_cnt +=1
        print("pcd, img, txt %dth saved"%self.cnt)

    def color_camera_cb(self, msg):
        print("camera cb start")
        self.color_cv2_img = self.cv_br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        cv2.imwrite("/home/kimm/pcd_img_data/rgb_img/%s.png"% str(rospy.get_rostime()), self.color_cv2_img)

        
        # cv2.imshow("s",self.cv2_img)
        # cv2.waitKey(1)
    
    # def simultaneously_convert(self):

    #     num_iter = 1000
    #     print("in save function")

    #     for i in range(num_iter):
    #         # input_key = input()
    #         print(i)
    #         time.sleep(1)
    #         # if input_key == "":
    #             # pcl.save(self.lidar_points,"/home/cm/rosbag_file/output_pcd/%d.pcd"%i)
    #             # cv2.imwrite("/home/cm/rosbag_file/output_pcd/%d.png" %i, self.cv2_img)
    #         pcl.save(self.lidar_points,"/home/kimm/pcd_img_data/new_pcd/%d.pcd"%i)
            
    #         # cv2.imwrite("/home/kimm/pcd_img_data/new_img/%d.png" %i, self.color_cv2_img)
    #         cv2.imwrite("/home/kimm/pcd_img_data/new_seg_img/%d.png"%i, self.seg_cv2_img)

    #         print("img, pcd %dth file saved" %i)

def main():
    rospy.init_node("convert_node")

    test = Convert()
    # test.simultaneously_convert()
    
    rospy.spin()

if __name__ == '__main__':
        main()
