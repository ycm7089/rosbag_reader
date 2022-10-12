#!/usr/bin/env python3

import rospy
import sys
import rosbag
import pcl
# import pcl_helper
import numpy as np

def example():
    pc_array = np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32)
    print(type(pc_array))
    print(pc_array)
    pc = pcl.PointCloud(pc_array)
    print(pc)
    # output is <PointCloud of 2 points>

def read_bagfile():
    bag = rosbag.Bag("/home/cm/livox_data.bag")
    count = 0
    for topic, msg, t in bag.read_messages(topics=['/livox/lidar']):
        print(type(msg.points[count].x))
        pc_array = np.array([round(msg.points[count].x, 3), round(msg.points[count].y,3),
                             round(msg.points[count].z,3)], dtype=np.float64)
        print(type(pc_array))
        pc = pcl.PointCloud()
        print(type(pc))
        print(pc_array)
        print(pc)
        
        # pc.from_array(pc_array)
        # pc = pcl_helper.XYZ_to_XYZRGB(pc,[255,255,255])
        # print(pc)
        count = count + 1
        # print(len(msg))
        
    # print(pc_array[0], pc_array[1])
    bag.close()

if __name__ == '__main__':
    # example()
    read_bagfile()
