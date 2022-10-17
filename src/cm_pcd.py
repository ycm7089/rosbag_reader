#!/usr/bin/env python3

from hashlib import new
from traceback import print_tb
import rospy
import sys
import rosbag
import pcl
import numpy as np

def example():
    pc_array = np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32)
    print(type(pc_array))
    print(pc_array)
    pc = pcl.PointCloud(pc_array)
    print(pc)
    # output is <PointCloud of 2 points>
def eee():
    arr = np.empty((0,3), int)
    arr = np.append(arr, np.array([[1, 2, 3]]), axis=0)
    for i in range(1,10) :
        arr = np.append(arr, np.array([[i, i, i]]), axis=0)
    print(arr)
    

def read_bagfile():
    bag = rosbag.Bag("/home/kimm/livox_data_right.bag")
    # livox_data_front rosbag data : 1305      livox_data_right rosbag data : 1349 
    count = 0
    new_array = np.empty((0,3))
    for topic, msg, t in bag.read_messages(topics=['/livox/lidar']):
        # print(type(msg.points[count].x))
        # if count >= 1:
            # pc_array = np.array([[msg.points[count-1].x,msg.points[count-1].y, msg.points[count-1].z],
                                # [msg.points[count].x,msg.points[count].y, msg.points[count].z]], dtype=np.float32)
        if count == 0 :
            pc_array = np.array([[msg.points[count].x,msg.points[count].y, msg.points[count].z]], dtype=np.float32)
        else :
            pc_array = np.append(pc_array, np.array([[msg.points[count].x, msg.points[count].y, msg.points[count].z]]), axis=0)

        
        count = count + 1
    print(count)
    print(pc_array)
    for i in range(0,int(count/10)) :
        if i >=1 :      
            new_array = np.array([pc_array[10*i-9], pc_array[10*i-8], pc_array[10*i-7], pc_array[10*i-6], pc_array[10*i-5],
                                 pc_array[10*i-4], pc_array[10*i-3], pc_array[10*i-2], pc_array[10*i-1], pc_array[10*i]], dtype=np.float32)
            print(new_array)
            pc = pcl.PointCloud(new_array)

            pcl.save(pc,"/home/kimm/catkin_ws/src/rosbag_reader/pcd_data/"+ str(i) + ".pcd")

        if i == count :
            pass
        # elif i == int(count)/2:
            # new_array = np.array([pc_array[2 * i-2], pc_array[2 * i-1]], dtype=np.float32)
    # print(int(count/2))
    # for i in range(1,count) :
    #     new_array = np.array(pc_array)
    # print(new_array)
    print(pc)
    # pcl.save(pc, "11.pcd")
    # print(pc_array[0], pc_array[1])
    bag.close()

if __name__ == '__main__':
    # example()
    # eee()
    read_bagfile()
