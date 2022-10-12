#!/usr/bin/env python3

from hashlib import new
from traceback import print_tb
import rospy
import sys
import rosbag
#import pcl
import numpy as np
import open3d as o3d

def read_bagfile():
    bag = rosbag.Bag("/home/yshin/agricultural_ws/cm_livox_data.bag")
    count = 0
    
    pc_array = np.empty((0,3))
    
    for topic, msg, t in bag.read_messages(topics=['/livox/lidar']):
        print(count)
        count = count + 1
        
        #print(msg.point_num)
        
        for i in range(msg.point_num):
            pt_array = np.array([msg.points[i].x,msg.points[i].y, msg.points[i].z], dtype=np.float32)
            
            
            if msg.points[i].x > 0.01:       
                pc_array = np.vstack([pc_array, pt_array]) 
                #print(pt_array)
                
        if count == 315:
            break;
                
        #print(pc_array.shape)
        
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(pc_array)
    
    o3d.visualization.draw_geometries([o3d_pc],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
        # if count >= 1:
            # pc_array = np.array([[msg.points[count-1].x,msg.points[count-1].y, msg.points[count-1].z],
                                # [msg.points[count].x,msg.points[count].y, msg.points[count].z]], dtype=np.float32)
        #if count == 0 :
        #    pc_array = np.array([[msg.points[count].x,msg.points[count].y, msg.points[count].z]], dtype=np.float32)
        #else :
        #    pc_array = np.append(pc_array, np.array([[msg.points[count].x, msg.points[count].y, msg.points[count].z]]), axis=0)
            
            
        # if count <3 :
        #     print(pc_array)
        # if count >1 :
           
        # print(pc_array)
        # print(pc)
        # pcl.save(pc,"/home/kimm/pcd_data/"+ str(count) + ".pcd")
        
        #count = count + 1
        # print(len(msg))

    #for i in range(0,int(count/2)) :
        # 0 ~ 2335  count is 2336
    #    if i >=1 :      
    #        new_array = np.array([pc_array[2 * i-1], pc_array[2 * i]], dtype=np.float32)
    #        print(new_array)
    #    elif i == int(count)/2:
    #        new_array = np.array([pc_array[2 * i-2], pc_array[2 * i-1]], dtype=np.float32)
    #print(int(count/2))
    # for i in range(1,count) :
    #     new_array = np.array(pc_array)
    #print(new_array)
    #pc = pcl.PointCloud(new_array)
    #print(pc)
    # pcl.save(pc, "11.pcd")
    # print(pc_array[0], pc_array[1])
    bag.close()

if __name__ == '__main__':
    # example()
    # eee()
    read_bagfile()
