#!/usr/bin/env python3

import os

import tf
import rospy
import string
import time

import shutil

data_num = 5665

def main():
    rospy.init_node("pcd_png_txt_convert_node")

    pcd_path = '/home/kimm/pcd_img_data/pcd/'
    rgb_path = '/home/kimm/pcd_img_data/rgb_img/'

    rostime = []
    pose_x = []
    pose_y = []
    pose_z = []
    orient_x = []
    orient_y = []
    orient_z = []
    orient_w = [] 
    
    with open("/home/kimm/pcd_img_data/txt/0413_txt.txt") as ff:
        for line in ff:
            ttime,x,y,z,xx,yy,zz,ww = line.split()
            ttime = str(ttime)
            x = str(x)
            y = str(y)
            z = str(z)
            xx =str(xx)
            yy =str(yy)
            zz =str(zz)
            ww =str(ww)

            rostime.append(ttime)
            pose_x.append(x)
            pose_y.append(y)
            pose_z.append(z)

            orient_x.append(xx)
            orient_y.append(yy)
            orient_z.append(zz)
            orient_w.append(ww)

    pcd_flist = os.listdir(pcd_path)
    rgb_flist = os.listdir(rgb_path)

    pcd_flist = sorted(pcd_flist) # LiDAR data name is timestamp
    rgb_flist = sorted(rgb_flist) # Camera data name is timestamp

    split_pcd_flist = []
    split_rgb_flist = []
    
    re_f = open("/home/kimm/pcd_img_data/txt/re_0413_txt.txt", 'w')
    
    for i in range(len(pcd_flist)):

        split_pcd_flist.append(pcd_flist[i][0:19])
              
        min_sub = 99999999999999999999999999999999
        txt_min_sub = 99999999999999999999999999999999
        min_idx = -1
        txt_min_idx = -1

        for j in range(len(rgb_flist)):  
            split_rgb_flist.append(rgb_flist[j][0:19])

            if abs(float(split_rgb_flist[j])-float(split_pcd_flist[i])) < min_sub :
                min_sub = abs(float(split_rgb_flist[j])-float(split_pcd_flist[i]))
                min_idx = j
        
        for idx in range(len(rostime)):
            if abs(float(rostime[idx])-float(split_pcd_flist[i])) < txt_min_sub :
                txt_min_sub = abs(float(rostime[idx])-float(split_pcd_flist[i]))
                txt_min_idx = idx

        re_f.write(str(pose_x[txt_min_idx]))
        re_f.write(" ")
        re_f.write(str(pose_y[txt_min_idx]))
        re_f.write(" ")
        re_f.write(str(pose_z[txt_min_idx]))
        re_f.write(" ")
        re_f.write(str(orient_x[txt_min_idx]))
        re_f.write(" ")
        re_f.write(str(orient_y[txt_min_idx]))
        re_f.write(" ")
        re_f.write(str(orient_z[txt_min_idx]))
        re_f.write(" ")
        re_f.write(str(orient_w[txt_min_idx]))
        re_f.write("\n")
        
        src1 = pcd_path + str(split_pcd_flist[i]) + '.pcd' 
        src2 = rgb_path + str(split_rgb_flist[min_idx]) + '.png'
        
        dst1 = '/home/kimm/pcd_img_data/resaved_pcd/' + '%d.pcd' % i
        dst2 = '/home/kimm/pcd_img_data/resaved_img/' + '%d.png' % i

        shutil.copy2(src1,dst1)
        shutil.copy2(src2,dst2)

        print("=="*30)

    re_f.close()

if __name__ == '__main__':
        main()