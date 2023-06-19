import json
import os.path as osp
import os
import PIL.Image
from PIL import Image

from labelme import utils

import cv2
import numpy as np

img_dir = '/home/kimm/colorize_json_test'

colirze_save_dir = img_dir + 'colorize/'
label_save_dir = img_dir + 'label/'

# 하나 하나 직접 하고싶을 때는 cnt를 input_key로 바꿔야 합니다!
# print("input the json's number")
# input_key = int(input())
# load_json_dir = ('/home/cm/label_pic/json/%d.json' % input_key)
    
def main() :
    cnt = 0

    while True :
        load_json_dir = (img_dir + 'json/')

        if load_json_dir :
            read_image = Image.open(img_dir + 'img/').convert("L")
            color_image = Image.open(img_dir + 'img/').convert("RGB")
            
            data = json.load(open(load_json_dir))

            # only use json file
            # imageData = data.get("imageData")
            # img = utils.img_b64_to_arr(imageData)
            
            print(load_json_dir)
            
            label_colormap = {
                'unlabeled':(0, 0, 0),
                'road':(128, 64, 128),
                'sidewalk':(244, 35, 232),
                'building':(70, 70, 70),
                'sky':(70, 130, 180),
                'person':(220, 20, 60),
                'car':(0, 0, 142),
                'vegetation':(107, 142, 35),
            }

            img = np.array(read_image)    
            colorize_img = np.array(color_image)    
            
            for shape in data["shapes"] :
                label_name = shape["label"]
                label_points = shape["points"]
                
                color_points = []
                
                for xy in label_points:

                    color_points.append(xy) 

                color_points_numpy = np.zeros((1,len(color_points),2))

                for i in range(0,len(color_points)):

                    color_points_numpy[0][i] = np.array(color_points[i])

                    if i == len(color_points)-1:

                        color_points_numpy = np.asarray(color_points_numpy,dtype=int)

                        if label_name in label_colormap:                   
                            if label_name == "unlabeled":
                                cv2.fillPoly(img, color_points_numpy, color = [0])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])
                            
                            if label_name == "road":
                                cv2.fillPoly(img, color_points_numpy, color = [1])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])

                            if label_name == "sidewalk":
                                cv2.fillPoly(img, color_points_numpy, color = [2])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])

                            if label_name == "building":
                                cv2.fillPoly(img, color_points_numpy, color = [3])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])
                                
                            if label_name == "sky":
                                cv2.fillPoly(img, color_points_numpy, color = [4])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])

                            if label_name == "person":
                                cv2.fillPoly(img, color_points_numpy, color = [5])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])

                            if label_name == "car":
                                cv2.fillPoly(img, color_points_numpy, color = [6])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])

                            if label_name == "vegetation":
                                cv2.fillPoly(img, color_points_numpy, color = [7])
                                cv2.fillPoly(color_image, color_points_numpy, color = label_colormap[label_name])
                
            PIL.Image.fromarray(img).save(osp.join(label_save_dir, "%6d.png"%cnt))   
            PIL.Image.fromarray(color_image).save(osp.join(colirze_save_dir, "%6d.png"%cnt))  
            # ss = PIL.Image.open(osp.join(label_save_dir,"%6d.png" %cnt))

            print("Save img_%d Completed" %cnt)
            cnt +=1

        else :
            break
if __name__ == '__main__':
    main()