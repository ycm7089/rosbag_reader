import json
import os.path as osp

import PIL.Image

from PIL import Image
import matplotlib.pylab as plt

from labelme import utils

import cv2
import numpy as np

num_json = 1

img_save_dir = '/home/kimm/map_save/'
label_save_dir = img_save_dir + 'label/'

# 하나 하나 직접 하고싶을 때는 cnt를 input_key로 바꿔야 합니다!
# print("input the json's number")
# input_key = int(input())
# load_json_dir = ('/home/cm/label_pic/json/%d.json' % input_key)

# def encode_target(cls, target):
#     return cls.id_to_t

def black_image(input_image):
    for i in range(0,input_image.size[0]):
        for j in range(0, input_image.size[1]):
            # rgb = input_image.getpixel((i,j))
            input_image.putpixel((i,j),(4))
    return input_image



def main() :
   
    load_json_dir = ('/home/kimm/map_save/test.json')
    read_image = Image.open("/home/kimm/map_save/for_labeling.png").convert("L")
    
    data = json.load(open(load_json_dir))

    # only use json file
    # imageData = data.get("imageData")
    # img = utils.img_b64_to_arr(imageData)
    
    print(load_json_dir)
    
    label_colormap = {
        'traversable' : (0,0,0,128), 'nontraversable' : (255,255,255,128)
    }

    # black_image(input_image=read_image)

    img = np.array(read_image)        
    
    # for my labeling json file
    # for shape in sorted(data["shapes"], key=lambda x: x["label"]):
    for shape in data["shapes"] :
        label_name = shape["label"]
        label_points = shape["points"]

        print(label_name)
        
        color_points = []
        
        for xy in label_points:

            color_points.append(xy) 

        color_points_numpy = np.zeros((1,len(color_points),2))

        for i in range(0,len(color_points)):

            color_points_numpy[0][i] = np.array(color_points[i])

            if i == len(color_points)-1:

                color_points_numpy = np.asarray(color_points_numpy,dtype=int)

                if label_name in label_colormap:
                    # colorize img for rgb
                    # Code 상에서는 COLOR B G R 순이지만 실제로는 RGB순이라서 code안에서 RGB순으로 써줘야함 imshow에서 보이는 색은 상관이 없음!   
                    
                    # colorize img for Long type img (label)
                    if label_name == "traversable":
                        a = cv2.fillPoly(img, color_points_numpy, color = [1])
                    
                    if label_name == "nontraversable":
                        a = cv2.fillPoly(img, color_points_numpy, color = [0])

        
    PIL.Image.fromarray(img).save(osp.join(label_save_dir, "2.png"))
    
    ss = PIL.Image.open(osp.join(label_save_dir,"2.png"))
    print("Save img_0 Completed")

if __name__ == '__main__':
    main()