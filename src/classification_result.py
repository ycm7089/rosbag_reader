from sklearn.metrics import  accuracy_score, f1_score, precision_recall_curve, recall_score, precision_score, roc_curve, confusion_matrix
from sklearn import metrics
import numpy as np
from PIL import Image
import cv2
import PIL.Image
import os.path as osp
import matplotlib.pyplot as plt

img_save_dir = '/home/kimm/map_save/'
label_save_dir = img_save_dir + 'label/'



def main():
    label_image = Image.open("/home/kimm/map_save/label/2.png").convert("L")
    elevation_image = Image.open("/home/kimm/map_save/elevation_binary.png").convert("L")
    
    label_img = np.array(label_image)    
    elevation_img = np.array(elevation_image)
    ## for visualization marking X
    check_img = np.zeros(label_image.size)
    
    label_acc_list = []
    ele_acc_list = []

    for i in range(2667):
        for j in range(2667):
            if elevation_img[j][i] == 255 :
                elevation_img[j][i] = 1
            else :
                elevation_img[j][i] = 0

            if elevation_img[j][i] != 1 and elevation_img[j][i] != 0 :
                print("elevation has another number except 0 and 1 : %f" %elevation_img[j][i])
            # if elevation_img[i][j] == 1 and label_img[i,j] == 1:
                # Traversable

            if label_img[j][i] != 1 and label_img[j][i] != 0:
                print("label has another number except 0 and 1 : %f" %label_img[j][i])

                # circle
                # check_img = cv2.circle(check_img,(i,j),1,(255,0,0))
                
                # x
                check_img = cv2.drawMarker(check_img,(j,i),color=(255,0,0),markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
            label_acc_list.append(label_img[j][i].tolist())
            ele_acc_list.append(elevation_img[j][i].tolist())
    check_img = cv2.resize(check_img, dsize=(640, 640))
    
    print(check_img.shape)
    cv2.imshow("circle", check_img)
    cv2.waitKey(0)

    # a = confusion_matrix(label_acc_list,ele_acc_list).ravel()
    tn, fp, fn, tp = confusion_matrix(label_acc_list, ele_acc_list).ravel()
    print(tn, fp, fn ,tp)
    # plt.matshow(a)
    # plt.title('Confusion Matrix Plot')
    # plt.colorbar()
    # plt.xlabel('Precited')
    # plt.ylabel('Actual')
    # plt.show()

    acc_score =accuracy_score(label_acc_list, ele_acc_list)# , normalize=False) # result is 7053677
    print(acc_score)

    prec_score = precision_score(label_acc_list, ele_acc_list)#,average=None)
    print(prec_score)

    # recall_score = metrics.recall_score(label_img, elevation_img)
    
    f1_score = metrics.f1_score(label_acc_list, ele_acc_list)#, pos_label='positive', average='macro')
    print(f1_score)
    # fbeta_score0 = metrics.fbeta_score(label_img, elevation_img, beta=0.5)
    
    # fbeta_score1 = metrics.fbeta_score(label_img, elevation_img, beta=1)
    
    # fbeta_score2 = metrics.fbeta_score(label_img, elevation_img, beta=2)
    
    # pre_recall_fscore_support = metrics.precision_recall_fscore_support(label_img, elevation_img, beta=0.5)
    # print(acc_score, f1_score)#,fbeta_score0,fbeta_score1,fbeta_score2)
    


if __name__ == '__main__':
    main()