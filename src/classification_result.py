from sklearn.metrics import  accuracy_score, f1_score, precision_recall_curve, recall_score, precision_score, roc_curve, confusion_matrix
from sklearn import metrics
import numpy as np
from PIL import Image
import cv2
import PIL.Image
import os.path as osp
import matplotlib.pyplot as plt

def main():
    # ground truth
    label_image = Image.open("/home/kimm/map_save/label/ground_truth.png").convert("L")

    # lidar+camera
    # elevation_image = Image.open("/home/kimm/map_save/elevation_binary.png").convert("L")
    total_prob_image = Image.open("/home/kimm/map_save/total_prob.png").convert("L")

    # lego loam (Lidar)
    lego_loam_image = Image.open("/home/kimm/map_save/legoloam_elevation.png").convert("L")
    
    label_img = np.array(label_image)    
    # elevation_img = np.array(elevation_image)
    total_prob_img = np.array(total_prob_image)
    lego_loam_img = np.array(lego_loam_image)

    ## for visualization marking X or Circle
    check_img = np.zeros(total_prob_image.size)
    check_img2 = np.zeros(lego_loam_image.size)
    
    label_acc_list = []
    ele_acc_list = []
    total_acc_list = []
    lego_acc_list = []

    aa = []
    bb = []

    for i in range(2667):
        for j in range(2667):
            if lego_loam_img[j][i] == 255:
                lego_loam_img[j][i] = 1
                # check_img = cv2.drawMarker(check_img,(i,j),color=(255,0,0),markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
            else:
                lego_loam_img[j][i] = 0

            if total_prob_img[j][i] == 255:
                total_prob_img[j][i] = 1
                # check_img2 = cv2.drawMarker(check_img2,(i,j),color=(255,0,0),markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
            else :
                total_prob_img[j][i] = 0
                
            # if elevation_img[j][i] == 255 :
            #     elevation_img[j][i] = 1
            # else :
            #     elevation_img[j][i] = 0

            if lego_loam_img[j][i] != 1 and lego_loam_img[j][i] != 0 :
                print("legoloam has another number except 0 and 1 : %f" %lego_loam_img[j][i])
                # circle
                # check_img = cv2.circle(check_img,(i,j),1,(255,0,0))
                
                # x

            # if elevation_img[j][i] != 1 and elevation_img[j][i] != 0 :
            #     print("elevation has another number except 0 and 1 : %f" %elevation_img[j][i])

            if label_img[j][i] != 1 and label_img[j][i] != 0:
                print("label has another number except 0 and 1 : %f" %label_img[j][i])

            if total_prob_img[j][i] != 1 and total_prob_img[j][i] != 0:
                print("total has another number except 0 and 1 : %f" %total_prob_img[j][i])
            
            label_acc_list.append(label_img[j][i].tolist())
            # ele_acc_list.append(elevation_img[j][i].tolist())
            total_acc_list.append(total_prob_img[j][i].tolist())
            lego_acc_list.append(lego_loam_img[j][i].tolist())

    #         aa.append(recall_score(label_acc_list, total_acc_list,zero_division=0))
    #         bb.append(precision_score(label_acc_list, total_acc_list,zero_division=0))
    # print("!!!")
    # plt.plot(aa,bb,'ro')
    # plt.show()

    # check_img = cv2.resize(check_img, dsize=(640, 640))
    # check_img2 = cv2.resize(check_img2, dsize=(640, 640))
    
    # print(check_img.shape)
    # cv2.imshow("legoloam", check_img)
    # cv2.imshow("totalprob", check_img2)

    # cv2.waitKey(0)

#############################################################################
    print("Tn, Fp, Fn, Tp")
    # tn, fp, fn, tp = confusion_matrix(label_acc_list, ele_acc_list).ravel()
    # print("label vs elevation : ",tn, fp, fn ,tp)

    tn, fp, fn, tp = confusion_matrix(label_acc_list, total_acc_list).ravel()
    print("label vs total_proba : ",tn, fp, fn ,tp)

    tn, fp, fn, tp = confusion_matrix(label_acc_list, lego_acc_list).ravel()
    print("label vs legoloam : ",tn, fp, fn ,tp)
#############################################################################
    print("accurancy_score") # 전체 대비 정확하게 예측한 개수의 비율

    # ele_acc_score =accuracy_score(label_acc_list, ele_acc_list)# , normalize=False) # result is 7053677
    # print("label vs elevation : ", ele_acc_score)

    total_acc_score =accuracy_score(label_acc_list, total_acc_list)
    print("label vs total_proba : ",total_acc_score)

    lego_acc_score =accuracy_score(label_acc_list, lego_acc_list)
    print("label vs legoloam : ",lego_acc_score)
#############################################################################
    print("==="*30)
    print("precision_score") # Positive라고 예측한 것 중에서 얼마나 잘 맞았는지 비율
    
    # ele_prec_score = precision_score(label_acc_list, ele_acc_list)#,average=None)
    # print("label vs elevation : ", ele_prec_score)

    total_prec_score = precision_score(label_acc_list, total_acc_list)#,average=None)
    print("label vs total_proba : ", total_prec_score)
    
    lego_prec_score = precision_score(label_acc_list, lego_acc_list)#,average=None)
    print("label vs legoloam : ", lego_prec_score)
#############################################################################
    print("==="*30)
    print("recall_score") # Positive라고 예측한 것 중에서 얼마나 잘 맞았는지 비율
    
    # ele_recall_score = recall_score(label_acc_list, ele_acc_list)#,average=None)
    # print("label vs elevation : ", ele_recall_score)

    total_recall_score = recall_score(label_acc_list, total_acc_list)#,average=None)
    print("label vs total_proba : ", total_recall_score)
    
    lego_recall_score = recall_score(label_acc_list, lego_acc_list)#,average=None)
    print("label vs legoloam : ", lego_recall_score)

    plt.plot(total_recall_score,total_prec_score,'ro')
    plt.show()
#############################################################################
    print("==="*30)
    print("f1_score")
    
    # ele_f1_score = metrics.f1_score(label_acc_list, ele_acc_list)#, pos_label='positive', average='macro')
    # print("label vs elevation : ", ele_f1_score)

    total_f1_score = metrics.f1_score(label_acc_list, total_acc_list)#, pos_label='positive', average='macro')
    print("label vs total_proba : ",total_f1_score)    
    
    lego_f1_score = metrics.f1_score(label_acc_list, lego_acc_list)
    print("label vs legoloam : ",lego_f1_score)


if __name__ == '__main__':
    main()