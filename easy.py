# -*- coding: utf-8 -*-
"""
Created on Tue Jun 21 11:56:16 2022

@author: Jason
"""
import os
os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
# os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

import threading , queue
import time
#使用一般webcam
from predict import *
from motor import *
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
from action import *
import sys

#抓取紅色設定
myColorFinder = ColorFinder()
myColorFinder1 = ColorFinder()
# hsvVals_r = {'hmin': 0, 'smin': 103, 'vmin': 90, 'hmax': 9, 'smax': 255, 'vmax': 255}
hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
hsvVals_g_web = {'hmin': 39, 'smin': 95, 'vmin': 0, 'hmax': 104, 'smax': 214, 'vmax': 255}
# hsvVals_g ={'hmin': 82, 'smin': 114, 'vmin': 98, 'hmax': 112, 'smax': 255, 'vmax': 255}

#深度學習model設定，取得模型
get_model_label = True
if get_model_label:
    global model 
    model = load_model()
    get_model_label = False

#車子馬達初始化
# try:
#     s = motor_init()
# except:
#     s.close()

#機械手臂參數設定，手臂初始位置
global a
# a = [0, -12, -15, -15, 0]
a = [-18,0,0,0,0]

# 手臂motor1={"max":10,"min":-18}
# 手臂motor2={"max":10,"min":-8}
def arm_control1():
                    #迴圈重複判斷讓手臂到定點，是否到定點由camera產生的arm_loc 看他有沒有落在weed圈選的範圍
    try:
        #控制左右
        if a[0] > 0 :
            a[0] = 0
        elif a[0] < -18 :
            a[0] = -18
            
        #控制上下
        if a[1] > 10 :
            a[1] = 10
        elif a[1] <-8 :
            a[1] = -8

        #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
        #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
        if mid[0] - arm_loc[0] <= 0:
            a[0] = a[0]+0.5
        else:
            a[0] = a[0]-0.5
            
        #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
        #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
        if mid[1] - arm_loc[1] >= 0:
            a[1] = a[1]+0.5
        else:
            a[1] = a[1]-0.5
    except:
        pass
    
def arm_control2():
                    #迴圈重複判斷讓手臂到定點，是否到定點由camera產生的arm_loc 看他有沒有落在weed圈選的範圍
    try:
        #控制左右
        if a[4] > 10 :
            a[4] = 10 
        elif a[4] < -10 :
            a[4] = -10
            
        #控制上下
        if a[3] > 10 :
            a[3] = 10
        elif a[3] <-10 :
            a[3] = -10

        #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
        #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
        if mid_web[0] - arm_loc_web[0] <= 0:
            a[4] = a[4]+0.5
        else:
            a[4] = a[4]-0.5
            
        #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
        #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
        if mid_web[1] - arm_loc_web[1] >= 0:
            a[3] = a[3]+0.5
        else:
            a[3] = a[3]-0.5
    except:
        pass
    
#%%

#手臂初始化
ans = arm_init()
arm_home()

first = 1

#step1
cap = cv2.VideoCapture(2,cv2.CAP_DSHOW)
cap_web = cv2.VideoCapture(3,cv2.CAP_DSHOW)


while 1:
    _, frame = cap.read()
    results_roi = model(frame, size=640)  # includes NMS
    results_roi.pred
    data = results_roi.pandas().xyxy[0]

    _, frame_web = cap_web.read()
    results_roi_web = model(frame_web, size=640)
    results_roi_web.pred
    data_web = results_roi_web.pandas().xyxy[0]


    if data.name.any():
        if results_roi.pandas().xyxy[0].name[0] == 'arm':
            data = results_roi.pandas().xyxy[0]
            arm_loc = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
            arm_loc = [int(arm_loc[0][0]),int(arm_loc[1][0])]
            cv2.circle(frame,(int(arm_loc[0]),int(arm_loc[1])), 8, (0, 255, 255), -1)
        else:
            arm_loc = None
        
        print('===========================================')
        print(f"arm_loc = {arm_loc}")

    if data_web.name.any():
        if results_roi_web.pandas().xyxy[0].name[0] == 'arm':
            data_web = results_roi_web.pandas().xyxy[0]
            arm_loc_web = ((data_web.xmin + data_web.xmax)/2,(data_web.ymin + data_web.ymax)/2)
            arm_loc_web = [int(arm_loc_web[0][0]),int(arm_loc_web[1][0])]
            cv2.circle(frame_web,(int(arm_loc_web[0]),int(arm_loc_web[1])), 8, (0, 255, 255), -1)

        else:
            arm_loc_web = None
        print('===========================================')
        print(f"arm_loc_web = {arm_loc_web}")

    imgColor_g,mask_g = myColorFinder.update(frame,hsvVals_g)
    
    #抓取出區域輪廓以及中心點 cvzone.findContours
    imgContour_g,contours_g = cvzone.findContours(frame, mask_g,minArea=500)
    imgStack_all = cvzone.stackImages([ imgColor_g,imgContour_g],2,0.5)


    #將有判斷出來的雜草圈出，並且計算出中心點mid，並且用圓圈標出中心點
    try:
        if contours_g:
            mid = contours_g[0]['center']
            cv2.circle(frame,(int(mid[0]),int(mid[1])), 8, (0, 0, 255), -1)
        else:
            mid = None
    except:
        pass

    
    imgColor_g_web,mask_g_web = myColorFinder1.update(frame_web,hsvVals_g_web)
    
    #抓取出區域輪廓以及中心點 cvzone.findContours
    imgContour_g_web,contours_g_web = cvzone.findContours(frame_web, mask_g_web,minArea=500)
    imgStack_all_web = cvzone.stackImages([ imgColor_g_web,imgColor_g_web],2,0.5)


    #將有判斷出來的雜草圈出，並且計算出中心點mid，並且用圓圈標出中心點
    try:
        if contours_g_web:
            mid_web = contours_g_web[0]['center']
            cv2.circle(frame_web,(int(mid_web[0]),int(mid_web[1])), 8, (0, 0, 255), -1)
        else:
            mid_web = None
            
    except:
        pass
    cv2.waitKey(1)
    cv2.imshow('frame', frame)
    cv2.imshow('frame_web', frame_web)
    
    # print('===========================================')
    # print(f"mid_web = {mid_web}")
    # print(f"arm_loc_web = {arm_loc_web}")
    
#%%
#step2
    #如果有抓到草
    if mid and arm_loc:
        #計算手臂與雜草距離
        sx = pow(abs((arm_loc[0]-mid[0])),2)
        sy = pow(abs((arm_loc[1]-mid[1])),2)
        now_dist = int(abs((sx-sy))**0.5)
        print(f'now_dist = {now_dist}')

    try:
        if mid_web and arm_loc_web:
         #計算手臂與雜草距離
         sx = pow(abs((arm_loc_web[0]-mid_web[0])),2)
         sy = pow(abs((arm_loc_web[1]-mid_web[1])),2)
         now_dist_web = int(abs((sx-sy))**0.5)
         print(f'now_dist_web = {now_dist_web}')
    except:
         pass

#%%

    if first:
            n = 0
            data1 = []
            first = 0

    if now_dist!=0:
            n = n + 1
            print(f'n = {n}')
            data1.append(now_dist)
            print(f'data1 = {data1}')

    if now_dist >= 40:
        case = 0
    else:
        case = 1

    if n==10:
            if data1[n-1]-data1[n-2]<=3 and data1[n-2]-data1[n-3]<=3 and data1[n-3]-data1[n-4]<=5:
     
                arm_move(a)
                print(f'arm_move(a) = {a}')
                n = 0
                first = 1

                if case == 0:
                    arm_control1()
                    
                elif case == 1:
                    arm_control2()
                    
                    try:
                        if now_dist_web<=20:
                            print("往下鑽")
                            nowpos = innfos.readpos(actuID)
                            print(f'nowpos = {nowpos}')
                            # arm_move([nowpos[0], nowpos[1], -7, 0, 0])
                            time.sleep(3)
                            arm_home()
                            a = [-18,0,0,0,0]
                            mid = None
                            arm_loc = None
                            case = 0
                    except:
                        pass
    elif n>10:
        first = 1
    #如果沒有抓到草

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        arm_home()
        break

cv2.destroyAllWindows()
cap.release()
arm_exit()

time.sleep(2)
sys.exit()

