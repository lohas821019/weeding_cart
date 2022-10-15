# -*- coding: utf-8 -*-
"""
Created on Tue Jul 26 10:20:32 2022

@author: Jason
"""

import os
import time
from predict import *
from motor import *
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
from action import *
import sys

# os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

global a
a = [-18,0,0,0,0]

def arm_control1():
                    #迴圈重複判斷讓手臂到定點，是否到定點由camera產生的arm_loc 看他有沒有落在weed圈選的範圍
    try:
        #控制左右
        if a[0] > 0 :
            a[0] = 0
        elif a[0] < -18 :
            a[0] = -18
            
        #控制上下
        # if a[1] > 10 :
        #     a[1] = 10
        # elif a[1] <-8 :
        #     a[1] = -8

        #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
        #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
        if mid[0] - arm_loc[0] <= 0:
            a[0] = a[0]+0.5
        else:
            a[0] = a[0]-0.5
            
        #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
        #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
        # if mid[1] - arm_loc[1] >= 0:
        #     a[1] = a[1]+0.5
        # else:
        #     a[1] = a[1]-0.5
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
    
def arm_control_by_red():
    try:
        #控制左右
        # if a[4] > 10 :
        #     a[4] = 10 
        # elif a[4] < -10 :
        #     a[4] = -10
        
        if a[0] > 0 :
            a[0] = 0
        elif a[0] < -18 :
            a[0] = -18
        
        #控制上下
        if a[3] > 10 :
            a[3] = 10
        elif a[3] <-10 :
            a[3] = -10

        #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
        #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
        # if temp_grass_B[0] - arm_loc_web[0] <= 0:
        #     a[4] = a[4]+0.5
        # else:
        #     a[4] = a[4]-0.5
        
        if temp_grass_B[0] - arm_loc_web[0] <= 0:
            a[0] = a[0]+0.5
        else:
            a[0] = a[0]-0.5
            
        #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
        #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
        if temp_grass_B[1] - arm_loc_web[1] >= 0:
            a[3] = a[3]+0.5
        else:
            a[3] = a[3]-0.5
    except:
        pass
#%%

#手臂初始化
ans = arm_init()
arm_home()

model = torch.hub.load(r'C:\Users\zanrobot\Documents\Github\yolov5', 'custom', path=r'C:\Users\zanrobot\Documents\Github\yolov5/arm_best.pt', source='local')
model = torch.hub.load(r'C:\Users\Jason\Documents\GitHub\yolov5', 'custom', path=r'C:\Users\Jason\Documents\GitHub\weeding_cart/arm_best.pt', source='local')

model.eval()

try:
    s = motor_init()
except:
    s.close()

first = 1
#step1
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap_web = cv2.VideoCapture(2,cv2.CAP_DSHOW)

grass_flag_A = 1
grass_flag_B = 1

while 1:
    #cam1
    _, frame = cap.read()

    results_roi = model(frame, size=640)
    results_roi.pred
    data = results_roi.pandas().xyxy[0]
    
    #cam2
    _, frame_web = cap_web.read()
    # frame_web = frame_web[]

    results_roi_web = model(frame_web, size=640)
    results_roi_web.pred
    data_web = results_roi_web.pandas().xyxy[0]

    cv2.imshow('frame', frame)
    cv2.imshow('frame_web', frame_web)
    
    #test
    # path =r'C:\Users\Jason\Documents\GitHub\weeding_cart\resoureces\IMG_9069 22.jpg'
    # cap_web = cv2.VideoCapture(path)
    

#%%cam1
    if data.name.any():
        #cam1尋找手臂的位置
        if results_roi.pandas().xyxy[0].name[0] == 'arm':
            data = results_roi.pandas().xyxy[0]
            arm_loc = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
            arm_loc = [int(arm_loc[0][0]),int(arm_loc[1][0])]
            cv2.circle(frame,(int(arm_loc[0]),int(arm_loc[1])), 8, (0, 255, 255), -1)
            
        #cam1尋找草的位置
        elif results_roi.pandas().xyxy[0].name[0] == 'grass':
            data = results_roi.pandas().xyxy[0]
            grass_loc = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
            grass_loc = [int(grass_loc[0][0]),int(grass_loc[1][0])]
            
            if grass_flag_A:
                temp_grass_A = grass_loc
                grass_flag_A = 0
                cv2.circle(frame,(int(temp_grass_A[0]),int(temp_grass_A[1])), 8, (0, 0, 255), -1)
            else:
                temp_grass_A = None

#%%cam2
    if data_web.name.any():
        #cam2找手臂的位置
        if results_roi.pandas().xyxy[0].name[0] == 'arm':
            data_web = results_roi_web.pandas().xyxy[0]
            arm_loc_web = ((data_web.xmin + data_web.xmax)/2,(data_web.ymin + data_web.ymax)/2)
            arm_loc_web = [int(arm_loc_web[0][0]),int(arm_loc_web[1][0])]
            cv2.circle(frame_web,(int(arm_loc_web[0]),int(arm_loc_web[1])), 8, (0, 255, 255), -1)
            
        #cam2找草的位置
        elif results_roi.pandas().xyxy[0].name[0] == 'grass':
            data_web = results_roi_web.pandas().xyxy[0]
            grass_loc_web = ((data_web.xmin + data_web.xmax)/2,(data_web.ymin + data_web.ymax)/2)
            grass_loc_web = [int(grass_loc_web[0][0]),int(grass_loc_web[1][0])]
            
            if grass_flag_B:
                temp_grass_B = grass_loc_web
                grass_flag_B = 0
                cv2.circle(frame_web,(int(temp_grass_B[0]),int(temp_grass_B[1])), 8, (0, 0, 255), -1)
            else:
                temp_grass_B = None

    cv2.waitKey(1)
    cv2.imshow('frame', frame)
    cv2.imshow('frame_web', frame_web)

#%%
#step2
    #如果有抓到草，車子停止
    if temp_grass_A: #and arm_loc:
        motor_control(s,0)
        
        #計算手臂與雜草距離
        sx = pow(abs((arm_loc[0]-temp_grass_A[0])),2)
        sy = pow(abs((arm_loc[1]-temp_grass_A[1])),2)
        now_dist = int(abs((sx-sy))**0.5)
        print(f'now_dist = {now_dist}')
        
        try:
            if temp_grass_B and arm_loc_web:
             #計算手臂與雜草距離
             sx = pow(abs((arm_loc_web[0]-temp_grass_B[0])),2)
             sy = pow(abs((arm_loc_web[1]-temp_grass_B[1])),2)
             now_dist_web = int(abs((sx-sy))**0.5)
             print(f'now_dist_web = {now_dist_web}')
        except:
             pass
         
        if first:
            n = 0
            data1 = []
            first = 0
                
        if now_dist!=0:
            n = n + 1
            print(f'n = {n}')
            data1.append(now_dist)
            print(f'data1 = {data1}')
                
        if now_dist >= 80:
            case = 0
        else:
            case = 1
            
        if n==5:
            if data1[n-1]-data1[n-2]<=3 and data1[n-2]-data1[n-3]<=3 and data1[n-3]-data1[n-4]<=5:
                arm_move(a)
                print(f'arm_move(a) = {a}')
                n = 0
                first = 1
                if case == 0:
                    arm_control1()
                elif case == 1:
                    # arm_control2()
                    arm_control_by_red()
                    try:
                        if now_dist_web <= 20:
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
                            grass_flag = 1
                    except:
                        pass
        elif n>5:
            first = 1
            
     #如果沒抓到草，車子移動
    else:
        motor_control(s,1)
        pass

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        arm_home()
        break

cv2.destroyAllWindows()
cap.release()
arm_exit()

time.sleep(2)
sys.exit()