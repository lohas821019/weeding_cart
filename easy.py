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
# hsvVals_r = {'hmin': 0, 'smin': 103, 'vmin': 90, 'hmax': 9, 'smax': 255, 'vmax': 255}
hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
# hsvVals_g ={'hmin': 82, 'smin': 114, 'vmin': 98, 'hmax': 112, 'smax': 255, 'vmax': 255}

#深度學習model設定，取得模型
get_model_label = True
if get_model_label:
    global model 
    model = load_model()
    get_model_label = False

#車子馬達初始化
try:
    s = motor_init()
except:
    s.close()

#機械手臂參數設定，手臂初始位置
global a
# a = [0, -12, -15, -15, 0]
a = [-18,0,0,0,0]

# 手臂motor1={"max":10,"min":-18}
# 手臂motor2={"max":10,"min":-8}
def worker():
    while True:
        case = 0
        mid_data = weed_signal.get()
        # weed_signal.task_done()
        mid = mid_data[0]
        arm_loc = mid_data[1]
        # print(f"mid = {mid}" )
        # print(f"arm_loc = {arm_loc}" )
        
        if mid and arm_loc:
            mid = (int(mid[0]),int(mid[1]))
            sx = (arm_loc[0]-mid[0])**2
            sy = (arm_loc[1]-mid[1])**2
            now_dist = int(abs((sx-sy))**0.5)
            
        if now_dist <= 50:
            #代表手臂在畫面上很靠近草
            case = 1
        print(f"case = {case}" )
        
        if case == 0:
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
                    
                arm_move(a)
            
            except:
                pass
            
        elif case == 1:
            print("往下鑽")
            car_signal.put('move')
            global n
            n = 0
            case = 0

#無人車行進設定
def car_moving(s):
    #如果quene收到'stop' 則停止，如果quene收到'move'，則車子移動
    if car_signal.get() == 'stop':
        print('car stop')
        # motor_control(s,0)
    elif car_signal.get()== 'move':
        # motor_control(s,1)
        print('car move')
    else:
        pass


#Queue初始化宣告
# weed_signal = queue.Queue()
# car_signal = queue.Queue()

# t = threading.Thread(target=worker, daemon=True)
# t.start()
# t1 = threading.Thread(target=car_moving,args=(s,), daemon=True)
# t1.start()

#%%

#手臂初始化
ans = arm_init()
arm_home()

#step1
# task_done = False
cap = cv2.VideoCapture(2,cv2.CAP_DSHOW)

while 1:
    task_done = False
    _, frame = cap.read()
    results_roi = model(frame, size=640)  # includes NMS
    results_roi.pred
    data = results_roi.pandas().xyxy[0]
    try:
        if data.name.any():
            if results_roi.pandas().xyxy[0].name[0] == 'arm':
                data = results_roi.pandas().xyxy[0]
                arm_loc = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
                arm_loc = [int(arm_loc[0]),int(arm_loc[1])]
            else:
                arm_loc = None
    except:
        pass
    
    imgColor_g,mask_g = myColorFinder.update(frame,hsvVals_g)
    
    #抓取出區域輪廓以及中心點 cvzone.findContours
    imgContour_g,contours_g = cvzone.findContours(frame, mask_g,minArea=500)
    imgStack_all = cvzone.stackImages([ imgColor_g,imgContour_g],2,0.5)

    if contours_g:
        mid = contours_g[0]['center']
    else:
        mid = None

    #將有判斷出來的雜草圈出，並且計算出中心點mid，並且用圓圈標出中心點
    try:
        cv2.circle(frame,(int(arm_loc[0]),int(arm_loc[1])), 8, (0, 255, 255), -1)
        cv2.circle(frame,(int(mid[0]),int(mid[1])), 8, (0, 0, 255), -1)
        # car_signal.put('stop')
    except:
        pass
    cv2.imshow('frame', frame)

#step2
    #如果有抓到草
    if mid and arm_loc:
        print('車子停止，除草')
        #計算手臂與雜草距離
        mid = (int(mid[0]),int(mid[1]))
        sx = (arm_loc[0]-mid[0])**2
        sy = (arm_loc[1]-mid[1])**2
        now_dist = int(abs((sx-sy))**0.5)
        
        if now_dist >= 50:
            case = 0
        else:
            case = 1
            
        if case == 0:
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
                arm_move(a)

            except:
                pass
            
        elif case == 1:
            print("往下鑽")
            time.sleep(2)
            arm_home()
            mid = None
            arm_loc = None
            task_done =True
            break
    #如果沒有抓到草
    else:
        print('車子行進')
        
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        arm_home()
        time.sleep(2)
        arm_exit()
        break
    
    if task_done == True:
        continue

cv2.destroyAllWindows()
cap.release()
time.sleep(2)
sys.exit()

