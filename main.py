# -*- coding: utf-8 -*-
"""
Created on Thu May 19 11:11:06 2022

@author: Jason
"""

import os
# os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

import threading , queue
import time

#使用一般webcam
from predict import *
from motor import *
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
from action import *

#抓取紅色設定
myColorFinder = ColorFinder()
hsvVals_r = {'hmin': 0, 'smin': 128, 'vmin': 134, 'hmax': 67, 'smax': 255, 'vmax': 255}
# hsvVals_r = {'hmin': 119, 'smin': 194, 'vmin': 116, 'hmax': 179, 'smax': 255, 'vmax': 255}
hsvVals_g = {'hmin': 38, 'smin': 166, 'vmin': 0, 'hmax': 120, 'smax': 255, 'vmax': 255}
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
        mid = weed_signal.get()

        #迴圈重複判斷讓手臂到定點，是否到定點由camera產生的arm_loc 看他有沒有落在weed圈選的範圍
        try:
            #需重新確認上下的正負號
            #控制左右
            if a[0] > 10 :
                a[0] = 10
            elif a[0] < -10 :
                a[0] = -10

            #控制上下
            if a[1] > 0 :
                a[1] = 0
            elif a[1] <-12 :
                a[1] = -12

            #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
            #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
            if mid[0] - arm_loc[0] > 0:
                a[0] = a[0]+1
                weed_signal.put(a)
            else:
                a[0] = a[0]-1
                weed_signal.put(a)
                
            #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
            #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
            
            if mid[1] - arm_loc[1] > 0:
                a[1] = a[1]+1
                weed_signal.put(a)
            else:
                a[1] = a[1]-1
                weed_signal.put(a)
            arm_move(a)
        except:
            pass

        #如果手臂已經到達定點，伸長手臂除草，然後讓車移動
        if region[0] <= arm_loc[0] <=region[2] and region[1] <= arm_loc[1] <=region[3]:
            #這邊再加入手臂伸長的動作
            arm_move([-18,0,0,0,0])
            time.sleep(2)
            #這邊加入手臂收回的動作
            arm_move([-18,-8,-15,-15,0])
            time.sleep(2)
            car_signal.put()== 'move'

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


#手臂初始化
ans = arm_init()

#Queue初始化宣告
weed_signal = queue.Queue()
car_signal = queue.Queue()

threading.Thread(target=worker, daemon=True).start()
threading.Thread(target=car_moving,args=(s,), daemon=True).start()
#%%

cap = cv2.VideoCapture(2)
hw = []
global arm_loc

while cap.isOpened():
    _, frame = cap.read()
    
    if hw == []:
        h = frame.shape[0]
        w = frame.shape[1]
        hw.append((h,w))
        # mid_px =(1.5*h)/2
        # mid_py = w/2
        # mid_pic = (mid_px,mid_py)
        region = []

    # roi = frame[int(h/2):h,0:w]
    results_roi = model(frame, size=640)  # includes NMS
    results_roi.pred
    data = results_roi.pandas().xyxy[0]
    
    #分辨出紅色與綠色
    imgColor_r,mask_r = myColorFinder.update(frame,hsvVals_r)
    imgColor_g,mask_g = myColorFinder.update(frame,hsvVals_g)
    
    #抓取出區域輪廓以及中心點
    imgContour_r,contours_r = cvzone.findContours(frame, mask_r)
    imgContour_g,contours_g = cvzone.findContours(frame, mask_g)
    imgStack_all = cvzone.stackImages([imgColor_r, imgColor_g, imgContour_r, imgContour_g],2,0.5)
    
    #找到紅色的區域(手臂的位置)
    if contours_r:
        arm_loc = contours_r[0]['center']

    #將有判斷出來的雜草圈出，並且計算出中心點mid，並且用圓圈標出中心點
    try:
        for i in range(0,len(data)):
            data = data.iloc[i]
            region = [int(data.xmin), int(data.ymin), int(data.xmax), int(data.ymax)]
            cv2.rectangle(frame, (int(data.xmin), int(data.ymin)), (int(data.xmax), int(data.ymax)), (0, 0, 255), 2)
            # mid = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
            #測試用
            mid = contours_g[0]['center']
            # print(mid)
            cv2.circle(frame,(int(mid[0]),int(mid[1])), 8, (0, 0, 255), -1)
            car_signal.put('stop')
            weed_signal.put(mid)
    except:
        pass

    # cv2.imshow("frame", frame)
    cv2.namedWindow('img_all', cv2.WINDOW_AUTOSIZE)
    cv2.imshow("img_all",imgStack_all)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        # arm_home()
        time.sleep(2)
        break

cv2.destroyAllWindows()
cap.release()
time.sleep(2)
arm_exit()

