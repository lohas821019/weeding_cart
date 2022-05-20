# -*- coding: utf-8 -*-
"""
Created on Thu May 19 11:11:06 2022

@author: Jason
"""


import os
os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

import threading , queue

import time

#使用一般webcam
from predict import *
from motor import *
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder

#抓取紅色設定
myColorFinder = ColorFinder()

hsvVals_r = {'hmin': 130, 'smin': 212, 'vmin': 86, 'hmax': 179, 'smax': 255, 'vmax': 255}
hsvVals_g = {'hmin': 32, 'smin': 99, 'vmin': 0, 'hmax': 55, 'smax': 158, 'vmax': 255}

#深度學習model設定
get_model_label = 1

if get_model_label:
    global model 
    model = load_model()
    get_model_label = 0

s = motor_init()

#機械手臂參數設定
global a
a = [0, -12, -15, -15, 0]

def worker():
    while True:
        mid = weed_signal.get()

        #重複移動讓手臂到定點
        try:
            if a[0] > 10 :
                a[0] = 10
            elif a[0] < -10 :
                a[0] = -10
    
            if a[1] > 0 :
                a[1] = 0
            elif a[1] <-12 :
                a[1] = -12
                    
            if mid[0] - mid_pic[0] > 0:
                a[0] = a[0]+1
                weed_signal.put(a)
            else:
                a[0] = a[0]-1
                weed_signal.put(a)
                
            if mid[1] - mid_pic[1] > 0:
                a[1] = a[1]+1
                weed_signal.put(a)
            else:
                a[1] = a[1]-1
                weed_signal.put(a)
            arm_move(a)
        except:
            pass
        
        #如果手臂已經除玩草，讓車移動
        if mid == None:
            car_signal.put()== 'move'

#無人車行進設定
def car_moving(s):
    if car_signal.get() == 'stop':
        motor_control(s,0)
    elif car_signal.get()== 'move':
        motor_control(s,1)
    else:
        pass
    
# ans = arm_init()
weed_signal = queue.Queue()
car_signal = queue.Queue()

threading.Thread(target=worker, daemon=True).start()
threading.Thread(target=car_moving,args=(s,), daemon=True).start()
#%%
cap = cv2.VideoCapture(1)
hw = []

while cap.isOpened():
    _, frame = cap.read()
    
    if hw == []:
        h = frame.shape[0]
        w = frame.shape[1]
        hw.append((h,w))
        mid_px =(1.5*h)/2
        mid_py = w/2
        mid_pic = (mid_px,mid_py)

    roi = frame[int(h/2):h,0:w]
    results_roi = model(roi, size=640)  # includes NMS
    results_roi.pred
    data = results_roi.pandas().xyxy[0]    # includes NMS
    
    imgColor_r,mask_r = myColorFinder.update(frame,hsvVals_r)
    imgColor_g,mask_g = myColorFinder.update(frame,hsvVals_g)

    imgContour_r,contours_r = cvzone.findContours(frame, mask_r)
    imgContour_g,contours_g = cvzone.findContours(frame, mask_g)
    imgStack_all = cvzone.stackImages([imgColor_r, imgColor_g, imgContour_r, imgContour_g],2,0.5)


    try:
        for i in range(0,len(data)):
            data = data.iloc[i]
            cv2.rectangle(roi, (int(data.xmin), int(data.ymin)), (int(data.xmax), int(data.ymax)), (0, 0, 255), 2)
            mid = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
            print(mid)
            cv2.circle(roi,(int(mid[0]),int(mid[1])), 15, (0, 0, 255), -1)
            
            car_signal.put('stop')
            weed_signal.put(mid)

    except:
        pass

    cv2.imshow("roi", roi)
    cv2.namedWindow('img_all', cv2.WINDOW_AUTOSIZE)
    cv2.imshow("img_all",imgStack_all)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        # arm_home()
        time.sleep(2)
        break

cv2.destroyAllWindows()
cap.release()
# arm_exit()
