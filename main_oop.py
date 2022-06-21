# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 09:42:38 2022

@author: Jason
"""
import os
# os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
# os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

import threading , queue
import time
import math
import sys

#使用一般webcam
from predict import *
from motor import *
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
from action import *

#%%
class Car():
    def __init__(self):
        self.s = motor_init()

    def move(self,s):
        self.motor_control(s,0)

    def close(self):
        self.s.close()


class Arm():
    def __init__(self):
        self.ans = arm_init()
        self.a = [-18,0,0,0,0]
        self.home()
        
    def home(self):
        arm_home()
    
    def moving(self,mid,arm_loc):
        #控制左右
        if self.a[0] > 0 :
            self.a[0] = 0
        elif self.a[0] < -18 :
            self.a[0] = -18

        #控制上下
        if self.a[1] > 10 :
            self.a[1] = 10
        elif self.a[1] <-8 :
            self.a[1] = -8

        #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
        #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
        if mid[0] - arm_loc[0] <= 0:
            self.a[0] = self.a[0]+0.5
        else:
            self.a[0] = self.a[0]-0.5
        
        #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
        #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
        
        if mid[1] - arm_loc[1] >= 0:
            self.a[1] = self.a[1]+0.5
        else:
            self.a[1] = self.a[1]-0.5
        arm_move(self.a)

class Yolov5_Model():
    def __init__(self):
        self.model_flag = False
        if self.model_flag == False:
            self.model = load_model()

    def predict(self,frame):
        self.results_roi = self.model(frame, size=640)  # includes NMS
        self.results_roi.pred
        return self.results_roi

class Cam():
    def __init__(self):
        self.hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
        self.hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
        # self.cap = cv2.VideoCapture(2,cv2.CAP_DSHOW)

def worker(arm):
    while True:
        case = 0
        mid_data = weed_signal.get()
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
                arm.moving(mid,arm_loc)
            except:
                pass
            
        elif case == 1:
            print("往下鑽")
            car_signal.put('move')
            n = 0
            main_signal.put(n)
            case = 0

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

def main():
    myColorFinder = ColorFinder()
    car = Car()
    arm = Arm()
    model = Yolov5_Model()
    model_flag = True
    cam = Cam()

    if main_signal.get():
        n = main_signal.get()
    else:
        n = 0

    # cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
    cap = cv2.VideoCapture(0)
    while cap.isOpened:
        _, frame = cap.read()
        cv2.imshow("frame", frame)
        results_roi = model.predict(frame)
        data = results_roi.pandas().xyxy[0]

        try:
            if data.name.any():
                if results_roi.pandas().xyxy[0].name[0] == 'arm':
                    data = results_roi.pandas().xyxy[0]
                    arm_loc = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
                    arm_loc = [int(arm_loc[0]),int(arm_loc[1])]
                else:
                    arm_loc = None
                
                if results_roi.pandas().xyxy[0].name[0] == 'grass':
                    mid = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
                else:
                    mid = None
            else:
                arm_loc = None
        except:
            pass


        imgColor_g,mask_g = myColorFinder.update(frame,cam.hsvVals_g)
        #抓取出區域輪廓以及中心點 cvzone.findContours
        imgContour_g,contours_g = cvzone.findContours(frame, mask_g,minArea=500)
        imgStack_all = cvzone.stackImages([ imgColor_g,imgContour_g],2,0.5)

        if contours_g:
            mid = contours_g[0]['center']
        else:
            mid = None
            
        if n == 0:
            temp = mid
            n = 1      
            
        try:
            cv2.circle(frame,(int(arm_loc[0]),int(arm_loc[1])), 8, (0, 255, 255), -1)
            cv2.circle(frame,(int(temp[0]),int(temp[1])), 8, (0, 0, 255), -1)
            
            car_signal.put('stop')
            if weed_signal.qsize() < 1:
                weed_signal.put((temp,arm_loc))
        except:
            pass

        cv2.imshow("frame", frame)
        cv2.namedWindow('img_all', cv2.WINDOW_AUTOSIZE)
        cv2.imshow("img_all",imgStack_all)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            arm_home()
            time.sleep(2)
            arm_exit()
            break

    cv2.destroyAllWindows()
    cap.release()
    sys.exit()

if __name__=="__main__":

    main()
    main_signal =queue.Queue()
    weed_signal = queue.Queue()
    car_signal = queue.Queue()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t1 = threading.Thread(target=car_moving,args=(s,), daemon=True)
    t1.start()
    