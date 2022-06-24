# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 09:42:38 2022

@author: Jason
"""
import os
# os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

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
class Car(threading.Thread):
    instance = None
    Car_flag = False
    
    def __init__(self,car_signal):
        if Car.Car_flag:
            return
        self.car_signal = car_signal
        threading.Thread.__init__(self)
        self.COM_PORT = 'COM4'
        self.baudRate = 9600
        self.ser1 = serial.Serial(self.COM_PORT, self.baudRate, timeout=0.5)
        print('初始化成功')
        Car.Car_flag = True
            
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance

    def forward(self):
        self.ser1.write(1)
        print('前進')
        
    def backward(self):
        self.ser1.write(2)
        print('後退')
        
    def stop(self):
        self.ser1.write(0)
        print('停止')

    def close(self):
        self.ser1.close()
        print('關閉')
        
    def run(self):
        if self.car_signal.get() == 'stop':
            print('car stop')
            self.stop()
            
        elif self.car_signal.get()== 'move':
            # motor_control(s,1)
            print('car move')
            self.forward()
        else:
            pass

#%%
class Arm(threading.Thread):
    instance = None
    Arm_flag = False
    
    def __init__(self,car_signal,weed_signal):
        if Arm.Arm_flag:
            return
        
        self.car_signal = car_signal
        self.weed_signal = weed_signal
        
        self.actuID = [0x01, 0x02, 0x03, 0x04, 0x05]
        self.statusg = innfos.handshake()
        self.data = innfos.queryID(5)
        self.innfos.enableact(self.actuID)
        time.sleep(1)
        self.innfos.trapposmode(self.actuID) #梯形模式
        self.a = [-18,0,0,0,0]
        self.innfos.setpos(self.actuID, [0,0,0,0,0])
        time.sleep(1)
        print("手臂初始化完成")
        Arm.Arm_flag = True
        
        
        
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance
    
    def home(self):
        self.innfos.setpos(self.actuID, [0,0,0,0,0])
        time.sleep(1)
        print("手臂回家")
    
    def move(self,pos):
        self.innfos.setpos(self.actuID, pos)
        time.sleep(1)
        print("手臂移動")
        
    def arm_exit(self):
        self.innfos.disableact(self.actuID)
        print("斷開手臂連接")

    def arm_show_nowpos(self):
        self.nowpos = innfos.readpos(self.actuID)
        print(f"nowpos = {self.nowpos}")
        time.sleep(1)
    
    def arm_show_limitpos(self):
        self.limitpos = innfos.readposlimit(self.actuID) #讀取極限位置設置
        print(f"limitpos = {self.limitpos}")
        time.sleep(1)
    
    def arm_set_limitpos(self):
        self.innfos.poslimit(self.actuID,[15,-15],[15,-15])
    
    def moving(self):
        if weed_signal.get():
            data = weed_signal.get()
            mid = data[0]
            arm_loc = data[1]

            if mid and arm_loc:
                mid = (int(mid[0]),int(mid[1]))
                sx = (arm_loc[0]-mid[0])**2
                sy = (arm_loc[1]-mid[1])**2
                now_dist = int(abs((sx-sy))**0.5)
                
            if now_dist <= 50:
                #代表手臂在畫面上很靠近草
                print("往下鑽")
                time.sleep(2)
                self.car_signal.put('move')
                self.home()
                self.mid = None
                self.arm_loc = None

            else:
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
                self.move(self.a)


class Yolov5_Model():
    instance = None
    model_flag = False
    
    def __init__(self):
        
        if Yolov5_Model.model_flag:
            return
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./arm_best.pt', force_reload=True) 
        self.model.eval()
        Yolov5_Model.model_flag = True
            
    def predict(self,frame):
        self.results_roi = self.model(frame, size=640)  # includes NMS
        self.results_roi.pred
        return self.results_roi
    
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance
#%%
class Cam():
    def __init__(self,car_signal,weed_signal):
        
        self.car_signal = car_signal
        self.weed_signal = weed_signal
        
        self.hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
        self.hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
        
        self.cap = cv2.VideoCapture(2,cv2.CAP_DSHOW)
        self.model = Yolov5_Model()
        self.myColorFinder = ColorFinder()

        self.arm_loc = None
        self.mid = None


    def run(self):
        while self.cap.isOpened:
            _, self.frame = self.cap.read()
            self.results_roi= self.model.predict(self.frame)
            self.data = self.results_roi.pandas().xyxy[0]
            cv2.imshow("frame", self.frame)

            imgColor_g,mask_g = self.myColorFinder.update(self.frame,self.hsvVals_g)
            #抓取出區域輪廓以及中心點 cvzone.findContours
            imgContour_g,contours_g = cvzone.findContours(self.frame, mask_g,minArea=500)
            imgStack_all = cvzone.stackImages([ imgColor_g,imgContour_g],2,0.5)

            if contours_g:
                self.mid = contours_g[0]['center']
            else:
                self.mid = None
                
            try:
                if self.data.name.any():
                    if self.pandas().xyxy[0].name[0] == 'arm':
                        data_arm = self.results_roi.pandas().xyxy[0]
                        self.arm_loc = ((data_arm.xmin + data_arm.xmax)/2,(data_arm.ymin + data_arm.ymax)/2)
                        self.arm_loc = [int(self.arm_loc[0]),int(self.arm_loc[1])]
                        
                    else:
                        self.arm_loc = None
                    
                    # if self.results_roi.pandas().xyxy[0].name[0] == 'grass':
                    #     data_grass = self.results_roi.pandas().xyxy[0]
                    #     self.mid = ((data_grass.xmin + data_grass.xmax)/2,(data_grass.ymin + data_grass.ymax)/2)
                    # else:
                    #     self.mid = None
            except:
                pass

            try:
                for i in range(0,len(self.data)):
                    self.data1 = self.data.iloc[i]
                    # cv2.rectangle(self.frame, (int(self.data1.xmin), int(self.data1.ymin)), (int(self.data1.xmax), int(self.data1.ymax)), (0, 0, 255), 2)
                    cv2.circle(self.frame,(int(self.arm_loc[0]),int(self.arm_loc[1])), 15, (0, 255, 255), -1)
                    cv2.circle(self.frame,(int(self.mid[0]),int(self.mid[1])), 15, (0, 0, 255), -1)
            except:
                pass
            
            if self.mid and self.arm_loc:
                self.car_signal.put('stop')
                if self.weed_signal.qsize() < 1:
                    self.weed_signal.put((self.mid,self.arm_loc))
                    
            cv2.imshow("frame", self.frame)
            cv2.namedWindow('img_all', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("img_all",imgStack_all)
            
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        cv2.destroyAllWindows()
        self.cap.release()
        sys.exit()
        
        #%%
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



def main():
    car = Car()
    arm = Arm()
    cam = Cam()

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

# if __name__=="__main__":

#     main()
weed_signal = queue.Queue()
car_signal = queue.Queue()

#     t_car = threading.Thread(target=worker, daemon=True)
#     t.start()
#     t1 = threading.Thread(target=car_moving,args=(s,), daemon=True)
#     t1.start()
    