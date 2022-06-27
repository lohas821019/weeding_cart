# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 09:42:38 2022
@author: Jason
"""
import os
os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
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
    instance = None
    Car_flag = False
    
    def __init__(self):
        if Car.Car_flag:
            return
        # threading.Thread.__init__(self)
        self.COM_PORT = 'COM3'
        self.baudRate = 9600
        self.ser1 = serial.Serial(self.COM_PORT, self.baudRate, timeout=0.5)
        print('車輛初始化成功')
        Car.Car_flag = True
            
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance

    def forward(self):
        self.ser1.write(1)
        print('車輛前進')
        
    def backward(self):
        self.ser1.write(2)
        print('車輛後退')
        
    def stop(self):
        self.ser1.write(0)
        print('車輛停止')

    def close(self):
        self.ser1.close()
        print('車輛關閉')
        
    # def run(self):
    #     while 1 :
    #         job = self.car_signal.get()
    #         # print(f'Working on {job}')

    #         if job == 'stop':
    #             # print('car stop')
    #             self.stop()
                
    #         elif job == 'move':
    #             # print('car move')
    #             self.forward()
                
    #         elif job == 'q':
    #             self.close()
    #             # print('Ending the car')
    #             break
    #         # print(f'Finished {job}')
    #         self.car_signal.task_done()
            
#測試用
# car_signal = queue.Queue()
# car_signal.empty()
# c =Car(car_signal)
# c.start()
# c.isAlive()
# c.close()

# car_signal.put('move')
# car_signal.put('stop')
# car_signal.put('q')


#%%
class Arm():
    instance = None
    Arm_flag = False
    
    def __init__(self):
        if Arm.Arm_flag:
            return
        # threading.Thread.__init__(self)
    
        
        self.actuID = [0x01, 0x02, 0x03, 0x04, 0x05]
        self.statusg = innfos.handshake()
        self.data = innfos.queryID(5)
        innfos.enableact(self.actuID)
        time.sleep(1)
        innfos.trapposmode(self.actuID) #梯形模式
        self.a = [-18,0,0,0,0]
        innfos.setpos(self.actuID, [0,0,0,0,0])
        time.sleep(1)
        print("手臂初始化完成")
        Arm.Arm_flag = True
        
        
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance
    
    def home(self):
        innfos.setpos(self.actuID, [0,0,0,0,0])
        time.sleep(2)
        print("手臂回家")
    
    def move(self,pos):
        innfos.setpos(self.actuID, pos)
        time.sleep(2)
        print("手臂移動")
        
    def arm_exit(self):
        innfos.disableact(self.actuID)
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
        innfos.poslimit(self.actuID,[15,-15],[15,-15])

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
    def __init__(self,car,arm):
        self.car = car
        self.arm = arm
        
        self.car_falg = 1
        self.arm_falg = 1
        
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
            # print(self.data)
            cv2.imshow("frame", self.frame)

            imgColor_g,mask_g = self.myColorFinder.update(self.frame,self.hsvVals_g)
            
            #抓取出區域輪廓以及中心點 cvzone.findContours
            imgContour_g,contours_g = cvzone.findContours(self.frame, mask_g,minArea=500)
            imgStack_all = cvzone.stackImages([ imgColor_g,imgContour_g],2,0.5)

            if contours_g:
                self.mid = contours_g[0]['center']
                cv2.circle(self.frame,(int(self.mid[0]),int(self.mid[1])), 15, (0, 0, 255), -1)

            else:
                self.mid = None
                
            try:
                if self.data.name.any():
                    if self.results_roi.pandas().xyxy[0].name[0] == 'arm':
                        data_arm = self.results_roi.pandas().xyxy[0]
                        self.arm_loc = ((data_arm.xmin + data_arm.xmax)/2,(data_arm.ymin + data_arm.ymax)/2)
                        self.arm_loc = [int(self.arm_loc[0]),int(self.arm_loc[1])]
                        cv2.circle(self.frame,(int(self.arm_loc[0]),int(self.arm_loc[1])), 15, (0, 255, 255), -1)
    
                    else:
                        self.arm_loc = None
                    
                    # if self.results_roi.pandas().xyxy[0].name[0] == 'grass':
                    #     data_grass = self.results_roi.pandas().xyxy[0]
                    #     self.mid = ((data_grass.xmin + data_grass.xmax)/2,(data_grass.ymin + data_grass.ymax)/2)
                    # else:
                    #     self.mid = None
            except:
                pass

            # try:
            #     for i in range(0,len(self.data)):
            #         self.data1 = self.data.iloc[i]
            #         # cv2.rectangle(self.frame, (int(self.data1.xmin), int(self.data1.ymin)), (int(self.data1.xmax), int(self.data1.ymax)), (0, 0, 255), 2)
            #         # cv2.circle(self.frame,(int(self.arm_loc[0]),int(self.arm_loc[1])), 15, (0, 255, 255), -1)
            #         cv2.circle(self.frame,(int(self.mid[0]),int(self.mid[1])), 15, (0, 0, 255), -1)
            # except:
            #     pass
            
            # self.mid =[100,100]
            # self.arm_loc =[200,200]
            
            try:
                if self.mid and self.arm_loc and len(self.arm_loc) == 2:
                    if self.car_falg:
                        self.car.stop()
                        self.car_falg = 0
                    # if self.arm_falg:
                    #     self.arm.home()
    
                    print(f'mid = {self.mid}')
                    print(f'arm_loc = {self.arm_loc}')
                    
                    sx = pow(abs((self.arm_loc[0]-self.mid[0])),2)
                    sy = pow(abs((self.arm_loc[1]-self.mid[1])),2)
                    self.now_dist = int(abs((sx-sy))**0.5)
                    print(f'now_dist = {self.now_dist}')
    
                    if self.now_dist <= 40:
                        #代表手臂在畫面上很靠近草
                        print("往下鑽")
                        self.arm.move([-15.599999999999975, -7.19999999999999, -7, 0, 0])
                        self.arm.home()
                        self.arm.a = [-18,0,0,0,0]
                        self.mid = None
                        self.arm_loc = None
                        self.car_falg = 1
                        self.car.forward()
                
                    else:
                    #控制左右
                        if self.arm.a[0] > 0 :
                            self.arm.a[0] = 0
                     #控制上下
                        if self.arm.a[1] > 10 :
                            self.arm.a[1] = 10
                        elif self.arm.a[0] < -18 :
                            self.arm.a[0] = -18
                    
                        elif self.arm.a[1] <-8 :
                            self.arm.a[1] = -8
                    
                        #如果目標物中心點x大於手臂的中心點x，則控制手臂往左
                        #如果目標物中心點x小於手臂的中心點x，則控制手臂往右
                        if self.mid[0] - self.arm_loc[0] <= 0:
                            self.arm.a[0] = self.arm.a[0]+0.5
                        else:
                            self.arm.a[0] = self.arm.a[0]-0.5
                        
                        #如果目標物中心點y大於手臂的中心點y，則控制手臂往上
                        #如果目標物中心點y小於手臂的中心點y，則控制手臂往下
                        
                        if self.mid[1] - self.arm_loc[1] >= 0:
                            self.arm.a[1] = self.arm.a[1]+0.5
                        else:
                            self.arm.a[1] = self.arm.a[1]-0.5
                        self.arm.move(self.arm.a)

            except:
                pass

            cv2.imshow("frame", self.frame)
            cv2.namedWindow('img_all', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("img_all",imgStack_all)
            
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                self.arm.home()

                break

        cv2.destroyAllWindows()
        self.cap.release()
        self.car.close()
        self.arm.arm_exit()
        
        sys.exit()

#%%
def main():
    
    # weed_signal = queue.Queue()
    # car_signal = queue.Queue()
    # arm_signal = queue.Queue()

    car = Car()
    arm = Arm()
    cam = Cam(car,arm)
    cam.run()


if __name__=="__main__":
    main()