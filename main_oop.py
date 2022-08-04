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
import sys
import serial
import numpy as np
import innfos
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder

#%% CAR
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
        data_s = np.array('1').tobytes()
        self.ser1.write(data_s)
        print('車輛前進')
        
    def backward(self):
        data_s = np.array('2').tobytes()
        self.ser1.write(data_s)
        print('車輛後退')
        
    def stop(self):
        data_s = np.array('0').tobytes()
        self.ser1.write(data_s)
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


#%% ARM
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
        
    def arm_control1(self,temp_grass_A,arm_loc):
        if self.a[0] > 0 :
            self.a[0] = 0
        elif self.a[0] < -18 :
            self.a[0] = -18
            
        if temp_grass_A[0] - arm_loc[0] <= 0:
            self.a[0] = self.a[0]+0.5
        else:
            self.a[0] = self.a[0]-0.5

    def arm_control_by_red(self,temp_grass_B,arm_loc_web):
    
            if self.a[0] > 0 :
                self.a[0] = 0
            elif self.a[0] < -18 :
                self.a[0] = -18

            if self.a[3] > 10 :
                self.a[3] = 10
            elif self.a[3] <-10 :
                self.a[3] = -10
    
            if temp_grass_B[0] - arm_loc_web[0] <= 0:
                self.a[0] = self.a[0]+0.5
            else:
                self.a[0] = self.a[0]-0.5
    
            if temp_grass_B[1] - arm_loc_web[1] >= 0:
                self.a[3] = self.a[3]+0.5
            else:
                self.a[3] = self.a[3]-0.5

#%% YOLO
class Yolov5_Model():
    instance = None
    model_flag = False
    
    def __init__(self):
        
        if Yolov5_Model.model_flag:
            return
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./arm_best.pt', force_reload=True) 
        # self.model = torch.hub.load(r'C:\Users\zanrobot\Documents\Github\yolov5', 'custom', path=r'C:\Users\zanrobot\Documents\Github\yolov5/arm_best.pt', source='local')
        self.model = torch.hub.load(r'C:\Users\Jason\Documents\GitHub\yolov5', 'custom', path=r'C:\Users\Jason\Documents\GitHub\weeding_cart/arm_best.pt', source='local')

        self.model.eval()
        Yolov5_Model.model_flag = True
            
    def predict(self,frame):
        self.results_roi = self.model(frame, size=640)
        self.results_roi.pred
        return self.results_roi
    
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance
#%% CAM
class Cam():
    def __init__(self,car,arm):
        self.car = car
        self.arm = arm
        
        self.car_falg = 1
        self.arm_falg = 1
        
        self.hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
        self.hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
        
        self.hsvVals_g_web = {'hmin': 39, 'smin': 95, 'vmin': 0, 'hmax': 104, 'smax': 214, 'vmax': 255}
        self.hsvVals_r_web = {'hmin': 0, 'smin': 112, 'vmin': 43, 'hmax': 9, 'smax': 255, 'vmax': 255}
        
        self.cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
        self.cap_web = cv2.VideoCapture(2,cv2.CAP_DSHOW)

        self.model = Yolov5_Model()
        
        self.myColorFinder = ColorFinder()
        self.myColorFinder1 = ColorFinder()

        self.arm_loc = None
        self.mid = None
        
        self.grass_flag_A = 1
        self.grass_flag_B = 1
        self.first = 1


    def run(self):
        while self.cap.isOpened:
            
            _, self.frame = self.cap.read()
            _, self.frame_web = self.cap_web.read()
            
            self.results_roi= self.model.predict(self.frame)
            self.results_roi_web= self.model.predict(self.frame_web)
            
            self.data = self.results_roi.pandas().xyxy[0]

            #cam1尋找手臂的位置
            if self.data.name.any():
                if self.results_roi.pandas().xyxy[0].name[0] == 'arm':
                    self.data = self.results_roi.pandas().xyxy[0]
                    self.arm_loc = ((self.data.xmin + self.data.xmax)/2,(self.data.ymin + self.data.ymax)/2)
                    self.arm_loc = [int(self.arm_loc[0][0]),int(self.arm_loc[1][0])]
                    cv2.circle(self.frame,(int(self.arm_loc[0]),int(self.arm_loc[1])), 8, (0, 255, 255), -1)
                else:
                    self.arm_loc = None
                    
            #cam1尋找草的位置
            self.imgColor_g,self.mask_g = self.myColorFinder.update(self.frame,self.hsvVals_g)
            self.imgContour_g,self.contours_g = cvzone.findContours(self.frame, self.mask_g,minArea=500)
            self.imgStack_all = cvzone.stackImages([self.imgColor_g,self.imgContour_g],2,0.5)
            
            try:
                if self.contours_g:
                    self.mid = self.contours_g[0]['center']
                    # cv2.circle(frame,(int(mid[0]),int(mid[1])), 8, (0, 0, 255), -1)
                    if self.grass_flag_A:
                        self.temp_grass_A = self.mid
                        self.grass_flag_A = 0
                    cv2.circle(self.frame,(int(self.temp_grass_A[0]),int(self.temp_grass_A[1])), 8, (0, 0, 255), -1)
                else:
                    self.temp_grass_A = None
            except:
                pass
            
            #cam2找草的位置
            self.imgColor_g_web,self.mask_g_web = self.myColorFinder1.update(self.frame_web,self.hsvVals_g_web)
            self.imgContour_g_web,self.contours_g_web = cvzone.findContours(self.frame_web, self.mask_g_web,minArea=500)
            # imgStack_all_web = cvzone.stackImages([ imgColor_g_web,imgColor_g_web],2,0.5)
        
            try:
                if self.contours_g_web:
                    self.mid_web = self.contours_g_web[0]['center']
                    # cv2.circle(frame_web,(int(mid_web[0]),int(mid_web[1])), 8, (0, 0, 255), -1)
                    if self.grass_flag_B:
                        self.temp_grass_B = self.mid_web
                        self.grass_flag_B = 0
                    cv2.circle(self.frame_web,(int(self.temp_grass_B[0]),int(self.temp_grass_B[1])), 8, (0, 0, 255), -1)
                else:
                    self.temp_grass_B = None
            except:
                pass
            
            #cam2找手臂的位置
            self.imgColor_r_web, self.mask_r_web = self.myColorFinder1.update(self.frame_web,self.hsvVals_r_web)
            self.imgContour_r_web, self.contours_r_web = cvzone.findContours(self.frame_web, self.mask_r_web,minArea=500)
            # imgStack_allr_web = cvzone.stackImages([imgColor_r_web,imgColor_r_web],2,0.5)
            
            try:
                if self.contours_r_web:
                    self.arm_loc_web = self.contours_r_web[0]['center']
                    cv2.circle(self.frame_web,(int(self.arm_loc_web[0]),int(self.arm_loc_web[1])), 8, (0, 255, 255), -1)
                else:
                    self.arm_loc_web = None
            except:
                pass
            
            cv2.waitKey(1)
            cv2.imshow("frame", self.frame)
            cv2.imshow("frame_web", self.frame_web)
        
        
            if self.temp_grass_A and self.arm_loc:
                self.car.stop()
                
                #計算手臂與雜草距離
                self.sx = pow(abs((self.arm_loc[0]-self.temp_grass_A[0])),2)
                self.sy = pow(abs((self.arm_loc[1]-self.temp_grass_A[1])),2)
                self.now_dist = int(abs((self.sx-self.sy))**0.5)
                print(f'now_dist = {self.now_dist}')

                if self.temp_grass_B and self.arm_loc_web:
                 #計算手臂與雜草距離
                 self.sx = pow(abs((self.arm_loc_web[0]-self.temp_grass_B[0])),2)
                 self.sy = pow(abs((self.arm_loc_web[1]-self.temp_grass_B[1])),2)
                 self.now_dist_web = int(abs((self.sx-self.sy))**0.5)
                 print(f'now_dist_web = {self.now_dist_web}')

                if self.first and self.now_dist!=0:
                    self.first = 0
                    n = 0
                    n = n + 1
                    print(f'n = {n}')
                    
                    data1 = []
                    data1.append(self.now_dist)
                    print(f'data1 = {data1}')
        
                if self.now_dist >= 50:
                    case = 0
                else:
                    case = 1
                    
                if n == 5:
                    if self.data1[n-1]-self.data1[n-2]<=3 and self.data1[n-2]-self.data1[n-3]<=3 and self.data1[n-3]-self.data1[n-4]<=5:
                        self.first = 1
                        
                        if case == 0:
                            self.arm.control1(self.temp_grass_A,self.arm_loc)
                            self.arm.move(self.arm.a)
                            
                        elif case == 1:
                            self.arm.arm_control_by_red(self.temp_grass_B,self.arm_loc_web)
                            self.arm.move(self.arm.a)
                            try:
                                if self.now_dist_web <= 20:

                                    self.arm.home()
                                    
                                    self.arm.a = [-18,0,0,0,0]
                                    self.mid = None
                                    self.arm_loc = None
                                    case = 0
                                    self.grass_flag_A = 1
                                    self.grass_flag_B = 1
                                    
                                    self.car.backward()
                            except:
                                pass
                elif n > 5:
                    self.first = 1
                    
             #如果沒抓到草，車子移動
            else:
                self.car.backward()

            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                self.arm.home()
                self.car.close()
                break

        cv2.destroyAllWindows()
        self.cap.release()
        self.arm.arm_exit()
        sys.exit()

#%%
def main():

    car = Car()
    arm = Arm()
    cam = Cam(car,arm)
    cam.run()


if __name__=="__main__":
    main()