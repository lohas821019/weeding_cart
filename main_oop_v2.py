# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 09:42:38 2022
@author: Jason
"""
import os
#os.chdir(r'C:\Users\zanrobot\Desktop\weeding_cart')
os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

import threading , queue
import time
import sys
import serial
import numpy as np
import innfos
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
import torch

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
        self.ser = serial.Serial(self.COM_PORT, self.baudRate, timeout=0.5)
        self.state = 0
        print('車輛初始化成功')
        Car.Car_flag = True
        
            
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance

    def forward(self):
        data_s = np.array('1').tobytes()
        self.ser.write(data_s)
        print('車輛前進')
        
    def backward(self):
        data_s = np.array('2').tobytes()
        self.ser.write(data_s)
        print('車輛後退')
        
    def stop(self):
        data_s = np.array('0').tobytes()
        self.ser.write(data_s)
        print('車輛停止')

    def close(self):
        self.ser.close()
        print('車輛關閉')
    
    def motor_on(self):
        data_s = np.array('6').tobytes()
        self.ser.write(data_s)
        print('除草馬達打開')        

    def motor_off(self):
        data_s = np.array('5').tobytes()
        self.ser.write(data_s)
        print('除草馬達關閉')  
    
    def response(self):
        # while self.ser.in_waiting:
        mcu_feedback = self.ser.readline()
        mcu_feedback = mcu_feedback.decode("utf-8") # 接收回應訊息並解碼
        mcu_feedback = mcu_feedback.strip()
        # time.sleep(0.1)
        return mcu_feedback
    
    def response1(self):
        tdata = self.ser.read()           # Wait forever for anything
        # time.sleep(0.3)              # Sleep (or inWaiting() doesn't give the correct value)
        data_left = self.ser.inWaiting()  # Get the number of characters ready to be read
        tdata += self.ser.read(data_left)
        tdata = tdata.decode('utf-8').strip()
        return tdata


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
        innfos.setpos(self.actuID,self.a)
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
            self.a[0] = self.a[0]+1
        else:
            self.a[0] = self.a[0]-1
        
        print(self.a)

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
                self.a[0] = self.a[0]+1
            else:
                self.a[0] = self.a[0]-1
    
            if temp_grass_B[1] - arm_loc_web[1] >= 0:
                self.a[3] = self.a[3]+1
            else:
                self.a[3] = self.a[3]-1
            print(self.a)

#%% Yolov5_Model_Arm
class Yolov5_Model_Arm():
    instance = None
    model_flag = False
    
    def __init__(self):
        
        if Yolov5_Model_Arm.model_flag:
            return
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./arm_best.pt', force_reload=True) 
        self.model = torch.hub.load(r'C:\Users\zanrobot\Documents\Github\yolov5', 'custom', path=r'C:\Users\zanrobot\Documents\Github\weeding_cart/weights/arm.pt', source='local')
        # self.model = torch.hub.load(r'C:\Users\Jason\Documents\GitHub\yolov5', 'custom', path=r'C:\Users\Jason\Documents\GitHub\weeding_cart/arm_best.pt', source='local')

        self.model.eval()
        Yolov5_Model_Arm.model_flag = True
            
    def predict(self,frame):
        self.results_roi = self.model(frame, size=640)
        self.results_roi.pred
        return self.results_roi
    
    def __new__(cls, *args, **kwargs):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
        return cls.instance
#%% Yolov5_Model_Grass
class Yolov5_Model_Grass():
    instance = None
    model_flag = False
    
    def __init__(self):
        
        if Yolov5_Model_Grass.model_flag:
            return
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./arm_best.pt', force_reload=True) 
        self.model = torch.hub.load(r'C:\Users\zanrobot\Documents\Github\yolov5', 'custom', path=r'C:\Users\zanrobot\Documents\Github\weeding_cart/weights/fake_grass.pt', source='local')
        # self.model = torch.hub.load(r'C:\Users\Jason\Documents\GitHub\yolov5', 'custom', path=r'C:\Users\Jason\Documents\GitHub\weeding_cart/arm_best.pt', source='local')

        self.model.eval()
        Yolov5_Model_Grass.model_flag = True
            
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
    def __init__(self,car,arm,model_arm,model_grass,q,q2):
        self.q = q
        self.q2 = q2
        self.car = car
        self.arm = arm
        self.model_arm = model_arm
        self.model_grass = model_grass

        self.hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
        self.hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}
        
        self.hsvVals_g_web = {'hmin': 39, 'smin': 95, 'vmin': 0, 'hmax': 104, 'smax': 214, 'vmax': 255}
        self.hsvVals_r_web = {'hmin': 0, 'smin': 112, 'vmin': 43, 'hmax': 9, 'smax': 255, 'vmax': 255}
    
        self.myColorFinder1 = ColorFinder()

        self.arm_loc = None
        self.mid = None
        
        self.grass_flag_A = 1
        self.grass_flag_B = 1
        self.temp_grass_A = None
        self.temp_grass_B = None
        
        self.limit_times = 0
        self.frame_black_count = 0
        
        self.n = 0
        self.data1 = []

        self.car.state = 0
        self.finished_flag = 0
        self.move_flag = 1
        self.car_set_flag = 1
        # self.roi = self.frame[100:420,220:420]

    def distance(self,x,y):
        sx = pow(abs((x[0]-y[0])),2)
        sy = pow(abs((x[1]-y[1])),2)
        now_dist = int(abs((sx-sy))**0.5)
        return now_dist
        
    def mid_point(self,data):
        arm_loc = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
        arm_loc = [int(arm_loc[0][0]),int(arm_loc[1][0])]
        return arm_loc
    
    def thread_job(self):
        t = threading.Thread(target=self.show_image)
        t.start()

    def show_image(self):
        self.cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
        self.cap_web = cv2.VideoCapture(1,cv2.CAP_DSHOW)

        while True:
            _, self.frame = self.cap.read()
            _, self.frame_web = self.cap_web.read()

            self.results_roi= self.model_arm.predict(self.frame)
            self.results_roi_g= self.model_grass.predict(self.frame)
            self.data0_a = self.results_roi.pandas().xyxy[0]
            self.data0_g = self.results_roi_g.pandas().xyxy[0]

            self.results_roi_web= self.model_arm.predict(self.frame_web)
            self.results_roi_web_g= self.model_grass.predict(self.frame_web)
            self.data1_a = self.results_roi_web.pandas().xyxy[0]
            self.data1_g = self.results_roi_web_g.pandas().xyxy[0]
            
            #%%cam1
            if self.data0_a.name.any():
                if self.results_roi.pandas().xyxy[0].name[0] == 'arm':
                    self.arm_loc = self.mid_point(self.data0_a)
                    cv2.circle(self.frame,(int(self.arm_loc[0]),int(self.arm_loc[1])), 8, (0, 255, 255), -1)
                else:
                    self.arm_loc = None
                    
            #cam1尋找草的位置
            if self.data0_g.name.any():
                if self.results_roi_g.pandas().xyxy[0].name[0] == 'grass':
                    self.mid = self.mid_point(self.data0_g)
                    if self.grass_flag_A:
                        self.temp_grass_A = self.mid
                        self.grass_flag_A = 0
                    cv2.circle(self.frame,(int(self.temp_grass_A[0]),int(self.temp_grass_A[1])), 8, (0, 0, 255), -1)
                else:
                    self.mid = None

            #%%
            #cam2
            self.imgColor_r_web, self.mask_r_web = self.myColorFinder1.update(self.frame_web,self.hsvVals_r_web)
            self.imgContour_r_web, self.contours_r_web = cvzone.findContours(self.frame_web, self.mask_r_web,minArea=500)
            try:
                if self.contours_r_web:
                    self.arm_loc_web = self.contours_r_web[0]['center']
                    cv2.circle(self.frame_web,(int(self.arm_loc_web[0]),int(self.arm_loc_web[1])), 8, (0, 255, 255), -1)
                else:
                    self.arm_loc_web = None
            except:
                pass
        
            #cam2尋找草的位置
            if self.data1_g.name.any():
                if self.results_roi_web_g.pandas().xyxy[0].name[0] == 'grass':
                    self.mid_web = self.mid_point(self.data1_g)
                    print(f'self.mid_web = {self.mid_web}')
                    if self.grass_flag_B:
                        self.temp_grass_B = self.mid_web
                        self.grass_flag_B = 0
                    cv2.circle(self.frame_web,(int(self.temp_grass_B[0]),int(self.temp_grass_B[1])), 8, (0, 0, 255), -1)
                else:
                    self.temp_grass_B = None
                
            cv2.waitKey(1)
            cv2.imshow("frame", self.frame)
            cv2.imshow("frame_web", self.frame_web)

            #%%
            #計算手臂與雜草距離
            if self.arm_loc and self.temp_grass_A :
                self.now_dist = self.distance(self.arm_loc,self.temp_grass_A)
                print(f'now_dist = {self.now_dist}')
            try:
                if self.temp_grass_B and self.arm_loc_web:
                    #計算手臂與雜草距離
                    self.now_dist_web = self.distance(self.arm_loc_web,self.temp_grass_B)
                    print(f'now_dist_web = {self.now_dist_web}')
            except:
                pass

            try:
                if self.now_dist >= 50:
                    case = 0
                    item = self.q.put((case,self.temp_grass_A,self.arm_loc))
                else:
                    case = 1
                    item = self.q.put((case,self.temp_grass_B,self.arm_loc_web))
            except:
                pass
            
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                self.arm.home()
                self.car.motor_off()
                self.car.close()
                break

        #%%
    def run(self):
        while True:
            self.limit_times = self.limit_times +1
            #如果返回指定代表有收到訊號，車子往前走了
            if self.move_flag:
                self.car.backward()
                self.move_flag = 0
                time.sleep(2)
                           
            if self.car.response() =='9': #車子回傳指令後
                self.car.stop()
                self.move_flag = 0
                
                #self.cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
                if self.q.size()>0:
                    case = self.q.get(timeout =0.01)

                    if case[0] == 0:
                        self.temp_grass_A,self.arm_loc = (case[1],case[2])
                        self.arm.arm_control1(self.temp_grass_A,self.arm_loc)
                        self.arm.move(self.arm.a)
                        
                    elif case == 1:
                        try:
                            self.temp_grass_B,self.arm_loc_web = (case[1],case[2])
                            self.arm.arm_control_by_red(self.temp_grass_B,self.arm_loc_web)
                            self.arm.move(self.arm.a)
                        except:
                            pass
                    try:
                        if self.q2.size()>0: 
                            self.now_dist_web = self.q2.get(timeout =0.01) 
                    except:
                        pass

                    if self.now_dist_web <= 40 or self.limit_times >=80:
                        self.arm.home()
                        self.arm.a = [-18,0,0,0,0]
                        self.mid = None
                        self.arm_loc = None
                        self.now_dist_web = 1000
                        self.limit_times = 0 
                        case = 0
                        self.grass_flag_A = 1
                        self.grass_flag_B = 1
                        self.finished_flag = 1
                        self.car_set_flag = 1
                        self.car.motor_off()
                        cv2.destroyAllWindows()
                else:
                    self.move_flag = 1     


#%%
def main():

    car = Car()
    arm = Arm()
    model_arm = Yolov5_Model_Arm()
    model_grass = Yolov5_Model_Grass()

    cam = Cam(car,arm,model_arm,model_grass,q,q2)
    time.sleep(5)
    cam.run()

    q = queue.Queue()
    q2 = queue.Queue()

if __name__=="__main__":
    main()