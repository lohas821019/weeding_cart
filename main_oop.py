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

#使用一般webcam
from predict import *
from motor import *
import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
from action import *


class Car():
    def __init__(self):
        self.s = motor_init()

    def move(self,s):
        self.motor_control(s,0)

    def close(self):
        self.s.close()


class Arm():
    def __init__(self):
        self.ans = self.arm_init()
    
    def home(self):
        self.arm_home()
    
class Yolov5_Model():
    def __init__(self):
        self.model_flag = False
        self.model = load_model()


class Cam():
    def __init__(self):
        self.hsvVals_r = {'hmin': 0, 'smin': 40, 'vmin': 41, 'hmax': 9, 'smax': 255, 'vmax': 255}
        self.hsvVals_g = {'hmin': 71, 'smin': 238, 'vmin': 0, 'hmax': 100, 'smax': 255, 'vmax': 255}

        self.cap = cv2.VideoCapture(2,cv2.CAP_DSHOW)

def main():
    cap = Cam()
    while cap.isOpened:
        _, frame = cap.read()
        cv2.imshow("frame", frame)


    
    



if __name__=="__main__":
    main()
    # weed_signal = queue.Queue()
    # car_signal = queue.Queue()

    # t = threading.Thread(target=worker, daemon=True)
    # t.start()
    # t1 = threading.Thread(target=car_moving,args=(s,), daemon=True)
    # t1.start()
    