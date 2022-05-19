# -*- coding: utf-8 -*-
"""
Created on Thu May 19 11:11:06 2022

@author: Jason
"""


import os
os.chdir(r'C:\Users\Jason\Documents\GitHub\weeding_cart')

import threading
import time

#使用一般webcam
from predict import *
import cv2

get_model_label = 1

if get_model_label:
    global model 
    model = load_model()
    get_model_label = 0

def worker():
    while True:
        item = q.get()
        if item is None:
            print("ending the worker")
            break

        print(item)
        arm_move(item)
        q.task_done()

ans = arm_init()
q = queue.Queue()
threading.Thread(target=worker, daemon=True).start()
#%%
cap = cv2.VideoCapture(0)
hw = []

while cap.isOpened():
    _, frame = cap.read()
    
    if hw == []:
        h = frame.shape[0]
        w = frame.shape[1]
        hw.append((h,w))

    roi = frame[int(h/2):h,0:w]
    results_roi = model(roi, size=640)  # includes NMS
    results_roi.pred
    data = results_roi.pandas().xyxy[0]    # includes NMS

    try:
        for i in range(0,len(data)):
            data = data.iloc[i]
            cv2.rectangle(roi, (int(data.xmin), int(data.ymin)), (int(data.xmax), int(data.ymax)), (0, 0, 255), 2)
            mid = ((data.xmin + data.xmax)/2,(data.ymin + data.ymax)/2)
            print(mid)
            cv2.circle(roi,(int(mid[0]),int(mid[1])), 15, (0, 0, 255), -1)
            
            if data_g[0] - data_r[0] > 0:
                a[0] = a[0]+1
                q.put(a)
            else:
                a[0] = a[0]-1
                q.put(a)
            
            if data_g[1] - data_r[1] > 0:
                a[1] = a[1]+1
                q.put(a)
            else:
                a[1] = a[1]-1
                q.put(a)
            
    except:
        pass

    cv2.imshow("roi", roi)
    cv2.imshow("frame", frame)
    

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        q.join()
        q.put(None)
        arm_home()
        time.sleep(2)
        break





cv2.destroyAllWindows()
cap.release()
arm_exit()
