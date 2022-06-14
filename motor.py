# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 14:39:33 2022

@author: Jason
"""

import serial
import time

def motor_init():
    #右側輪胎
    COM_PORT1 = 'COM3'
    baudRate = 9600
    ser1 = serial.Serial(COM_PORT1, baudRate, timeout=0.5)
    return ser1
    
def motor_control(ser1,choice):
    try:
        while True:
            # 接收用戶的輸入值並轉成小寫
            # choice = input('0/1/2/3/4 --> 0停止 /1往前 /2往後 /3向前往左 /4向前往右 /q 離開')

            if choice == 'q':
                ser1.write(0)
                time.sleep(0.5)
                break

            elif (choice.isnumeric()): 
                if int(choice) == 0:
                    ser1.write(0)
    
                elif int(choice) == 1:
                    ser1.write(1)
                    
                elif int(choice) == 2:
                    ser1.write(2)
                    
                elif int(choice) == 3:
                    ser1.write(3)
                    
                elif int(choice) == 4:
                    ser1.write(4)
    
                else :
                    print('0/1/2/3/4 --> 0停止 /1往前 /2往後 /3向前往左 /4向前往右 /q 離開')
                    pass
    
    finally:
        ser1.close()
        print('各串口已關閉')