# -*- coding: utf-8 -*-
"""
Created on Wed Feb 23 13:55:36 2022

@author: Jason
"""
import innfos
import time

actuID = [0x01, 0x02, 0x03, 0x04, 0x05]

def arm_init():
    statusg = innfos.handshake()
    data = innfos.queryID(6)
    innfos.enableact(actuID)
    time.sleep(1)
    innfos.trapposmode(actuID) #梯形模式


def arm_exit():
    innfos.disableact(actuID)

def arm_home():
    innfos.setpos(actuID, [0, 12, 15, 0, 0])
    time.sleep(1)


def arm_move(pos):
    try:
        innfos.setpos(actuID, pos)
        time.sleep(1)
    except:
        pass
    
def arm_show_nowpos():
    nowpos = innfos.readpos(actuID)
    print(nowpos)
    time.sleep(1)

def arm_show_limitpos():
    limitpos = innfos.readposlimit(actuID) #讀取極限位置設置
    print(limitpos)
    time.sleep(1)

def arm_set_limitpos():
    innfos.poslimit(actuID,[15,-15],[15,-15])

# arm_show_nowpos()
# arm_show_limitpos()

# arm_move([0,-6, -30, -15, 0])
 