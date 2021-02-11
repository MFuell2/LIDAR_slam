# bno055_test.py Simple test program for MicroPython bno055 driver

# Copyright (c) Peter Hinch 2019
# Released under the MIT licence.

import machine
import time
import math
from bno055 import *
from Kalman import KalmanFilter
import pymatrix as pym



class Pos():
    def __init__(self, t, acc_x, acc_y):
        # Current Characteristics after update
        self.t = t
        self.x = 0
        self.y = 0
        self.vel_x = 0
        self.vel_y = 0
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.miniscule = 0.3
        
        
    def update(self, acc_x, acc_y, t):
        if abs(acc_x - self.acc_x) <= self.miniscule:
            acc_x = 0
        if abs(acc_y - self.acc_y) <= self.miniscule:
            acc_y = 0
            
        # Get direction of movement
        if acc_x >= 0:
            acc_x_dir = 1
        else:
            acc_x_dir = -1
        if acc_y >= 0:
            acc_y_dir = 1
        else:
            acc_y_dir = -1
            
        self.vel_x += acc_x*(t-self.t)
        self.vel_y += acc_y*(t-self.t)
        self.x += (self.vel_x*(t-self.t)) + (0.5*(acc_x**2)*acc_x_dir)
        self.y += (self.vel_y*(t-self.t)) + (0.5*(acc_y**2)*acc_y_dir)

        self.acc_x = acc_x
        self.acc_y = acc_y
        self.t = t


 
# Pyboard hardware I2C
i2c = machine.I2C(1)

imu = BNO055(i2c, 0x28)
imu2 = BNO055(i2c, 0x29)
calibrated1 = False
calibrated2 = False

start_time = time.ticks_ms()/1000
update_time_prev_howLong = 0
update_time_howLong = start_time
initial_x = imu.lin_acc()[0]
initial_y = imu.lin_acc()[1]
initial_z = imu.lin_acc()[2]

x = pym.matrix([[initial_x],[0],[0],\
                [initial_y],[0],[0],\
                [initial_z],[0],[0]])

KF = KalmanFilter(x)

pos = Pos(start_time, initial_x, initial_y)

iterations = 20
i = 0
#xHist1 = [0,0,0]
#yHist1 = [0,0,0]
#zHist1 = [0,0,0]
#xHist2 = [0,0,0]
#yHist2 = [0,0,0]
#zHist2 = [0,0,0]
myTime1 = []
myTime2 = []
myTime3 = []
myTime4 = []
myTime5 = []
update_time_prev = 0
update_time = 0


print("Begin")
while i<iterations:
    update_time_prev_howLong = update_time_howLong
    update_time_howLong = time.ticks_ms()/1000
    myTime1.append(update_time_howLong - update_time_prev_howLong)
    #time.sleep(.01)

    #----------------------------------------
    oneT = imu.lin_acc()
    one = [-1*oneT[0],-1*oneT[1],oneT[2]]
    twoT = imu2.lin_acc()
    two = [twoT[0],twoT[1],twoT[2]]
    
    avg = [(one[0]+two[0])/2, (one[1]+two[1])/2, (one[2]+two[2])/2]

    
    #fourPtAvg1 = [(xHist1[0]+xHist1[1]+xHist1[2]+(one[0]))/4, (yHist1[0]+yHist1[1]+yHist1[2]+(one[1]))/4, (zHist1[0]+zHist1[1]+zHist1[2]+(one[2]))/4]
    #fourPtAvg2 = [(xHist2[0]+xHist2[1]+xHist2[2]+(two[0]))/4, (yHist2[0]+yHist2[1]+yHist2[2]+(two[1]))/4, (zHist2[0]+zHist2[1]+zHist2[2]+(two[2]))/4]
    #fourPtAvg  = [(fourPtAvg1[0]+fourPtAvg2[0])/2, (fourPtAvg1[1]+fourPtAvg2[1])/2, (fourPtAvg1[2]+fourPtAvg2[2])/2]
    
    #fourPtAvg = [(xHist[0]+xHist[1]+xHist[2]+(xAvgTrun))/4, (yHist[0]+yHist[1]+yHist[2]+(yAvgTrun))/4, (zHist[0]+zHist[1]+zHist[2]+(zAvgTrun))/4]
    
    #xHist1[0] = xHist1[1]
    #xHist1[1] = xHist1[2]
    #xHist1[2] = one[0]
    
    #yHist1[0] = yHist1[1]
    #yHist1[1] = yHist1[2]
    #yHist1[2] = one[1]
    
    #zHist1[0] = zHist1[1]
    #zHist1[1] = zHist1[2]
    #zHist1[2] = one[2]
    #------------------
    #xHist2[0] = xHist2[1]
    #xHist2[1] = xHist2[2]
    #xHist2[2] = two[0]
    
    #yHist2[0] = yHist2[1]
    #yHist2[1] = yHist2[2]
    #yHist2[2] = two[1]
    
    #zHist2[0] = zHist2[1]
    #zHist2[1] = zHist2[2]
    #zHist2[2] = two[2]
    #-----------------------------------------
    update_time_prev_howLong = update_time_howLong
    update_time_howLong = time.ticks_ms()/1000
    myTime2.append(update_time_howLong - update_time_prev_howLong)
    
    update_time2 = time.ticks_ms()/1000
    acc_x = avg[0]#fourPtAvg[0]#imu.lin_acc()[0]
    acc_y = avg[1]#fourPtAvg[1]#imu.lin_acc()[1]
    acc_z = avg[2]#fourPtAvg[2]#imu.lin_acc()[2]
    dt = update_time2 - pos.t
    #myTime.append(dt)
    
    update_time_prev_howLong = update_time_howLong
    update_time_howLong = time.ticks_ms()/1000
    myTime3.append(update_time_howLong - update_time_prev_howLong)
    
    smoothed = KF.update(dt,[acc_x, acc_y, acc_z]) # Get smoothed acceleration from Kalman Filter
    
    update_time_prev_howLong = update_time_howLong
    update_time_howLong = time.ticks_ms()/1000
    myTime4.append(update_time_howLong - update_time_prev_howLong)
    
    smooth_acc_x = float(smoothed[0][-1])
    smooth_acc_y = float(smoothed[3][-1])
    smooth_acc_z = float(smoothed[6][-1])
    
    update_time_prev_howLong = update_time_howLong
    update_time_howLong = time.ticks_ms()/1000
    myTime5.append(update_time_howLong - update_time_prev_howLong)
    
    pos.update(smooth_acc_x, smooth_acc_y, update_time)   # Update position
    
    #data = [(update_time),(smooth_acc_x), (smooth_acc_y), (pos.vel_x), (pos.vel_y), (pos.x), (pos.y)]
    #print('     Time,     AccX,     AccY,     VelX,     VelY,        X,        Y|')
    #print('{:9.3f} {:9.3f} {:9.3f} {:9.3f} {:9.3f} {:9.3f} {:9.3f}|\n'.format(*data))

    i+=1

print(' ')
for i in range(len(myTime1)):
    #print('index: ', str(i), ' Time dt 1:', str(myTime1[i]), ' Time dt 2:', str(myTime2[i]), ' Time dt 3:', str(myTime3[i]), ' Time dt 4:', str(myTime4[i]), ' Time dt 5:', str(myTime5[i]))
    print('index: {:2.0f} Time 1: {:6.4f} Time 2: {:6.4f} Time 3: {:6.4f} Time 4: {:6.4f} Time 5: {:6.4f}'.format(i, myTime1[i], myTime2[i], myTime3[i], myTime4[i], myTime5[i]))
    


 

