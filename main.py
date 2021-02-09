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
#         self.vel_x = new_vel_x
#         self.vel_y = new_vel_y
#         self.x = new_x
#         self.y = new_y
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.t = t
        


# Data storage setup
# filename = 'data/'+str(time.ticks_ms()/1000)+'_data.txt'
header = 'Time, \taccX, \taccY, \tvelX, \tvelY, \tX, \tY'
# with open(filename,'w') as f:
#     f.write(header)
 
 
# Pyboard hardware I2C
i2c = machine.I2C(1)

time.sleep(2)
imu = BNO055(i2c)
time.sleep(2)
calibrated = False


start_time = time.ticks_ms()/1000
initial_x = imu.lin_acc()[0]
initial_y = imu.lin_acc()[1]
initial_z = imu.lin_acc()[2]

x = pym.matrix([[initial_x],[0],[0],\
                [initial_y],[0],[0],\
                [initial_z],[0],[0]])

KF = KalmanFilter(x)

pos = Pos(start_time, initial_x, initial_y)



iterations = 1000
i = 0
while i<iterations:
    time.sleep(.01)
    update_time = time.ticks_ms()/1000
    acc_x = imu.lin_acc()[0]
    acc_y = imu.lin_acc()[1]
    acc_z = imu.lin_acc()[2]
    dt = update_time - pos.t
    
    # Get smoothed acceleration from Kalman Filter
    smoothed = KF.update(dt,[acc_x, acc_y, acc_z])
    smooth_acc_x = float(smoothed[0][-1])
    smooth_acc_y = float(smoothed[3][-1])
    smooth_acc_z = float(smoothed[6][-1])
    
    # Update position
    pos.update(smooth_acc_x, smooth_acc_y, update_time)
#     print('dt:', dt, '\tacc_x:', acc_x, '\tsmX:', smooth_acc_x, '\tacc_y:', acc_y,'\tsmY:', smooth_acc_y)
    
    
    
    data = [str(update_time),str(smooth_acc_x), str(smooth_acc_y),
            str(pos.vel_x), str(pos.vel_y), str(pos.x), str(pos.y)]

    line = ','.join(data)+'\n'
#         f.write(line)
    print(header)
    print(data,'\n')
    i+=1  
        
        
#             if not calibrated:
#             calibrated = imu.calibrated()
#             print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
#         print('Temperature {}°C'.format(imu.temperature()))
#         print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
#         print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
#         print('\nAccel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
#         print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
#         print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
#         print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
#         print('Quat1     {:4.0f} Quat2 {:4.0f} Quat3 {:4.0f}'.format(*imu.quaternion()))
    
print('Time taken:',str(update_time-start_time))

    
print(imu.error_count, '\\',iterations, 'read errors')



 
