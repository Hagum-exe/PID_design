from lib.imu import MPU6050
from time import sleep
from machine import Pin, I2C
import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

interval = 20

bias_size = 10

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=200000)   #initiate sda and scl pin for MPU6050
imu = MPU6050(i2c)  #initiate MPU6050 class

angle_roll_sum = 0
angle_pitch_sum = 0


angle_roll_temp = 0
angle_pitch_temp = 0

kalman_roll = 0
kalman_unvertainty_roll = 2*2
kalman_pitch = 0
kalman_unvertainty_pitch = 2*2

def animate():
    xs = [0]
    ys = []
    
    x_sum = 0
    for i in range(0, 50):
        ax = round(imu.accel.x, 2)  #read acceleration along axis
        ay = round(imu.accel.y, 2)
        az = round(imu.accel.z, 2)
        gx = round(imu.gyro.x)  #read roll rate along axis
        gy = round(imu.gyro.y)
        gz = -round(imu.gyro.z)
        
        angle_roll = 0
        angle_pitch = 0
    
        try:
            angle_roll = math.atan(ay/(math.sqrt(ax**2+az**2)))*(180/math.pi)
            angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))*(180/math.pi)

            angle_roll_temp = angle_roll
            angle_pitch_temp = angle_pitch
            
            kalman_roll, kalman_uncertainty = kalman_1d(kalman_roll, kalman_uncertainty, gx, angle_roll)
            kalman_pitch, kalman_uncertainty = kalman_1d(kalman_pitch, kalman_uncertainty, gy, angle_pitch)
            x_sum += interval
            xs.append(x_sum)
        
        except:
            angle_roll = angle_roll_temp
            angle_pitch = angle_pitch_temp
    
        ys.append(angle_roll)
        
        x_sum += interval
        xs.append(x_sum)
        
    ax1.clear()
    ax1.plot(xs, ys)
        
        

def kalman_1d(kalman_state, kalman_uncertainty, kalman_input, kalman_measurement):
    kalman_state = kalman_state+0.004*kalman_input
    kalman_uncertainty = kalman_uncertainty + 0.004*0.004*4*4
    kalman_gain = kalman_uncertainty / (kalman_uncertainty + 3*3)
    kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state)
    kalman_uncertainty = (1-kalman_gain)*kalman_uncertainty
    
    return kalman_state, kalman_uncertainty
    
ani = animation.FuncAnimation(fig, animate, interval = 1000)
plt.show()
    


