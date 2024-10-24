from lib.imu import MPU6050
from time import sleep
from machine import Pin, I2C
import matplotlib.pyplot as plt
from datetime import datetime
import time

def update_subplot(subplot_ax, X, Y):
    subplot_ax.plot(X, Y)
    plt.draw()

##initialise I2C pin
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=200000)   #initiate sda and scl pin for MPU6050
imu = MPU6050(i2c)  #initiate MPU6050 class

##initialise matplot lib graphs
fig, ax= plt.subplots(2,3)

ax[0, 0].set_title('Accel X')
ax[0, 1].set_title('Accel Y')
ax[0, 2].set_title('Accel Z')

ax[1, 0].set_title('Gyro X')
ax[1,1].set_title('Gyro Y')
ax[1,2].set_title('Gyro Z')

while True:
    ax = []
    ay = []
    az = []
    
    gx = []
    gy = []
    gz = []
    
    time_lst= []
    
    for i in range(0, 50):
        ax.append(round(imu.accel.x, 2))  #extracting information from MPU6050 class methods
        ay.append(round(imu.accel.y, 2))
        az.append(round(imu.accel.z, 2))
        
        gx.append(round(imu.gyro.x))
        gy.append(round(imu.gyro.y))
        gz.append(round(imu.gyro.z))
        time_lst.append(datetime.now())
        
    ax[0, 0].plot()
    
    time.sleep(0.1)
        