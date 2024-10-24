from umatrix import *
from math import *

cycle_time_seconds = 1/250

class horizontal_kalman():
    def __init__(self):
        self.S = matrix([0], [0])  #state space matrix
        self.F = matrix([1, cycle_time_seconds], [0, 1])  #state transition matrix
        self.G = matrix(0.5*cycle_time_seconds**2, cycle_time_seconds)    #control matrix
        self.H = matrix([1, 0])   #observation matrix
        self.I = matrix([1,0], [0,1]) #identity matrix
        self.Q = self.G*self.G.transpose*10**2  #process uncertinaty
        self.R = 30**2  #measurement uncertainty
        self.P = matrix([0,0], [0,0]) #predicted uncretainty
        
        self.disp = 0
        self.L =0
        self.K = 0
        
       
    def kalman_filter(self, acc, velo):
        self.S = self.F*self.S + self.G*acc #equation 4.3.2.1
        self.P = self.F*self.P*self.F.transpose+self.Q  #equation 4.3.2.2
        self.L = self.H*self.P*self.H.transpose+self.R  #equation 4.3.2.3
        self.K = self.P*self.H.transpose*self.L.inverse #equation 4.3.2.4
        
        self.disp+=velo*cycle_time_seconds  #numeric integration to calulcate displacement
        
        ############ Update statespace matrix and predicted state #################
        self.S = self.S + self.K*(self.disp-self.H*self.S)   #equation 4.3.2.5
        disp_kalman = self.S[0][0]
        velo_kalman = self.S[1][0]
        
        self.P = (self.I-self.K*self.H)*self.P  #equation 4.3.2.6
        
        return disp_kalman, velo_kalman
    
class horizontal_measure():
    
    def __init__(self, imu_inst):
        self.imu_inst = imu_inst
        
        self.horzontal_kalman_x = horizontal_kalman()
        self.horizontal_kalman_y = horizontal_kalman()
    
    def measure_inertial_acc(self):   #method to measure inertial horizontal velocity
        ax, ay, az = self.imu_inst.measure_acc()   #measure acceleration with aggregated object imu_inst of the mpu_measure class
        angle_roll, angle_pitch, angle_yaw = self.imu_inst.measure_angle()   #measure roll, pitch and yaw anngle with imu_inst
        
        acc_x_inertial = -sin(angle_pitch)*az + cos(angle_pitch)*sin(angle_yaw)*ay+cos(angle_yaw)*cos(angle_pitch)*ax  #calculate inertial acceleration in the x axis
        
        acc_y_inertial = -sin(angle_roll)*az + cos(angle_roll)*sin(angle_yaw)*ax+cos(angle_yaw)*cos(angle_roll)*ay #calculate inertail acceleration in the y axis
        
        acc_x_inertial = (acc_x_inertial-1)*9.81*100  #convert measurements to cms^-2
        acc_y_inertial = (acc_y_inertial-1)*9.81*100
        
        return acc_x_inertial, acc_y_inertial   #return horizontal velocity

    def measure_horizontal_velo(self):
        velo_x = 0
        velo_y = 0
        
        return velo_x, velo_y
    def kalman_measure(self):
        velo_x, velo_y = self.measure_horizontal_velo
        acc_x_inertial, acc_y_inertial = self.measure_inertial_acc()
        
        disp_kalman_x, velo_kalman_x = self.horzontal_kalman_x.kalman_filter(acc_x_inertial, velo_x)
        disp_kalman_y, velo_kalman_y = self.horizontal_kalman_y.kalman_filter(acc_y_inertial, velo_y)

        return disp_kalman_x, velo_kalman_x, disp_kalman_y, velo_kalman_y
class horizontal_velocity():
    
    def __init__(self):
        pass
        self.velo_x_inertial = 0   #start horizontal velocity as 0 
        self.velo_y_inertial = 0 


    def measure_inertial_acc(self, imu_inst):   #method to measure inertial horizontal velocity
        ax, ay, az = imu_inst.measure_acc()   #measure acceleration with aggregated object imu_inst of the mpu_measure class
        angle_roll, angle_pitch, angle_yaw = imu_inst.measure_angle()   #measure roll, pitch and yaw anngle with imu_inst
        
        acc_x_intertial = -sin(angle_pitch)*az + cos(angle_pitch)*sin(angle_yaw)*ay+cos(angle_yaw)*cos(angle_pitch)*ax  #calculate inertial acceleration in the x axis
        
        acc_y_intertial = -sin(angle_roll)*az + cos(angle_roll)*sin(angle_yaw)*ax+cos(angle_yaw)*cos(angle_roll)*ay #calculate inertail acceleration in the y axis
        
        acc_x_intertial = (acc_x_intertial-1)*9.81*100  #convert measurements to cms^-2
        acc_y_intertial = (acc_y_intertial-1)*9.81*100
        
        return acc_x_intertial, acc_y_intertial   #return horizontal velocity
        



A = matrix([1, 2, 3], [4, 5, 6], [7, 8, 9])
M = matrix([12, 23, 31], [40, 50, 60], [71, 87, 98])

print(A-M)
