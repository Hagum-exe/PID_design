from machine import Pin, I2C
import utime
from toolkit import hcsr04, pmw3901
from pid_controller import PID_controller
from time import sleep
from main import cycle_time_seconds

#################### constants #############################
#######I2C constants ##########
nano_sda = 16
nano_scl =  17
#freq = 100000
msg_size = 15
addr = 1

####### traversal constants #######
max_speed = 0
min_speed = 0

##### attitude PID control constants #####
attitude_kp = 0
attitude_ki = 0
attitude_kd = 0

######### traversal PID control constants #######
ki_limit = 0

traversal_pitch_kp = 0
traversal_pitch_ki = 0 
traversal_pitch_kd = 0 

traversal_roll_kp = 0
traversal_roll_ki = 0
traversal_roll_kd = 0


###################### attitdue_control class ######################
class attitude_control(): #class for attitude control with hcsr04 sensor
    
    def __init__(self, attitude_trigger, attitude_echo, kp, ki, kd, ki_limit):  #starting attitude_control instance by instantiating PID_controller class and hcsr04 class
        self.PID_controller = PID_controller.__init__(kp, ki, kd, ki_limit)
        self.hcsr04 = hcsr04(attitude_trigger, attitude_echo)
        
      
    def attitude_respond(self, sp): #returns coordinated PID response for motor input
      attitude_pv = self.hcsr04.measure()
      return self.PID_controller.respond(sp, attitude_pv)


#################### traversal_speed class ########################
class traversal_speed():
    def __init__(self, max_speed, min_speed, pmw3901):
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.roll_PID_controller = PID_controller(traversal_roll_kp, traversal_roll_ki, traversal_roll_kd, ki_limit)
        self.pitch_PID_controller = PID_controller(traversal_pitch_kp, traversal_pitch_ki, traversal_pitch_kd, ki_limit)
        self.pmw3901 = pmw3901
    
    def normalise(self,value):
        return self.min_speed + ((self.max_speed - self.min_speed) * ((value - 0)))
    
    def traversal_response(self, target_speed_vector):  #speed vector is a list with 2 elements which specifies the desired traversal dirction. 
        actual_speed_vector = self.pmw3901.measure()
        pitch_sp = target_speed_vector[1]
        pitch_pv = self.normalise(actual_speed_vector[1])
        
        roll_sp = target_speed_vector[0]
        roll_pv = self.normalise(actual_speed_vector[0])
        
        
        pitch_resp = self.roll_PID_controller.respond(pitch_sp, pitch_pv)
        roll_resp = self.roll_PID_controller.respond(roll_sp, roll_pv)

        return pitch_resp, roll_resp
    

#################### traversal_displacement class #####################
class traversal_displacement():
    def __init__(self, max_speed, min_speed, nano_sda, nano_scl, addr, msg_size):
        self.pmw3901 = pmw3901(nano_sda, nano_scl, addr, msg_size)
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.roll_PID_controller = PID_controller(traversal_roll_kp, traversal_roll_ki, traversal_roll_kd, ki_limit)
        self.pitch_PID_controller = PID_controller(traversal_pitch_kp, traversal_pitch_ki, traversal_pitch_kd, ki_limit)
        self.target_displacement_vector = [0, 0]
    
    def normalise(self,value):
        return self.min_speed + ((self.max_speed - self.min_speed) * ((value - 0)))
    
    def traversal_respond(self, target_displacement_vector, actual_displacement_vector):  #displacement vector is a list with 2 elements which specifies the desired traversal displacement. 
        if self.target_displacement_vector != target_displacement_vector:
            self.target_displacement_vector = target_displacement_vector
            
            
        else:
            actual_speed_vector = self.pmw3901.measure()
            
            actual_displacement_vector[0] = actual_displacement_vector[0] + actual_speed_vector[0]*cycle_time_seconds 
            actual_displacement_vector[1] = actual_displacement_vector[1] + actual_speed_vector[1]*cycle_time_seconds 
            
            pitch_sp = target_displacement_vector[0]
            pitch_pv = self.normalise(actual_displacement_vector[0])
            
            roll_sp = target_displacement_vector[1]
            roll_pv = self.normalise(actual_displacement_vector[1])
            
            
            pitch_resp = self.roll_PID_controller.respond(pitch_sp, pitch_pv)
            roll_resp = self.roll_PID_controller.respond(roll_sp, roll_pv)
        
        return pitch_resp, roll_resp
    
    
