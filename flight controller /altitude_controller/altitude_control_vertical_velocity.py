from math import sin, cos, tan

class attitude_control(): #class for attitude control with hcsr04 sensor
    
    def __init__(self, attitude_trigger, attitude_echo, kp, ki, kd, ki_limit):  #starting attitude_control instance by instantiating PID_controller class and hcsr04 class
        self.PID_controller = PID_controller.__init__(kp, ki, kd, ki_limit)
        self.hcsr04 = hcsr04(attitude_trigger, attitude_echo)
        self.vertical_velo = 0
        
      
    def attitude_respond(self, sp): #returns coordinated PID response for motor input
      attitude_pv = self.hcsr04.measure()
      return self.PID_controller.respond(sp, attitude_pv)
  
    def vertical_velocity_measure(self, imu_inst):
        ax, ay, az = imu_inst.measure_acc()
        angle_roll, angle_pitch = imu_inst.measure_angle()
        acc_z_intertial = sin(angle_pitch)*ax + cos(angle_pitch)*sin(angle_roll)*ay+cos(angle_pitch)*cos(angle_roll)*az
        acc_z_intertial = (acc_z_intertial-1)*9.81*100
        self.vertical_velo += acc_z_intertial*cycle_time_seconds
        return self.vertical_velo
      
      