import time

from machine import Pin, PWM, I2C
from imu import MPU6050
#from obstacle_avoidance import obstacle_avoidance
#from traversal_attitude import attitude_control
#from pid_controller import PID_controller

import time
import machine
import math

import toolkit

############################## CONSTANTS ################################
################## system presets ###################
target_cycle_freq = 250 #target PID frequency in hz
cycle_time_seconds = 1/target_cycle_freq
cycle_time_us = int(round(cycle_time_seconds*1000000, 0))


######### max angle freedom ##########
#max_roll_angle = 90
#max_pitch_angle = max_roll_angle
#max_yaw_angle = 360

angle_yaw = 0
#yaw_current_position = 0

############ gyro constants ##########
calibration_size = 1000

angle_roll_temp = 0
angle_pitch_temp = 0
angle_yaw_temp = 0

################ PID constants ###################
############## RATE contants ###############
rate_roll_kp = 1.0
rate_roll_ki = 0.0
rate_roll_kd = 0.0
rate_pitch_kp = rate_roll_kp
rate_pitch_ki = rate_roll_ki
rate_pitch_kd = rate_roll_kd

rate_yaw_kp = 2
rate_yaw_ki = 12
rate_yaw_kd = 0.0

rate_i_term_limit = 400

############## ANGLE constants ##############
angle_roll_kp = 2
angle_roll_ki = 0
angle_roll_kd = 0
angle_pitch_kp = angle_roll_kp
angle_pitch_ki = angle_roll_ki
angle_pitch_kd = angle_roll_kd

angle_i_term_limit = 0

pid_decrease_ratio = 1



################ device pinouts ###############
######## MPU6050 ##########
mpu_i2c_sda = 0
mpu_i2c_scl = 1


######### LED #############
buzz_pin = 28
buzz_led = toolkit.buzz_led(buzz_pin)

######## ESC ###########
pwm_output_freq = 50
pwm_motor1 = 3 #rear right, cw
pwm_motor2 = 5 #front right, ccw
pwm_motor3 = 7 #rear left, ccw
pwm_motor4 = 9 #front left, cw


####### CRSF RX ##########
pwm_pin_1 = 2
pwm_pin_2 = 4
pwm_pin_3 = 6
pwm_pin_4 = 8
pwm_pin_5 = 10

############## setup pwm read from CRSF RX ############
pwm_channel_1 = Pin(pwm_pin_1, Pin.IN)
pwm_channel_2 = Pin(pwm_pin_2, Pin.IN)
pwm_channel_3 = Pin(pwm_pin_3, Pin.IN)
pwm_channel_4 = Pin(pwm_pin_4, Pin.IN)
pwm_channel_5 = Pin(pwm_pin_5, Pin.IN)

############## PWM output constants ################
pwm_min = 1000
pwm_max = 2000


############## throttle settings ###################
throttle_idle = 1150  #minimum throttle for four mototrs to spin up but not lift up
max_throttle = 1800  #maximum throttle limit
throttle_arm_upper_limit = 0.07


################################# CLASSES ###########################################
#################PID controller class#########################
class PID_controller:
  def __init__(self, kp, ki, kd, ki_limit):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.ki_limit = ki_limit
    
    self.E = 0
    self.I = 0
    self.sp = 0

  def respond(self, sp, pv):    #pv as current position being passed in, sp is set point
    if (sp != self.sp): #renew integral sum if new setpoint is given
      self.I = 0
      self.sp = sp
    err = sp - pv
    P = self.kp * err
    I = self.I + ((self.ki * err)/2) * cycle_time_seconds
    I = max(min(I, self.ki_limit), -self.ki_limit) # constrain within I-term limits, prevents integral windup
    D = self.kd * ((err - self.E)/cycle_time_seconds)
    self.E = err
    self.I = I
    return P + I + D

class mpu_measure():
    def __init__(self, mpu_i2c_sda, mpu_i2c_scl):
      i2c = I2C(0, sda=machine.Pin(mpu_i2c_sda), scl=machine.Pin(mpu_i2c_scl), freq=200000)
      self.imu = MPU6050(i2c)
      
      self.bias_rate_roll = 0
      self.bias_rate_pitch = 0
      self.bias_rate_yaw = 0
      
      self.kalman_roll_temp = 0
      self.kalman_pitch_temp = 0
      
      self.kalman_roll = 0
      self.kalman_pitch = 0
      
      self.kalman_uncertainty_roll = 2*2
      self.kalman_uncertainty_pitch = 2*2
        
        
    def calibrate_rate(self, calibration_size):
      self.bias_rate_roll = 0   #initiate all rotation rate biases as 0
      self.bias_rate_pitch = 0
      self.bias_rate_yaw = 0
      for i in range(0, calibration_size):  #take a sample of rotation rates of size specified in calibrartion_size
          gx = round(self.imu.gyro.x) 
          gy = round(self.imu.gyro.y) 
          gz = -round(self.imu.gyro.z) 

          self.bias_rate_roll += gx #sum up all rotation rates
          self.bias_rate_pitch += gy
          self.bias_rate_yaw += gz
      
      self.bias_rate_roll = self.bias_rate_roll / calibration_size  #calculate mean of rotation rates
      self.bias_rate_pitch = self.bias_rate_roll / calibration_size
      self.bias_rate_yaw = self.bias_rate_roll / calibration_size
    
    def kalman_1d(self, kalman_state, kalman_uncertainty, kalman_input, kalman_measurement):
      kalman_state = kalman_state+0.004*kalman_input
      kalman_uncertainty = kalman_uncertainty + 0.004*0.004*4*4
      kalman_gain = kalman_uncertainty / (kalman_uncertainty + 3*3)
      kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state)
      kalman_uncertainty = (1-kalman_gain)*kalman_uncertainty
    
    def measure_rate(self):
      gx = round(self.imu.gyro.x) - self.bias_rate_roll
      gy = round(self.imu.gyro.y) - self.bias_rate_pitch
      gz = -round(self.imu.gyro.z) - self.bias_rate_yaw
      
      return gx, gy, gz
    
    def measure_acc(self):
      ax = round(self.imu.accel.x, 2)  #read acceleration along axis
      ay = round(self.imu.accel.y, 2)
      az = round(self.imu.accel.z, 2)
      return ax, ay, az
      
      
    def measure_angle(self):
        ax = round(self.imu.accel.x, 2)  #read acceleration along axis
        ay = round(self.imu.accel.y, 2)
        az = round(self.imu.accel.z, 2)
        gx = round(self.imu.gyro.x) 
        gy = round(self.imu.gyro.y) 
        gz = -round(self.imu.gyro.z) 

        try:
            angle_roll =  math.atan(ay/(math.sqrt(ax**2+az**2)))*(180/math.pi)
            angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))*(180/math.pi)
            
            self.kalman_roll, self.kalman_uncertainty_roll = self.kalman_1d(self.kalman_roll, self.kalman_uncertainty_roll, gx, angle_roll)
            
            self.kalman_pitch, self.kalman_uncertainty_pitch = self.kalman_1d(self.kalman_pitch, self.kalman_uncertainty_pitch, gy, angle_pitch)
        
            self.kalman_roll_temp = self.kalman_roll
            self.kalman_pitch_temp = self.kalman_pitch
            
        except:
            self.kalman_roll = self.kalman_roll_temp
            self.kalman_pitch = self.kalman_pitch_temp
            
        return self.kalman_roll, self.kalman_pitch
        

################################# FUNCTIONS #########################################
################## calculate ESC duty cycle ###################
def calculate_duty_cycle(throttle:float, dead_zone:float = 0.03) -> int:
    """Determines the appropriate PWM duty cycle, in nanoseconds, to use for an ESC controlling a BLDC motor"""

    ### SETTINGS (that aren't parameters) ###
    duty_ceiling:int = pwm_max # the maximum duty cycle (max throttle, 100%) is 2 ms, or 10% duty (0.10)
    duty_floor:int = pwm_min # the minimum duty cycle (min throttle, 0%) is 1 ms, or 5% duty (0.05). HOWEVER, I've observed some "twitching" at exactly 5% duty cycle. It is off, but occasionally clips above, triggering the motor temporarily. To prevent this, i'm bringing the minimum down to slightly below 5%
    ################

    # calcualte the filtered percentage (consider dead zone)
    range:float = 1.0 - dead_zone - dead_zone
    percentage:float = min(max((throttle - dead_zone) / range, 0.0), 1.0)
    
    dutyns:int = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    # clamp within the range
    dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return int(dutyns)

################### normalisation #######################
def normalise(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

################### relay fatal errror #######################
def FATAL_ERROR(msg):
  error_msg = "Fatal error @ " + str(time.ticks_ms()) + " ms: " + msg
  print(error_msg)
  toolkit.log(error_msg)
  try:
    buzz_led.error_msg(msg)
  except:
    pass

################### relay event message ##################### 
def event_msg(msg):
  event_msg = "Event @ "+ str(time.ticks_ms()) + " ms: " + msg
  print(event_msg)
  toolkit.log(event_msg)
  try: buzz_led.event_msg(msg)
  except: pass
  

  
######################### measure RX PWM pulse width ################
def read_pulse_width(pin):    #measures pulse width of PWM signal from CRSF receiver
    while pin.value() == 0:
      pass
    start = time.time_ns()
    while pin.value() == 1:
        pass
    end = time.time_ns()
    duration = end - start
    return int(duration / 1000)
  


#################################### RUN main function ###########################
def run():
  #(say hello via radio)     
  event_msg('fc_start_up')
  #message('Waiting 3 seconds for the IMU to settle...')
  time.sleep(1)
  
  ##overclock
  machine.freq(250000000)
  print("Overclocked to 250,000,000")
  
  event_msg('gyro_calib')
  
  
  
  ################### Gyro calibration #######################
  imu_inst = mpu_measure()
  imu_inst.calibrate_rate(calibration_size)
  event_msg('end_gyro_calib') 
  event_msg('mpu_online')
  gyroscope_online = True

  #################### RX sanity check ##################
  pitch_input = read_pulse_width(pwm_channel_4)
  roll_input = read_pulse_width(pwm_channel_5)
  throttle_input = normalise(read_pulse_width(pwm_channel_2), pwm_min, pwm_max, 0, 1)
  yaw_input =read_pulse_width(pwm_channel_1)
  arm_input = read_pulse_width(pwm_channel_3)
  
  event_msg('rx_online')

  rx_online = True
    
  if gyroscope_online and rx_online:
    
      
    ####################### setup motor pwm #####################
    m1 = PWM(pwm_motor1)
    m2 = PWM(pwm_motor2)
    m3 = PWM(pwm_motor3)
    m4 = PWM(pwm_motor4)
    
    m1.freq(pwm_output_freq)
    m2.freq(pwm_output_freq)
    m3.freq(pwm_output_freq)
    m4.freq(pwm_output_freq)
    
    
    m1.duty_ns(pwm_min*1000)
    m2.duty_ns(pwm_min*1000)
    m3.duty_ns(pwm_min*1000)
    m4.duty_ns(pwm_min*1000)
    
    ######### calculate throttle range ##############
    throttle_range = max_throttle - throttle_idle
    
    
    
    
    ############## instantiate PID control class ##################
    rate_PID_control_roll = PID_controller(rate_roll_kp, rate_roll_ki, rate_roll_kd, rate_i_term_limit)
    rate_PID_control_pitch = PID_controller(rate_pitch_kp, rate_pitch_ki, rate_pitch_kd, rate_i_term_limit)
    rate_PID_control_yaw = PID_controller(rate_yaw_kp, rate_yaw_ki, rate_yaw_kd, rate_i_term_limit)
    
    angle_PID_control_roll = PID_controller(angle_roll_kp, angle_roll_ki, angle_roll_kd, angle_i_term_limit)
    angle_PID_control_pitch = PID_controller(angle_pitch_kp, angle_pitch_ki, angle_pitch_kd, angle_i_term_limit)
    
    armed = False
    angle_mode = False
    
    ######################## BEGIN FC LOOP ################################
    try:
        event_msg('begin_fc_loop')  
        
        
        while True:
            # mark start time
            loop_begin_us:int = time.ticks_us()

            arm_input = normalise(read_pulse_width(pwm_channel_3), pwm_min, pwm_max, -1, 1)
            

            ############################### DISARMED ##########################
            if arm_input>=1:
                ###turn off all motors
                t1 = pwm_min*1000
                t2 = pwm_min*1000
                t3 = pwm_min*1000
                t4 = pwm_min*1000
                
                if armed == True:
                    print('disarmed')
                    armed = False
                
            ############################### ARMED, AIR MODE ###############################  
            elif arm_input < 1 and arm_input >= 0.5:
                ########################## Measure angle rotation RATE #################
                angle_rate_roll, angle_rate_pitch, angle_rate_yaw = imu_inst.measure_rate()
                
                ####################### Read desired rotation RATES ####################
                pitch_input = 0.15*(read_pulse_width(pwm_channel_4) - 1500)
                roll_input = 0.15*(read_pulse_width(pwm_channel_5) - 1500)
                throttle_input = read_pulse_width(pwm_channel_2)
                yaw_input = 0.15*(read_pulse_width(pwm_channel_1) - 1500)
                
                throttle_input = normalise(throttle_input, pwm_min, pwm_max, 0, 1)
                throttle_input = throttle_idle + throttle_input*throttle_range
                
                ########## calculate roll, pitch, yaw rate setpoint ########
                rate_setpoint_roll = roll_input
                rate_setpoint_pitch= pitch_input
                rate_setpoint_yaw = yaw_input
                
                #print(f'{rate_setpoint_roll}, {rate_setpoint_yaw}, {rate_setpoint_pitch}, {throttle_input}')
                
                
                #print(f'{angle_rate_roll}, {angle_rate_pitch}, {angle_rate_yaw}')
                
                roll_resp = rate_PID_control_roll.respond(rate_setpoint_roll, angle_rate_roll)
                pitch_resp = rate_PID_control_pitch.respond(rate_setpoint_pitch, angle_rate_pitch)
                yaw_resp = rate_PID_control_yaw.respond(rate_setpoint_yaw, angle_rate_yaw)
                
                #print(f'{roll_resp}, {pitch_resp}, {yaw_resp}')
                
                ########### calculate throttle value ###############
                t1:float = throttle_input + pitch_resp - roll_resp + yaw_resp
                t2:float = throttle_input- pitch_resp - roll_resp - yaw_resp
                t3:float = throttle_input + pitch_resp + roll_resp - yaw_resp
                t4:float = throttle_input - pitch_resp + roll_resp + yaw_resp
                print(f'{t1}, {t2}, {t3}, {t4}')
                
                
                ############ write throttle value to motors ##############
                
                t1 = min(max(throttle_idle, t1), max_throttle)
                t2 = min(max(throttle_idle, t2), max_throttle)
                t3 = min(max(throttle_idle, t3), max_throttle)
                t4 = min(max(throttle_idle, t4), max_throttle)
                
                t1 = int(round(t1))*1000
                t2 = int(round(t2))*1000
                t3 = int(round(t3))*1000
                t4 = int(round(t4))*1000
                
                #print(f'{t1}, {t2}, {t3}, {t4}')
                
                if armed == False:
                    print('armed')
                    armed = True
                    if angle_mode == True:
                        print('air_mode')
                        angle_mode = False
                    
            ######################### ARMED, ANGLE mode #########################
            elif arm_input <= 0.3:
                ####################### Read desired rotation angle ###################
                pitch_input = 0.1*(read_pulse_width(pwm_channel_4) - 1500)
                roll_input = 0.1*(read_pulse_width(pwm_channel_5) - 1500)
                throttle_input = read_pulse_width(pwm_channel_2)
                yaw_input = 0.1*(read_pulse_width(pwm_channel_1) - 1500)
                
                throttle_input = normalise(throttle_input, pwm_min, pwm_max, 0, 1)
                throttle_input = throttle_idle + throttle_input*throttle_range
                
                ###################### Read ACTUAL rotation angle ###################
                angle_roll, angle_pitch = imu_inst.measure_angle()
                
                
                ################################## PID response ############################
                angle_roll_setpoint = roll_input
                angle_pitch_setpoint = pitch_input
                rate_setpoint_yaw = yaw_input
                
                ########### Compute rotation rate response from angle PID controller ############
                rate_setpoint_roll = angle_PID_control_roll.respond(roll_input, angle_roll)
                rate_setpoint_pitch = angle_PID_control_pitch.respond(pitch_input, angle_pitch)
                
                ########### Compute motor response from output of angle PID controller ##########
                roll_resp = rate_PID_control_roll.respond(rate_setpoint_roll, angle_rate_roll)
                pitch_resp = rate_PID_control_pitch.respond(rate_setpoint_pitch, angle_rate_pitch)
                yaw_resp = rate_PID_control_yaw.respond(rate_setpoint_yaw, angle_rate_yaw)
                
                
                t1 = min(max(throttle_idle, t1), max_throttle)
                t2 = min(max(throttle_idle, t2), max_throttle)
                t3 = min(max(throttle_idle, t3), max_throttle)
                t4 = min(max(throttle_idle, t4), max_throttle)
                
                t1 = int(round(t1))*1000
                t2 = int(round(t2))*1000
                t3 = int(round(t3))*1000
                t4 = int(round(t4))*1000
                
                if armed == False:
                    print('armed')
                    armed = True
                    if angle_mode == False:
                        print('angle_mode')
                        angle_mode = True
                    
                
            m1.duty_ns(t1)
            m2.duty_ns(t2)
            m3.duty_ns(t3)
            m4.duty_ns(t4)
            
            ############################### maintain cycle frequency ###############################
            loop_end_us:int = time.ticks_us()
            elapsed_us:int = loop_end_us - loop_begin_us
            if elapsed_us < cycle_time_us:
                time.sleep_us(cycle_time_us - elapsed_us)
            
    except Exception as e:
      # before we do anything, turn the motors OFF
      m1.duty_ns(pwm_min*1000)
      m2.duty_ns(pwm_min*1000)
      m3.duty_ns(pwm_min*1000)
      m4.duty_ns(pwm_min*1000)

      # deinit
      m1.deinit()
      m2.deinit()
      m3.deinit()
      m4.deinit()
      #exc_type, exc_obj, exc_tb = sys.exc_info()
      #FATAL_ERROR(f'{str(e)}, line: {exc_tb.tb_lineno}')
      FATAL_ERROR(str(e))
    
    ############################ IF FAILED to begin FC LOOP ####################################
  else:
    pass
  

################################# MAIN ###############################
run()










