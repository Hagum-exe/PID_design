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
max_roll_angle = 60
max_pitch_angle = max_roll_angle
max_yaw_angle = max_roll_angle

angle_yaw = 0
#yaw_current_position = 0

############ gyro constants ##########
calibration_size = 100

angle_roll_temp = 0
angle_pitch_temp = 0
angle_yaw_temp = 0

##### kalman presets ####
kalman_pitch = 0
kalman_uncertainty_pitch = 2*2

kalman_roll = 0
kalman_uncertainty_roll = 2*2

kalman_yaw = 0
kalman_uncertainty_yaw= 2*2


################ PID constants ###################
roll_kp = 0.00043714285
roll_ki = 0.00255
roll_kd = 0.00002571429

pitch_kp = roll_kp
pitch_ki = roll_ki
pitch_kd = roll_kd

yaw_kp = 0.001714287
yaw_ki = 0.003428571
yaw_kd = 0.0


i_term_limit = 150

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
pwm_min = 1000000
pwm_max = 2000000


############## throttle settings ###################
throttle_idle = 0.14  #minimum throttle for four mototrs to spin up but not lift up
max_throttle = 0.22  #maximum throttle limit
throttle_arm_upper_limit = 0.05


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
    I = self.I + self.ki * err * cycle_time_seconds
    I = max(min(I, self.ki_limit), -self.ki_limit) # constrain within I-term limits, prevents integral windup
    D = self.kd * (err - self.E)/cycle_time_seconds
    self.E = err
    self.I = I
    return P + I + D


################################# FUNCTIONS #########################################

################### kalman filter ##################
def kalman_1d(kalman_state, kalman_uncertainty, kalman_input, kalman_measurement):
    kalman_state = kalman_state+0.004*kalman_input
    kalman_uncertainty = kalman_uncertainty + 0.004*0.004*4*4
    kalman_gain = kalman_uncertainty / (kalman_uncertainty + 3*3)
    kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state)
    kalman_uncertainty = (1-kalman_gain)*kalman_uncertainty
    
    return kalman_state, kalman_uncertainty   

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
    return duration
  


#################################### RUN main function ###########################
def run():
  #(say hello via radio)
  event_msg('fc_start_up')
  #message('Waiting 3 seconds for the IMU to settle...')
  time.sleep(3)
  
  ##overclock
  machine.freq(250000000)
  print("Overclocked to 250,000,000")
  
  event_msg('start_cablibrate')
  time.sleep(2)
  
  i2c = I2C(0, sda=machine.Pin(mpu_i2c_sda), scl=machine.Pin(mpu_i2c_scl), freq=200000)   #initiate sda and scl pin for MPU6050
  imu = MPU6050(i2c)  #initiate MPU6050 class
  
  bias_roll = 0
  bias_pitch = 0
  bias_yaw = 0
  
  
  
  for i in range(0, calibration_size):
    ax = round(imu.accel.x, 2)  #read acceleration along axis
    ay = round(imu.accel.y, 2)
    az = round(imu.accel.z, 2)
    gz = -round(imu.gyro.z)
    
    
    angle_roll = math.atan(ay/(math.sqrt(ax**2+az**2)))*(180/math.pi)
    angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))*(180/math.pi)
    angle_yaw = gz*0.001
    
    bias_roll += angle_roll
    bias_pitch += angle_pitch
    bias_yaw += angle_yaw

    
    time.sleep_ms(1)
    
  bias_roll = bias_roll / calibration_size
  bias_pitch = bias_pitch / calibration_size
  bias_yaw = bias_yaw / calibration_size
  
  event_msg('end_gyro_calib') 
  event_msg('mpu_online')
  gyroscope_online = True
      
  
  '''
  try:
    
    #################### gyro calibration ##################
    event_msg('start_cablibrate')
    time.sleep(2)
    
    i2c = I2C(0, sda=machine.Pin(mpu_i2c_sda), scl=machine.Pin(mpu_i2c_scl), freq=200000)   #initiate sda and scl pin for MPU6050
    imu = MPU6050(i2c)  #initiate MPU6050 class
    
    bias_roll = 0
    bias_pitch = 0
    bias_yaw = 0
    
    
    
    for i in range(0, calibration_size):
      ax = round(imu.accel.x, 2)  #read acceleration along axis
      ay = round(imu.accel.y, 2)
      az = round(imu.accel.z, 2)
      gz = -round(imu.gyro.z)
      
      
      angle_roll = math.atan(ay/(math.sqrt(ax**2+az**2)))*(180/math.pi)
      angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))*(180/math.pi)
      angle_yaw = gz*0.001
      
      bias_roll += angle_roll
      bias_pitch += angle_pitch
      bias_yaw += angle_yaw

      
      time.sleep_ms(1)
      
    bias_roll = bias_roll / calibration_size
    bias_pitch = bias_pitch / calibration_size
    bias_yaw = bias_yaw / calibration_size
    
    event_msg('end_gyro_calib') 
    event_msg('mpu_online')
    gyrpscope_online = True
      
  
    
  
  except:
    gyroscope_online = False
    FATAL_ERROR('mpu_error')
  '''
  
    
  #################### RX sanity check ##################
  pitch_input = normalise(read_pulse_width(pwm_channel_4), pwm_min, pwm_max, -1, 1)
  roll_input = normalise(read_pulse_width(pwm_channel_5), pwm_min, pwm_max, -1, 1)
  throttle_input = normalise(read_pulse_width(pwm_channel_2), pwm_min, pwm_max, 0, 1)
  yaw_input = normalise(read_pulse_width(pwm_channel_1), pwm_min, pwm_max, -1, 1)
  arm_input = normalise(read_pulse_width(pwm_channel_3), pwm_min, pwm_max, -1, 1)
  
  if pitch_input!=0.00 or roll_input!=0.00 or throttle_input!=0.00  or yaw_input!=0.00:
    rx_online = True
    event_msg('rx_online')
    
  else:
    rx_online = False
    FATAL_ERROR('rx_error')
    
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
    
    ######### calculate throttle range ##############
    throttle_range = max_throttle - throttle_idle
    
    
    
    
    ############## instantiate PID control class ##################
    roll_PID_control = PID_controller(roll_kp, roll_ki, roll_kd, i_term_limit)
    pitch_PID_control = PID_controller(pitch_kp, pitch_ki, pitch_kd, i_term_limit)
    yaw_PID_control = PID_controller(yaw_kp, yaw_ki, yaw_kd, i_term_limit)
    
  
  
    ######################## BEGIN FC LOOP ################################
    try:
      event_msg('begin_fc_loop')
      while True:
        # mark start time
        loop_begin_us:int = time.ticks_us()
        
        ################ measure gyro data ################
        ax = round(imu.accel.x, 2)  #read acceleration along axis
        ay = round(imu.accel.y, 2)
        az = round(imu.accel.z, 2)
        gx = round(imu.gyro.x)  #read roll rate along axis
        gy = round(imu.gyro.y)
        gz = -round(imu.gyro.z)
    
       
        ############### apply kalman filter and measure roll, pitch, yaw angle ###############
        try:
          angle_roll = math.atan(ay/(math.sqrt(ax**2+az**2)))*(180/math.pi) - bias_roll
          angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))*(180/math.pi) - bias_pitch
          angle_yaw += angle_yaw*cycle_time_seconds - bias_yaw

          angle_roll_temp = angle_roll
          angle_pitch_temp = angle_pitch
          angle_yaw_temp = angle_yaw

          kalman_roll, kalman_uncertainty_roll = kalman_1d(kalman_roll, kalman_uncertainty_roll, gx, angle_roll)
          
          kalman_pitch, kalman_uncertainty_pitch = kalman_1d(kalman_pitch, kalman_uncertainty_pitch, gy, angle_pitch)
          
          kalman_yaw, kalman_uncertainty_yaw = kalman_1d(kalman_yaw, kalman_uncertainty_yaw, gz, angle_yaw)
        
        except:
          angle_roll = angle_roll_temp
          angle_pitch = angle_pitch_temp
          angle_yaw = angle_yaw_temp
          
        ################# read RX data ###################
        pitch_input = normalise(read_pulse_width(pwm_channel_4), pwm_min, pwm_max, -1, 1)
        roll_input = normalise(read_pulse_width(pwm_channel_5), pwm_min, pwm_max, -1, 1)
        throttle_input = normalise(read_pulse_width(pwm_channel_2), pwm_min, pwm_max, 0, 1)
        yaw_input = normalise(read_pulse_width(pwm_channel_1), pwm_min, pwm_max, -1, 1)
        arm_input = normalise(read_pulse_width(pwm_channel_3), pwm_min, pwm_max, -1, 1)
        
        ############################### DISARMED ##########################
        if arm_input==1:
          ###turn off all motors
          duty_0_percent:int = calculate_duty_cycle(0.0)
          m1.duty_ns(duty_0_percent)
          m2.duty_ns(duty_0_percent)
          m3.duty_ns(duty_0_percent)
          m4.duty_ns(duty_0_percent)
          
        ############################### ARMED ###############################  
        elif arm_input < 1:
        
          adj_throttle:float = throttle_idle + (throttle_range * throttle_input)
          
          ########## calculate roll, pitch, yaw setpoint ########
          roll_setpoint = roll_input*max_roll_angle
          pitch_setpoint= pitch_input*max_pitch_angle 
          yaw_setpoint = yaw_input*max_yaw_angle
          
          ########## collect roll, pitch, yaw current position #########
          roll_current_position = normalise(kalman_roll, -90, 90, -1, 1) 
          pitch_current_position = normalise(kalman_pitch, -90, 90, -1, 1) 
          yaw_current_position  = normalise(kalman_yaw, -90, 90, -1, 1) 
          
          print(f'{roll_current_position}, {pitch_current_position}, {yaw_current_position}')
          
          roll_resp = roll_PID_control.respond(roll_setpoint, roll_current_position)
          pitch_resp = pitch_PID_control.respond(pitch_setpoint, pitch_current_position)
          yaw_resp = yaw_PID_control.respond(yaw_setpoint, yaw_current_position)
          
          #print(f'{roll_resp}, {pitch_resp}, {yaw_resp}')
          
          ########### calculate throttle value ###############
          t1:float = adj_throttle + pitch_resp + roll_resp - yaw_resp
          t2:float = adj_throttle + pitch_resp - roll_resp + yaw_resp
          t3:float = adj_throttle - pitch_resp + roll_resp + yaw_resp
          t4:float = adj_throttle - pitch_resp - roll_resp - yaw_resp
          
          ############ write throttle value to motors ##############
          m1.duty_ns(calculate_duty_cycle(t1))
          m2.duty_ns(calculate_duty_cycle(t2))
          m3.duty_ns(calculate_duty_cycle(t3))
          m4.duty_ns(calculate_duty_cycle(t4))
            
        ############################### maintain cycle frequency ################
        loop_end_us:int = time.ticks_us()
        elapsed_us:int = loop_end_us - loop_begin_us
        if elapsed_us < cycle_time_us:
            time.sleep_us(cycle_time_us - elapsed_us)
            
      
      
    
    ############################ IF FAILED to begin FC LOOP ####################################
  
    except Exception as e: # something went wrong. Flash the LED so the pilot sees it

      # before we do anything, turn the motors OFF
      duty_0_percent:int = calculate_duty_cycle(0.0)
      m1.duty_ns(duty_0_percent)
      m2.duty_ns(duty_0_percent)
      m3.duty_ns(duty_0_percent)
      m4.duty_ns(duty_0_percent)

      # deinit
      m1.deinit()
      m2.deinit()
      m3.deinit()
      m4.deinit()
      
      FATAL_ERROR(str(e))
  
  else:
    pass
  

################################# MAIN ###############################
run()

