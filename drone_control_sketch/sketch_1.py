import numpy as np
import time

from machine import Pin, PWM
from lib.imu import MPU6050
from time import sleep
from machine import Pin, I2C

from toolkit import *

#constants
max_roll_rate = 0
max_pitch_rate = 0
max_yaw_rate = 0

target_cycle_freq = 250 #target PID frequency in hz
cycle_time_seconds = 1/target_cycle_freq
cycle_time_us = int(round(cycle_time_seconds*1000000, 0))

###PID rates
roll_kp = 1
roll_ki = 1
roll_kd = 1

pitch_kp = 1
pitch_ki = 1
pitch_kd = 1

yaw_kp = 1
yaw_ki = 1
yaw_kd = 1
###########

###PID limits
i_term_limit = 150

##pin outs
###mpu6050
gpio_i2c_sda = 12
gpio_i2c_scl = 13
#####

###LED
led_pin = 25

###esc 
pwm_output_freq = 50
pwm_motor1 = 2 # front left, clockwise
pwm_motor2 = 28 # front right, counter clockwise
pwm_motor3 = 15 # rear left, counter clockwise
pwm_motor4 = 16 # rear right, clockwise
########

####receiver
pwm_channel_1 = 0
pwm_channel_2 = 1
pwm_channel_3 = 2
pwm_channel_4 = 3
pwm_channel_5 = 4
pwm_channel_6 = 5
######

###pwm min max setting
pwm_min = 1000000
pwm_max = 2000000

###throttle settings
throttle_idle = 0.14  #minimum throttle for four mototrs to spin up but not lift up
max_throttle = 0.22  #maximum throttle limit
throttle_arm_upper_limit = 0.05


############################

#classes
##uniersal controller class
class PID_controller:
  def __init__(self, kp, ki, kd, ki_limit):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.ki_limit = ki_limit
    
    self.E = 0
    self.I = 0
    self.sp = 0

  def respond(self, sp, pv):    #pv as current position being passed in
    if (sp != self.sp): #renew integral sum if new setpoint is given
      self.I = 0
      self.sp = sp
    err = sp - pv
    P = self.kp * err
    I = self.I + self.ki * err * cycle_time
    I = max(min(I, self.ki_limit), -self.ki_limit) # constrain within I-term limits, prevents integral windup
    D = self.kd * (err - self.E)/cycle_time
    self.E = err
    self.I = I
    return P + I + D, err
    #return P + I + D, err, 0
################################################################

      
##motor PWM signal output normalisation class
'''
class normalise():
    def __init__(self, pwm_min, pwm_max, pwm_resolution):
        self.power = np.arange(pwm_min, pwm_max, (pwm_max - pwm_min)/pwm_resolution)
        #print(self.power)
        
    def clamp (self, resp, min_resp, max_resp):
        return max(min(max_resp, resp), min_resp)
    
    def pwm_normalise(self, resp):
        return self.clamp(int(resp), 0, len(self.power)-1)
'''

##yaw pitch roll rotation control class
class rotation_control(PID_controller):
    def __init__(self,kp, ki, kd):
        super().__init__(kp, ki, kd)
        
        
    def rotation_respond(self, current_angle, set_point_angle, normalise_setting):
        resp = self.respond(current_angle, set_point_angle)
        return resp
    
    

#functions
##calculate duty cycle
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

##normalise
def normalise(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

##fatal error message
def FATAL_ERROR(msg:str) -> None:
  em:str = "Fatal error @ " + str(time.ticks_ms()) + " ms: " + msg
  print(em)
  toolkit.log(em) #fix this line
  
  #(send error message via radio)
  
  led = machine.Pin(led_pin, machine.Pin.OUT)
  while True:
          led.off()
          time.sleep(1.0)
          led.on()
          time.sleep(1.0)

         
def message(msg:str) -> None:
  print(msg)
  toolkit.radio_message(msg)


#MAIN
def run():
  #(say hello via radio)
  #(flash a LED)
  message('Waiting 3 seconds for the IMU to settle...')
  time.sleep(3)
  
  ##overclock
  machine.freq(250000000)
  print("Overclocked to 250,000,000")
  
  ##led setup
  led = machine.pin
  
  ##gyroscope setup
  i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=200000)   #initiate sda and scl pin for MPU6050
  imu = MPU6050(i2c)  #initiate MPU6050 class
  gx = round(imu.gyro.x)
  gy = round(imu.gyro.y)
  gz = round(imu.gyro.z)
  if gx!=0 and gy!=0 and gz!=0:
    gyroscope_online = True
    
  if gyroscope_online:
    #measure gyrobias
    gxs:list[float] = []
    gys:list[float] = []
    gzs:list[float] = []
    started_at_ticks_ms:int = time.ticks_ms()
    while ((time.ticks_ms() - started_at_ticks_ms) / 1000) < 3.0:
      gx = round(imu.gyro.x)
      gy = round(imu.gyro.y)
      gz = round(imu.gyro.z)
      
      gxs.append(gx)
      gys.append(gy)
      gzs.append(gz)
      time.sleep(0.025)
      
    gyro_bias_x = sum(gxs) / len(gxs)
    gyro_bias_y = sum(gys) / len(gys)
    gyro_bias_z = sum(gzs) / len(gzs)
    #(communicate gyrobias via radio)
      
    ####pwm setup
    m1 = PWM(pwm_motor1)
    m2 = PWM(pwm_motor2)
    m3 = PWM(pwm_motor3)
    m4 = PWM(pwm_motor4)
    
    m1.freq(pwm_output_freq)
    m2.freq(pwm_output_freq)
    m3.freq(pwm_output_freq)
    m4.freq(pwm_output_freq)
    
    ###constants calculation
    throttle_range = max_throttle - throttle_idle
    armed = False
    
    ###last integral and error setup
    roll_last_integral:float = 0.0
    roll_last_error:float = 0.0
    pitch_last_integral:float = 0.0
    pitch_last_error:float = 0.0
    yaw_last_integral:float = 0.0
    yaw_last_error:float = 0.0
    
    ###instatntiate PID controllers
    roll_PID_control = PID_controller(roll_kp, roll_ki, roll_kd)
    pitch_PID_control = PID_controller(pitch_kp, pitch_ki, pitch_kd)
    yaw_PID_control = PID_controller(yaw_kp, yaw_ki, yaw_kd)
    
  
  
  
    #(fire up a LED some where)
    #(radio 'beginning flight control loop)
    try:
      while True:
        # mark start time
        loop_begin_us:int = time.ticks_us()
        
        ###gyro data cleaning (maybe it needs to be rates instead?)
        gx = round(imu.gyro.x) - gyro_bias_x
        gy = round(imu.gyro.y) - gyro_bias_y
        gz = round(imu.gyro.z) - gyro_bias_z
        
        
        ###reading RC data
        pitch_input = normalise(pwm_min, read_pulse_width(pwm_channel_3), 0, 1)
        roll_input = normalise(pwm_min, read_pulse_width(pwm_channel_2), 0, 1)
        throttle_input = normalise(pwm_min, read_pulse_width(pwm_channel_1), 0, 1)
        yaw_input = normalise(pwm_min, read_pulse_width(pwm_channel_4), 0, 1)
        arm_input = normalise(pwm_min, read_pulse_width(pwm_channel_5), 0, 1)
        flight_mode_input = normalise(pwm_min, read_pulse_width(pwm_channel_6), 0, 1)
        
        if arm_input == 0:  ##if quadcopter is disarmed
          armed = False
          #(raddio quadcopter is disarmed)
          
          ###turn off all motors
          duty_0_percent:int = calculate_duty_cycle(0.0)
          m1.duty_ns(duty_0_percent)
          m2.duty_ns(duty_0_percent)
          m3.duty_ns(duty_0_percent)
          m4.duty_ns(duty_0_percent)
          
          ###reset PID
          roll_last_integral = 0.0
          roll_last_error = 0.0
          pitch_last_integral = 0.0
          pitch_last_error = 0.0
          yaw_last_integral = 0.0
          yaw_last_error = 0.0
          
        elif arm_input == 1:
          armed = True
          #(radio quadcopter is armed)
          
          if flight_mode_input == 0:
            manual = True
            #(radio manual mode)
            if throttle_input > throttle_arm_upper_limit:
              adj_throttle:float = throttle_idle + (throttle_range * throttle_input)
              
              roll_setpoint = roll_input*max_roll_rate 
              pitch_setpoint= pitch_input*max_pitch_rate 
              yaw_setpoint = yaw_input*max_yaw_rate 
              
              roll_current_position = gx
              pitch_current_position = gy
              yaw_current_position = gz
              
              roll_resp = roll_PID_control.respond(roll_setpoint, roll_current_position)
              pitch_resp = pitch_PID_control.respond(pitch_setpoint, pitch_current_position)
              yaw_resp = yaw_PID_control.respond(yaw_setpoint, yaw_current_position)
              
              ###calculate throttle values
              t1:float = adj_throttle + pitch_resp + roll_resp - yaw_resp
              t2:float = adj_throttle + pitch_resp - roll_resp + yaw_resp
              t3:float = adj_throttle - pitch_resp + roll_resp + yaw_resp
              t4:float = adj_throttle - pitch_resp - roll_resp - yaw_resp
              
              ###write PWM duty cycle
              m1.duty_ns(calculate_duty_cycle(t1))
              m2.duty_ns(calculate_duty_cycle(t2))
              m3.duty_ns(calculate_duty_cycle(t3))
              m4.duty_ns(calculate_duty_cycle(t4))
              
              
              
              
              
              
        
            elif flight_mode_input == 1:
              manual = False
              #(radio auto mode)
        

            loop_end_us:int = time.ticks_us()
            elapsed_us:int = loop_end_us - loop_begin_us
            if elapsed_us < cycle_time_us:
                time.sleep_us(cycle_time_us - elapsed_us)
            
      
      
      
  
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
    FATAL_ERROR("MPU6050 not online!")
  
    
  








'''
# calculate throttle values
                t1:float = adj_throttle + pitch_resp + roll_resp - yaw_resp
                t2:float = adj_throttle + pitch_resp - roll_resp + yaw_resp
                t3:float = adj_throttle - pitch_resp + roll_resp + yaw_resp
                t4:float = adj_throttle - pitch_resp - roll_resp - yaw_resp

                # Adjust throttle according to input
                M1.duty_ns(calculate_duty_cycle(t1))
                M2.duty_ns(calculate_duty_cycle(t2))
                M3.duty_ns(calculate_duty_cycle(t3))
                M4.duty_ns(calculate_duty_cycle(t4))
'''