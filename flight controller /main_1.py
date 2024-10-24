################## main_1 controls roll rate ######################
import time
from machine import Pin, PWM, I2C
from lib.imu import MPU6050
import time
import machine
import toolkit
import sys

#constants
max_roll_rate = 100
max_pitch_rate = 180
max_yaw_rate = 150

target_cycle_freq = 250 #target PID frequency in hz
cycle_time_seconds = 1/target_cycle_freq
cycle_time_us = int(round(cycle_time_seconds*1000000, 0))

##### Gyro Bias ########
gyro_bias_x = 0
gyro_bias_y = 0
gyro_bias_z = 0

###PID rates
roll_kp = 0.00043714285   #0.00125
roll_ki = 0.00			#0.00255
roll_kd = 0.0000		#0.00001571429

pitch_kp = roll_kp
pitch_ki = roll_ki
pitch_kd = roll_kd

yaw_kp = 0.0001
yaw_ki = 0.00
yaw_kd = 0.0
###########

###PID limits
i_term_limit = 100

##pin outs
###mpu6050
mpu_i2c_sda = 0
mpu_i2c_scl = 1
#####

###LED
buzz_pin = 28
buzz_led = toolkit.buzz_led(buzz_pin)
buzz_led.flash_buzz(300, 4)

###esc 
pwm_output_freq = 50
pwm_motor1 = 3 # front left, clockwise
pwm_motor2 = 5 # front right, counter clockwise
pwm_motor3 = 7 # rear left, counter clockwise
pwm_motor4 = 9 # rear right, clockwise
########

####receiver
pwm_pin_1 = 2
pwm_pin_2 = 4
pwm_pin_3 = 6
pwm_pin_4 = 8
pwm_pin_5 = 10
######

pwm_channel_1 = Pin(pwm_pin_1, Pin.IN)
pwm_channel_2 = Pin(pwm_pin_2, Pin.IN)
pwm_channel_3 = Pin(pwm_pin_3, Pin.IN)
pwm_channel_4 = Pin(pwm_pin_4, Pin.IN)
pwm_channel_5 = Pin(pwm_pin_5, Pin.IN)

###pwm min max setting
pwm_min = 1000000
pwm_max = 2000000

###rounding pwm input
round_digit = 3

###throttle settings
throttle_idle = 0.1  #minimum throttle for four mototrs to spin up but not lift up
max_throttle = 1  #maximum throttle limit
throttle_arm_upper_limit = 0.07


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
    
    norm_val = new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))
    
    if norm_val<new_min:
        norm_val = new_min
        
    elif norm_val>new_max:
        norm_val = new_max
    
    return norm_val

##fatal error message
def FATAL_ERROR(msg):
  error_msg = "Fatal error @ " + str(time.ticks_ms()) + " ms: " + msg
  print(error_msg)
  toolkit.log(error_msg)
  try:
    buzz_led.error_msg(msg)
  except:
    pass
  
def event_msg(msg):
  event_msg = "Event @ "+ str(time.ticks_ms()) + " ms: " + msg
  print(event_msg)
  toolkit.log(event_msg)
  try:
    buzz_led.event_msg(msg)
  except:
      pass
  
  #(send error message via radio)
  


'''
def message(msg:str) -> None:
  print(msg)
  toolkit.radio_message(msg)
'''

def read_pulse_width(pin):    #measures pulse width of PWM signal from CRSF receiver
    while pin.value() == 0:
      pass
    start = time.time_ns()
    while pin.value() == 1:
        pass
    end = time.time_ns()
    duration = end - start
    return duration
  

#MAIN
def run():
  #(say hello via radio)
  event_msg('fc_start_up')
  #message('Waiting 3 seconds for the IMU to settle...')
  time.sleep(0.5)
  
  ##overclock
  machine.freq(250000000)
  print("Overclocked to 250,000,000")
  
  gx = 0
  gy = 0
  gz = 0
  
  try:
    ##gyroscope setup
    i2c = I2C(0, sda=machine.Pin(mpu_i2c_sda), scl=machine.Pin(mpu_i2c_scl), freq=200000)   #initiate sda and scl pin for MPU6050
    imu = MPU6050(i2c)  #initiate MPU6050 class
    
    gx_1 = round(imu.gyro.x)
    gy_1 = round(imu.gyro.y)
    gz_1 = round(imu.gyro.z)
    time.sleep_ms(100)
    gyroscope_online = True
    '''
    gx_2 = round(imu.gyro.x)
    gy_2 = round(imu.gyro.y)
    gz_2 = -round(imu.gyro.z)
    if gx_1!=gx_2 or gy_1!=gy_2 or gz_1!=gz_2:
      gyroscope_online = True
      event_msg('mpu_online')
      
    else:
      gyroscope_online = False
      FATAL_ERROR('mpu_error')
    '''
    
  except:
    gyroscope_online = False
    FATAL_ERROR('mpu_error')
  
    
  ##check rx online
  roll_input = normalise(read_pulse_width(pwm_channel_4),pwm_min, pwm_max, -1, 1)
  pitch_input = normalise(read_pulse_width(pwm_channel_5),pwm_min, pwm_max, -1, 1)
  throttle_input = normalise(read_pulse_width(pwm_channel_2),pwm_min, pwm_max, 0, 1)
  yaw_input = normalise(read_pulse_width(pwm_channel_1),pwm_min, pwm_max, -1, 1)
  arm_input = normalise(read_pulse_width(pwm_channel_3),pwm_min, pwm_max, -1, 1)
  
  if pitch_input!=0.00 or roll_input!=0.00 or throttle_input!=0.00  or yaw_input!=0.00:
    rx_online = True
    event_msg('rx_online')
    
  else:
    rx_online = False
    FATAL_ERROR('rx_error')
    
  if gyroscope_online and rx_online:
    '''
    #measure gyrobias
    gxs:list[float] = []
    gys:list[float] = []
    gzs:list[float] = []
    started_at_ticks_ms:int = time.ticks_ms()
    while ((time.ticks_ms() - started_at_ticks_ms) / 1000) < 3.0:
      gx = round(imu.gyro.x)
      gy = round(imu.gyro.y)
      gz = -round(imu.gyro.z)
      
      gxs.append(gx)
      gys.append(gy)
      gzs.append(gz)
      time.sleep(0.025)
      
    gyro_bias_x = sum(gxs) / len(gxs)
    gyro_bias_y = sum(gys) / len(gys)
    gyro_bias_z = sum(gzs) / len(gzs)
    #(communicate gyrobias via radio)
    '''
    
    ####pwm setup
    m1 = PWM(pwm_motor1)
    m2 = PWM(pwm_motor2)
    m3 = PWM(pwm_motor3)
    m4 = PWM(pwm_motor4)
    
    m1.freq(pwm_output_freq)
    m2.freq(pwm_output_freq)
    m3.freq(pwm_output_freq)
    m4.freq(pwm_output_freq)
    
    duty_0_percent:int = calculate_duty_cycle(0.0)
    m1.duty_ns(duty_0_percent)
    m2.duty_ns(duty_0_percent)
    m3.duty_ns(duty_0_percent)
    m4.duty_ns(duty_0_percent)
    
    ###constants calculation
    throttle_range = max_throttle - throttle_idle
    armed = False
    
    '''
    ###last integral and error setup
    roll_last_integral:float = 0.0
    roll_last_error:float = 0.0
    pitch_last_integral:float = 0.0
    pitch_last_error:float = 0.0
    yaw_last_integral:float = 0.0
    yaw_last_error:float = 0.0
    '''
    
    ###instatntiate PID controllers
    roll_PID_control = PID_controller(roll_kp, roll_ki, roll_kd, i_term_limit)
    pitch_PID_control = PID_controller(pitch_kp, pitch_ki, pitch_kd, i_term_limit)
    yaw_PID_control = PID_controller(yaw_kp, yaw_ki, yaw_kd, i_term_limit)
    
  
  
  
    
    #(radio 'beginning flight control loop)
    try:
      event_msg('begin_fc_loop')
      while True:
          
          
            # mark start time
            #loop_begin_us:int = time.ticks_us()
            
            ###gyro data cleaning (maybe it needs to be rates instead?)
        gx = round(imu.gyro.x) - gyro_bias_x
        gy = round(imu.gyro.y) - gyro_bias_y
        gz = round(imu.gyro.z) - gyro_bias_z
        
        print(f'{gx}, {gy}, {gz}\t')
        
        
        ###reading RC data
        pitch_input = normalise(read_pulse_width(pwm_channel_4),pwm_min, pwm_max, -1, 1)
        roll_input = normalise(read_pulse_width(pwm_channel_5),pwm_min, pwm_max, -1, 1)
        throttle_input = normalise(read_pulse_width(pwm_channel_2),pwm_min, pwm_max, 0, 1)
        yaw_input = normalise(read_pulse_width(pwm_channel_1),pwm_min, pwm_max, -1, 1)
        arm_input = normalise(read_pulse_width(pwm_channel_3),pwm_min, pwm_max, -1, 1)
        
        print(pitch_input, roll_input)
        
        '''
        pitch_input = round(pitch_input, round_digit)
        roll_input = round(roll_input, round_digit)
        throttle_input = round(throttle_input, round_digit)
        yaw_input = round(yaw_input, round_digit)
        arm_input = round(arm_input, round_digit)
        '''
        
        if arm_input==1:  ## if quadcopter is DISARMED ##
            #armed = False
            #(raddio quadcopter is disarmed)
            
            ###turn off all motors
            duty_0_percent:int = calculate_duty_cycle(0.0)
            m1.duty_ns(duty_0_percent)
            m2.duty_ns(duty_0_percent)
            m3.duty_ns(duty_0_percent)
            m4.duty_ns(duty_0_percent)
            
            '''
            ###reset PID
            roll_last_integral = 0.0
            roll_last_error = 0.0
            pitch_last_integral = 0.0
            pitch_last_error = 0.0
            yaw_last_integral = 0.0
            yaw_last_error = 0.0
            '''
            
        elif arm_input < 1: ## if quadcopter is ARMED ##
            #armed = True
            #event_msg('armed')
            #(radio quadcopter is armed)
            
            #if flight_mode_input == 0:
            #manual = True
            #event_msg('man_mode')
            #(radio manual mode)
            #if throttle_input > throttle_idle:
            adj_throttle:float = throttle_idle + (throttle_range * throttle_input)
            
            roll_setpoint = roll_input*max_roll_rate 
            pitch_setpoint= pitch_input*max_pitch_rate 
            yaw_setpoint = yaw_input*max_yaw_rate 
            
            roll_current_position = gx
            pitch_current_position = gy
            yaw_current_position = gz
            
            #print(f'{roll_current_position}, {pitch_current_position}, {yaw_current_position}')
            
            roll_resp = roll_PID_control.respond(roll_setpoint, roll_current_position)
            pitch_resp = pitch_PID_control.respond(pitch_setpoint, pitch_current_position)
            yaw_resp = yaw_PID_control.respond(yaw_setpoint, yaw_current_position)
            
            #print(f'########{roll_resp}, {pitch_resp}, {yaw_resp}\n')
            #time.sleep(0.2)
            
            ###calculate throttle values
            '''
            t1:float = adj_throttle + pitch_resp + roll_resp - yaw_resp + throttle_arm_upper_limit
            t2:float = adj_throttle + pitch_resp - roll_resp + yaw_resp + throttle_arm_upper_limit
            t3:float = adj_throttle - pitch_resp + roll_resp + yaw_resp + throttle_arm_upper_limit
            t4:float = adj_throttle - pitch_resp - roll_resp - yaw_resp + throttle_arm_upper_limit
            '''
            
            '''
            t1:float = adj_throttle + pitch_resp + roll_resp - yaw_resp 
            t2:float = adj_throttle + pitch_resp - roll_resp + yaw_resp 
            t3:float = adj_throttle - pitch_resp + roll_resp + yaw_resp 
            t4:float = adj_throttle - pitch_resp - roll_resp - yaw_resp
            
            
            mmix [motor number] [throttle] [roll] [pitch] [yaw]
            
            mmix 1 1.000 -1.000 1.000 -1.000 (Rear Right motor)
            mmix 2 1.000 -1.000 -1.000  1.000 (Front Right motor)
            mmix 3 1.000 1.000 1.000 1.000 (Rear Left motor)
            mmix 4 1.000 1.000 -1.000 -1.000 (Front Left motor)
            
            '''
            
            t1:float = adj_throttle - roll_resp + pitch_resp - yaw_resp
            t2:float = adj_throttle - roll_resp - pitch_resp + yaw_resp
            t3:float = adj_throttle + roll_resp + pitch_resp + yaw_resp
            t4:float = adj_throttle + roll_resp - pitch_resp - yaw_resp
            ###write PWM duty cycle
            
            m1.duty_ns(calculate_duty_cycle(t1))
            m2.duty_ns(calculate_duty_cycle(t2))
            m3.duty_ns(calculate_duty_cycle(t3))
            m4.duty_ns(calculate_duty_cycle(t4))
            
            #log_packet = f'{gx}, {gy}, {gz}########{roll_resp}, {pitch_resp}, {yaw_resp}########{t1}, {t2}, {t3}, {t4}\n'
            
            #toolkit.log(log_packet)
                #elif flight_mode_input == 1:
                    #manual = False
                    #(radio auto mode)
                    #event_msg('auto_mode')
            '''        
            else:
                m1.duty_ns(calculate_duty_cycle(throttle_arm_upper_limit))
                m2.duty_ns(calculate_duty_cycle(throttle_arm_upper_limit))
                m3.duty_ns(calculate_duty_cycle(throttle_arm_upper_limit))
                m4.duty_ns(calculate_duty_cycle(throttle_arm_upper_limit))
            '''
        
      
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
      #exc_type, exc_obj, exc_tb = sys.exc_info()
      #FATAL_ERROR(f'{str(e)}, line: {exc_tb.tb_lineno}')
      FATAL_ERROR(str(e))
  
  else:
    pass
  
    
  

run()




