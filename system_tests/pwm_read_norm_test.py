import machine
from machine import Pin
import time


pwm_pin_1 = 2
pwm_pin_2 = 4
pwm_pin_3 = 6
pwm_pin_4 = 8
pwm_pin_5 = 10
#pwm_pin_6 = 14

pwm_channel_1 = Pin(pwm_pin_1, Pin.IN)
pwm_channel_2 = Pin(pwm_pin_2, Pin.IN)
pwm_channel_3 = Pin(pwm_pin_3, Pin.IN)
pwm_channel_4 = Pin(pwm_pin_4, Pin.IN)
pwm_channel_5 = Pin(pwm_pin_5, Pin.IN)
#pwm_channel_6 = Pin(pwm_pin_6, Pin.IN)


pwm_min = 1000000
pwm_max = 2000000

def normalise(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    
    norm_val = new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))
    
    if norm_val<new_min:
        norm_val = new_min
        
    elif norm_val>new_max:
        norm_val = new_max
    
    return norm_val


def read_pulse_width(pin):    #measures pulse width of PWM signal from CRSF receiver
    while pin.value() == 0:
      pass
    start = time.time_ns()
    while pin.value() == 1:
        pass
    end = time.time_ns()
    duration = end - start
    if duration<pwm_min:
        duration = pwm_min
        
    elif duration>pwm_max:
        duration = pwm_max
    return duration

def measure_bias(pwm_channel, new_min, new_max):
    count = 5
    bias_sum = 0
    for i in range(count):
        channel_val = normalise(read_pulse_width(pwm_channel), pwm_min, pwm_max, new_min, new_max)
        bias_sum+=channel_val
        time.sleep_ms(5)
        
    return bias_sum/count

'''
ch1_bias = measure_bias(pwm_channel_1, -1.00, 1.00)
ch2_bias = measure_bias(pwm_channel_2, 0.00, 1.00)
ch3_bias = measure_bias(pwm_channel_3, -1, 1)
ch4_bias = measure_bias(pwm_channel_4, -1, 1)
ch5_bias = measure_bias(pwm_channel_5, -1.00, 1.00)
#ch6_bias = measure_bias(pwm_channel_6, -1.00, 1.00)
'''
#print(ch1_bias)

ch1_val = 0
ch2_val = 0
ch3_val = 0
ch4_val = 0
ch5_val = 0
ch6_val = 0
while True:
    
    
    
    ch1_val = normalise(read_pulse_width(pwm_channel_1), pwm_min, pwm_max, -1.00, 1.00)
    ch2_val = normalise(read_pulse_width(pwm_channel_2), pwm_min, pwm_max, 0.00, 1.00)
    ch3_val = normalise(read_pulse_width(pwm_channel_3), pwm_min, pwm_max, -1.00, 1.00)
    ch4_val = normalise(read_pulse_width(pwm_channel_4), pwm_min, pwm_max, -1.00, 1.00)
    ch5_val = normalise(read_pulse_width(pwm_channel_5), pwm_min, pwm_max, -1.00, 1.00)
    
    
    '''
    ch1_val = (ch1_val-ch1_bias)+0
    ch2_val = (ch2_val-ch2_bias)+0
    ch3_val = (ch3_val-ch3_bias)+0
    ch4_val = (ch4_val-ch4_bias)+0
    ch5_val = (ch5_val-ch5_bias)+0
    '''
    
    '''
    ch1_val = read_pulse_width(pwm_channel_1)
    ch2_val = read_pulse_width(pwm_channel_2)
    ch3_val = read_pulse_width(pwm_channel_3)
    ch4_val = read_pulse_width(pwm_channel_4)
    ch5_val = read_pulse_width(pwm_channel_5)
    #ch6_val = read_pulse_width(pwm_channel_6)
    '''
    
    print(f'ch1: {ch1_val}\tch2: {ch2_val}\tch3: {ch3_val}\tch4: {ch4_val}\tch5: {ch5_val}\n')
    time.sleep_ms(50)
