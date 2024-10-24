import machine
from machine import Pin, I2C
import array, time
import rp2
import utime


#########buzzer and leds################################
class buzzer():
    
    def __init__(self, buzz_pin):
        self.buzzpin = machine.Pin(buzz_pin, machine.Pin.OUT)
    
    
    def buzz(self, interval_ms):
        self.buzzpin.high()
        time.sleep_ms(interval_ms)
        self.buzzpin.low()
        time.sleep_ms(interval_ms)
################################################################

NUM_LEDS = 4
PIN_NUM = 22
brightness = 0.8

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


# Create the StateMachine with the ws2812 program, outputting on pin
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=machine.Pin(PIN_NUM))

# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)

# Display a pattern on the LEDs via an array of LED RGB values.
ar = array.array("I", [0 for _ in range(NUM_LEDS)])

def pixels_show():
    dimmer_ar = array.array("I", [0 for _ in range(NUM_LEDS)])
    for i,c in enumerate(ar):
        r = int(((c >> 8) & 0xFF) * brightness)
        g = int(((c >> 16) & 0xFF) * brightness)
        b = int((c & 0xFF) * brightness)
        dimmer_ar[i] = (g<<16) + (r<<8) + b
    sm.put(dimmer_ar, 8)
    time.sleep_ms(10)
####################################

class led_strip():
    black = (0, 0, 0)
    red = (255, 0, 0)
    yellow = (255, 150, 0)
    green = (0, 255, 0)
    cyan = (0, 255, 255)
    blue = (0, 0, 255)
    purple = (180, 0, 255)
    white = (255, 255, 255)
    colors_list = [black, red, yellow, green, cyan, blue, purple, white]
    
    def __init__(self):
        pass
    
    def pixels_set(self, i, color):
        ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]
        
    def pixels_fill(self, color_index):
        color = led_strip.colors_list[color_index]
        for i in range(len(ar)):
            self.pixels_set(i, color)
            
    def flash(self, colour_index, interval_ms):
        self.pixels_fill(colour_index)
        pixels_show()
        time.sleep_ms(interval_ms)
        self.pixels_fill(0)
        pixels_show()
        time.sleep_ms(interval_ms)
    
        
################################################################

class buzz_led(buzzer, led_strip):
    dot_ms = 50
    dash_ms = 300
    error_dict = {'rx_error': '0000', 'mpu_error': '0001', 'pmw_error': '0010', 'hc_error': '0011'}
    event_dict = {'begin_fc_loop': '0100', 'fc_start_up': '0101', 'man_mode': '0110', 'auto_mode': '0111', 'armed': '0111', 'gyro_calib': '1001', 'end_gyro_calib': '1010'}
    
    def __init__(self, buzz_pin):
        buzzer.__init__(self, buzz_pin)
        led_strip.__init__(self)
        
    
    def flash_buzz(self, interval_ms, color_index):
        self.buzz(interval_ms)
        self.flash(color_index, interval_ms)
        
    def error_msg(self, msg):
        error_code = buzz_led.error_dict[msg]
        for digit in error_code:
            if digit=='0':
                self.flash_buzz(buzz_led.dot_ms, 1)
            if digit=='1':
                self.flash_buzz(buzz_led.dash_ms, 1)
                
    def event_msg(self, msg):
        event_code = buzz_led.event_dict[msg]
        for digit in event_code:
            if digit=='0':
                self.flash_buzz(buzz_led.dot_ms, 3)
            if digit=='1':
                self.flash_buzz(buzz_led.dash_ms, 3)

###############buzzer and leds end##########################

###############hcsr04 sensor suite##########################s
class hcsr04():
    
    def __init__(self, trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        
    def measure(self):
        self.trigger.low()
        utime.sleep_us(2)
        self.trigger.high()
        utime.sleep_us(5)
        self.trigger.low()
        signalon = 0
        signaloff = 0
        while self.echo.value() == 0:
            signaloff = utime.ticks_us()
        while self.echo.value() == 1:
            signalon = utime.ticks_us()
        timepassed = signalon - signaloff
        distance = (timepassed * 0.0343) / 2
        return distance


#################### measure ground speed PMW3901 sensor suite ###################
class pmw3901():
    def __init__(self, nano_sda, nano_scl, addr, msg_size):
        self.pmw3901_I2C = I2C(0, scl=nano_scl, sda=nano_sda, freq=100000)
        self.pmw3901_I2C.writeto(addr, 'Hi from Pi')
        self.msg_size = msg_size
        self.addr = addr
        
    def measure(self):
        self.pmw3901_I2C.readfrom(self.addr, self.msg_size)
        actual_speed_vector = []
        return actual_speed_vector
########################################################################




###############logging error and event messages#################
def log(msg):
    f = open('logs.txt', 'a')
    f.write(msg + '\n\n')
    f.close()

      

##test main
#buzz_led_1 = buzz_led(28)
#buzzer1 = buzzer(28)


#while True:
#    buzz_led_1.error_msg('hc_error')
    #buzzer1.buzz(300)