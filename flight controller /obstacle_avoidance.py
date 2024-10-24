from machine import Pin
import utime
from toolkit import hcsr04
from pid_controller import PID_controller
from traversal import traversal_speed

class obstacle_avoidance():
    def __init__(self, evasion_speed, l_trigger, l_echo, m_trigger, m_echo, r_trigger, r_echo, max_range, min_range, traversal_speed):
        self.l_sensor = hcsr04(l_trigger, l_echo)
        self.m_sensor = hcsr04(m_trigger, m_echo)
        self.r_sensor = hcsr04(r_trigger, r_echo)
        self.max_range = max_range
        self.min_range = min_range
        self.evasion_speed = evasion_speed
        self.traversal_speed = traversal_speed
    
       
    def obstacle_avoidance_response(self, org_speed_vector):
        l_sensor_distance = self.l_sensor.measure()
        m_sensor_distance = self.m_sensor.measure()
        r_sensor_distance = self.r_sensor.measure()
        
        l_proximity =  l_sensor_distance<=self.min_range
        m_proximity = m_sensor_distance<=self.min_range
        r_proximity = r_sensor_distance<=self.min_range
        traversal_speed_vector = org_speed_vector
        if l_proximity and not m_proximity and not r_proximity:
            target_speed_vector = [0, self.evasion_speed]
            
        elif r_proximity and not m_proximity and not r_proximity:
            target_speed_vector = [0, -self.evasion_speed]
        
        elif r_proximity and l_proximity and not m_proximity:
            target_speed_vector = [org_speed_vector[0], 0]

        elif r_proximity and l_proximity and m_proximity:
            target_speed_vector = [0, 0]
            
        self.traversal_speed.traversal_response(target_speed_vector)
       
    
    
     
        
        


