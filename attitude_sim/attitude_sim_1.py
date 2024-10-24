import matplotlib.pyplot as plt
import numpy as np

#simulation setttings
t_step = 0.01
t = np.arange(0.0, 20.0, t_step)    #from 0 to 20 seconds with increments of each step as 0.01

air_density = 1.15



#power = np.arange(power_min, power_max, (power_max - power_min)/power_resolution)   #makes an evenly spaced array between min and max power

#classes
##Drone class
class Drone:
  def __init__(self, g, mass, drag_coef, cross_section, pwr_min, pwr_max, pwr_resolution, controller):  ##physical features of drone captured in an object of this class
    self.g = g
    self.mass = mass
    self.drag_coef = drag_coef
    self.cross_section = cross_section
    self.power = np.arange(pwr_min, pwr_max, (pwr_max - pwr_min)/pwr_resolution)
    self.controller = controller 
    
    self.input = [0]
    self.accel = [0]
    self.speed = [0]
    self.pos = [0]
    self.index = 0

  def Drag(self, v):    ##calculates drag force with 0.5*drag_coef*cross_section*air_decnsity*v**2
    return 0.5 * self.drag_coef * self.cross_section * air_density * v * abs(v) 

  def Acceleration_f(self, input, m, v):
    f = self.power[input] - self.Drag(v)
    return f/m + self.g

  def Velocity_t(self, a, vOld):
    return vOld + a * t_step

  def Position_t(self, v, xOld):
    return xOld + v * t_step

  def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

  def Simulate(self, input):
    self.index += 1
    accel = self.Acceleration_f(input, self.mass, self.speed[self.index - 1])
    vel = self.Velocity_t(accel, self.speed[self.index - 1])
    pos = self.Position_t(vel, self.pos[self.index - 1])
    self.input.append(input)
    self.accel.append(accel)
    self.speed.append(vel)
    self.pos.append(pos)

  def Control(self, sp):
    pv = self.pos[self.index]
    resp, err, i = self.controller.Respond(pv, sp)
    input = Drone.clamp(int(resp), 0, (len(self.power) - 1))
    self.Simulate(input)

################################################################

##controller class
class PID_Controller():
  def __init__(self, kp, ki, kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.E = 0
    self.I = 0
    self.sp = 0

  def Respond(self, pv, sp):
    if (sp != self.sp):
      self.I = 0
      self.sp = sp
    err = sp - pv
    P = self.kp * err
    I = self.I + self.ki * err * t_step
    D = self.kd * (err - self.E)/t_step
    self.E = err
    self.I = I
    return P + I + D, err, 0




#MAIN

#initialise drones
drone1 = Drone(-9.81, 0.25, 0.9, 0.03, 0.1, 10, 1000, PID_Controller(600, 0, 400))
drone2 = Drone(-9.81, 0.25, 0.9, 0.03, 0.1, 10, 1000, PID_Controller(600, 3, 400))
drone3 = Drone(-9.81, 0.25, 0.9, 0.03, 0.1, 10, 1000, PID_Controller(600, 10, 400))
drone4 = Drone(-9.81, 0.25, 0.9, 0.03, 0.1, 10, 1000, PID_Controller(600, 20, 400))

##runs signal
signal = np.heaviside(t-2.0, 1.0) * 50

for index, sp in enumerate(signal):
  if (index > 0):
    drone1.Control(sp)
    drone2.Control(sp)
    drone3.Control(sp)
    drone4.Control(sp)


##plotting
fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(32, 8), sharey='row')
ax1.plot(t, drone1.pos, label = 'Kp = 600, Ki = 0, Kd = 400')
ax1.plot(t, signal, label = 'Signal')
ax2.plot(t, drone2.pos, label = 'Kp = 600, Ki = 3, Kd = 400')
ax2.plot(t, signal, label = 'Signal')
ax3.plot(t, drone3.pos, label = 'Kp = 600, Ki = 10, Kd = 400')
ax3.plot(t, signal, label = 'Signal')
ax4.plot(t, drone4.pos, label = 'Kp = 600, Ki = 100, Kd = 400')
ax4.plot(t, signal, label = 'Signal')

ax1.set(xlabel='Time (s)', ylabel='Height (m)',
       title='Position')
ax2.set(xlabel='Time (s)', ylabel='Height (m)',
       title='Position')
ax3.set(xlabel='Time (s)', ylabel='Height (m)',
       title='Position')
ax4.set(xlabel='Time (s)', ylabel='Height (m)',
       title='Position')
ax1.set_ylim([-10, 60])


ax1.legend(loc='best')
ax2.legend(loc='best')
ax3.legend(loc='best')
ax4.legend(loc='best')
plt.show()