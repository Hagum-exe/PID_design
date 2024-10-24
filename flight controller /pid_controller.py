from main import cycle_time_seconds
################constants#######################


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