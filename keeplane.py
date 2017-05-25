from lowleveldriver import LowLevelDriver

from scipy.interpolate import splrep, splev
import numpy as np

class KeepLane(LowLevelDriver):
    #def __init__(self, lanes_str, lim_left, lim_right):
    def __init__(self, lanes_str, lane):
        """
        Arguments:
        lane_left, lane_right - in trackPos units, e.g. on a 3-lane road 
            keeping right is between 1/3 and 1
        """
        LowLevelDriver.__init__(self, lanes_str)
        
        self.calculateLanesData()
        left, right = self.findLaneBorders(lane)
        self._center = 0.5*(left + right)
        self._width = 5.
        
        self._steer_max = 0.366519
        
    def drive(self, state, control):
        angle = state.angle
        dist = state.trackPos - self._center
        
        control.steer = (angle - dist*0.5)/self._steer_max

class SwitchLane(LowLevelDriver):
    def __init__(self, new_left, new_right, start_pos, start_angle, start_time, duration):
        self._new_cent = 0.5*(new_left + new_right)
        self._start_t = start_time
        
        t0, x0 = start_time, start_pos
        t1, x1 = start_time + duration, self._new_cent
        dx0, dx1 = start_angle, 0
        tmargin = (t1 - t0) /1.5
        pt = [t0 - tmargin, t0, t1, t1 + tmargin]
        px = [x0 - tmargin * dx0, x0, x1, x1 + tmargin * dx1]
        self._spline_params = splrep(pt, px)
    
    def get_steer(self, time):
        xeval = splev(time, self._spline_params, der=1)
        return np.arctan(xeval)
    
    def drive(self, state, control):
        steer = state.angle - self.get_steer(state.curLapTime)
        control.steer= steer

class SwitchVelocity(LowLevelDriver):
    def __init__(self,start_velocity,new_velocity,start_time,duration):
        t0, v0 = start_time, start_velocity
        t1, v1 = start_time + duration, new_velocity
        dv0, dv1 = 0, 0
        tmargin = (t1 - t0) /1.5
        pt = [t0 - tmargin, t0, t1, t1 + tmargin]
        pv = [v0 - tmargin * dv0, v0, v1, v1 + tmargin * dv1]
        self._spline_params = splrep(pt, px)
    
    def get_accel(self, time):
        aeval = splev(time, self._spline_params, der=1)
        return aeval
    
    def drive(self, state, control):
        accel = self.get_accel(state.curLapTime)
        control.accel = accel
    
class KeepVelocity(LowLevelDriver):
    def __init__(self):
        self.velocityPID = PID()
    
    def setVelocity(self,velocity):
        self.velocityPID.setPoint(velocity)
    
    def drive(self,state,control):
        accel = self.velocityPID.update(state.speedX)
        print accel
        if accel >= 0:
            control.accel = accel
        else:
            control.brake = -accel
    
if __name__ == "__main__":
    sl = SwitchLane(-1., -0.3333, 0.5, 0.0, 0, 5)
    state = object()
    steers = np.empty(10)
    
    for t in xrange(10):
        steers[t] = sl.get_steer(t)
    
    print steers
    
class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=.03, I=0.001, D=0.01, Derivator=0, Integrator=0, 
        Integrator_max=500, Integrator_min=-500,PID_max=1,PID_min=-1):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.PID_range = PID_min, PID_max

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value
        if PID < self.PID_range[0]:
            PID = -1
        elif PID > self.PID_range[1]:
            PID = 1

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator
