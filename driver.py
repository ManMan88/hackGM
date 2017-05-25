'''
Created on Apr 4, 2012

@author: lanquarden
Edited by General Motors
'''
import random

from enum import Enum

import carControl
import carState
import msgParser

# from keeplane import KeepLane, SwitchLane, KeepVelocity, SwitchVelocity

STUCK_TIME = 25


class Direction(Enum):
    FORWARD = 1
    OPPOSITE = -1


class Driver(object):
    '''
    A driver object for the SCRC
    '''

    def __init__(self, stage, lanes_str, seed):
        '''Constructor'''
        self.WARM_UP = 0
        self.QUALIFYING = 1
        self.RACE = 2
        self.UNKNOWN = 3
        self.stage = stage

        self.parser = msgParser.MsgParser()

        self.state = carState.CarState()

        self.control = carControl.CarControl()

        self.steer_lock = 0.366519
        self.max_speed = 250
        self.prev_rpm = None

        self.stuckCounter = 0
        self.bringingCartBack = 0

        self.lanes = [Direction[x] for x in lanes_str.split(',')]
        random.seed(a=seed)  # Set a random seed
        
    def init(self):
        '''Return init string with rangefinder angles'''
        self.angles = [0 for x in range(19)]
        
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15

        for i in range(5, 9):
            self.angles[i] = -20 + (i-5) * 5
            self.angles[18 - i] = 20 - (i-5) * 5
        
        return self.parser.stringify({'init': self.angles})

    def drive(self, msg):
        self.state.setFromMsg(msg)

        if self.is_stuck():
            self.bringCarBackOnTrack()
        else:
            self.steer()
            self.gear()
            self.speed()

        return self.control.toMsg()

    def is_stuck(self):
        if self.state.trackPos > 1.0 or self.state.trackPos < -1.0:
            # update stuckCounter counter
            self.stuckCounter += 1
            self.bringingCartBack = 150
        else:
            if self.bringingCartBack != 0:
                self.bringingCartBack = self.bringingCartBack - 1
            else:
                # if not stuckCounter reset stuckCounter counter
                self.stuckCounter = 0
        return self.stuckCounter > STUCK_TIME

    def bringCarBackOnTrack(self):
        # Set gear and steering command assuming car is
        # pointing in a direction out of track

        # If car is pointing in direction of the street
        if self.state.angle * self.state.trackPos > 0.0:
            self.control.gear = 1
            self.control.steer = - self.state.angle / 4
        # Back of car is pointing into direction of street
        else:
            # to bring car parallel to track axis
            self.control.steer = self.state.angle / 4  # steerLock;
            self.control.gear = -1  # gear R

        if self.bringingCartBack < 5:

            self.control.accel = 0
            self.control.brake = 1
            self.control.gear = 0
            self.control.steer = 0

        else:

            # Build a CarControl variable and return it

            self.control.accel = 0.3
            self.control.brake = 0
        self.control.clutch = 0
        self.control.focus = 0
        self.control.meta = 0
        return

    def steer(self):
        angle = self.state.angle
        dist = self.state.trackPos
        
        self.control.steer = (angle - dist*0.5)/self.steer_lock
    
    def gear(self):
        rpm = self.state.rpm
        gear = self.state.gear
        
        if self.prev_rpm is None:
            up = True
        else:
            if (self.prev_rpm - rpm) < 0:
                up = True
            else:
                up = False
        
        if up and rpm > 7000:
            gear += 1
        
        if not up and rpm < 3000:
            gear -= 1

        self.control.gear = gear

    def speed(self):
        speed = self.state.speedX
        accel = self.control.accel

        if speed < self.max_speed:
            accel += 0.1
            if accel > 1:
                accel = 1.0
        else:
            accel -= 0.1
            if accel < 0:
                accel = 0.0

        self.control.accel = accel
        self.control.brake = 0.0

    def onShutDown(self):
        pass
    
    def onRestart(self):
        pass

class TestDriver(Driver):
    def __init__(self, stage, lanes_str, seed):
        Driver.__init__(self, stage, lanes_str, seed)
        self.laneKeeper = KeepLane(lanes_str, 0)
        self.velocityKeeper = KeepVelocity()
        self.velSwitcher = SwitchVelocity(0, self.max_speed, 5, 10)
        self.break_event = SwitchVelocity(self.max_speed, 0, 20, 3)
        self.switcher = SwitchLane(-1., -0.3333, 0.5, 0.0, 0, 5)
        self.velocityKeeper.setVelocity(0)
        
    def steer(self):
        self.laneKeeper.drive(self.state, self.control)
        #curvature = self.laneKeeper.findCurve(self.state)
        #print self.isCurve(curvature)
    
    def speed(self):
        if 5 < self.state.curLapTime < 15:
            target_vel = self.velSwitcher.get_target_velocity(self.state.curLapTime)
            self.velocityKeeper.setVelocity(target_vel)
#        elif self.state.curLapTime >= 20:
#            target_vel = self.break_event.get_target_velocity(self.state.curLapTime)
#            self.velocityKeeper.setVelocity(target_vel)
            

        curvature = self.laneKeeper.findCurve(self.state)
        if self.isCurve(curvature):
            overide_velocity = curvature*self.steer_lock
            print overide_velocity
            self.velocityKeeper.setVelocity(overide_velocity)
            
        self.velocityKeeper.drive(self.state, self.control)
    
    def isCurve(self, curvature):
        """
        True if angular velocity > steer max.
        """
        
        return self.max_speed/curvature > self.steer_lock
    
