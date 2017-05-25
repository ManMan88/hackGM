'''
Created on Apr 5, 2012

@author: lanquarden
Edited by General Motors
'''
import msgParser


class CarControl(object):
    '''
    An object holding all the control parameters of the car
    '''
    # TODO range check on set parameters

    def __init__(self, accel = 0.0, brake = 0.0, gear = 1, steer = 0.0, clutch = 0.0, focus = 0, meta = 0):
        '''Constructor'''
        self.parser = msgParser.MsgParser()
        
        self.actions = None
        
        self._accel = accel
        self._brake = brake
        self._gear = gear
        self._steer = steer
        self._clutch = clutch
        self._focus = focus
        self._meta = meta
    
    def toMsg(self):
        self.actions = {}
        
        self.actions['accel'] = [self._accel]
        self.actions['brake'] = [self._brake]
        self.actions['gear'] = [self._gear]
        self.actions['steer'] = [self._steer]
        self.actions['clutch'] = [self._clutch]
        self.actions['focus'] = [self._focus]
        self.actions['meta'] = [self._meta]
        
        return self.parser.stringify(self.actions)

    @property
    def accel(self):
        return self._accel

    @accel.setter
    def accel(self, value):
        self._accel = value

    @property
    def brake(self):
        return self._brake

    @brake.setter
    def brake(self, value):
        self._brake = value

    @property
    def gear(self):
        return self._gear

    @gear.setter
    def gear(self, value):
        self._gear = value

    @property
    def steer(self):
        return self._steer

    @steer.setter
    def steer(self, value):
        self._steer = value

    @property
    def clutch(self):
        return self._clutch

    @clutch.setter
    def clutch(self, value):
        self._clutch = value

    @property
    def meta(self):
        return self._meta

    @meta.setter
    def meta(self, value):
        self._meta = value


        
        