import logging

from driver import Driver
from keeplane import SwitchLane, KeepLane
from lowlevel import LowLevelDriverBase

class in_turning: pass:
    
class switch_lane: pass

class slow_down: pass
    
class keep_lane: pass

class emergency: pass

class bypass:
    class moving_out: pass
    class waiting_for_bypass: pass:
    class moving_in: pass
    bypass_cur_state=moving_out


class back_on_track: pass


logging.basicConfig(level=10)


class BypasserDummy(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')

class BackOnTrackDummy(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')


class KeepLaneDummy(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')

class TurnerDummy(KeepLaneDummy):
    def drive(self, state, control):
        self._logger.debug('here')

class SwitchLaneDummy(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')


class OurDriver(Driver):
    def __init__(self, *args, **kwargs):

        Driver.__init__(self, *args, **kwargs)
        self._logger = logging.getLogger().getChild(self.__class__.__name__)
        self._curstate = keep_lane

    def drive(self, msg):
        self.state.setFromMsg(msg)
        self.drive_from_state(self.state, self.control)
        return self.control.toMsg()

    def drive_from_state(self, sensor_state, control):
        def get_driver(curstate):
            return {emergency: EmegencyDummy,
                    keep_lane: KeepLaneDummy,
                    bypass: Bypasser,
                    in_turning: TurnerDummy,
                    slowdown:SlowDownDummy
                    back_on_track: BackOnTrack}[curstate]()

        next_state = self._determine_state(self._curstate, sensor_state)
        if next_state != self._curstate: # replace the driver            
            self.lowlevel_driver = get_driver(self._curstate)
        self._curstate = next_state
        
        self.lowlevel_driver.drive(sensor_state, control)

        self.gear()
        if not lowlevel_driver.speed_override:
            self.speed()

    def ifCarAhead(self):
        pass
    def ifIsGoingToCrash(self):
        pass
    def ifReadyToBypass(self):
        pass
            
    def _determine_state(self,cur_state, sensor_state):
        next_state = cur_state
        if self.is_stuck():
            next_state = back_on_track
        elif self.ifIsGoingToCrash():
            next_state = emergency
        elif cur_state in [keep_lane,slow_down ]:
            if self.ifCarAhead():
                if self.ifReadyToBypass():
                    next_state = bypass
                else:
                    next_state = slow_down
            elif self.checkIfIsInTurn():
                next_state = in_turning
            else:
                next_state = keep_lane # not going to crash
        elif cur_state in [in_turning ]:
            if self.ifCarAhead():
                next_state = slow_down
            elif self.checkIfIsInTurn():
                next_state = in_turning
            else:
                next_state = keep_lane 
        elif cur_state in [emergency ]:
            if self.ifCarAhead():
                next_state = slow_down
            else:
                next_state = keep_lane
        elif cur_state in [bypass]:
            next_state = bypass
            if cur_state.bypass_state == bypass.moving_out:
                if self.ifInBypassingLane():
                    next_state.bypass_state = bypass.waiting_for_bypass
                else:
                    next_state.bypass_state = bypass.moving_out
            elif cur_state.bypass_state == bypass.moving_in:
                if self.ifInProperLane():
                    next_state = keep_lane
                else:
                    next_state.bypass_state == bypass.moving_in
            elif cur_state.bypass_state == bypass.waiting_for_bypass:
                if self.readyToFinishBypassing():
                    next_state.bypass_state == bypass.moving_in
                else:
                    next_state.bypass_state == bypass.waiting_for_bypass
        
        
        return next_state
