import logging

from driver import Driver
from keeplane import SwitchLane, KeepLane
from lowlevel import LowLevelDriverBase


class switch_lane: pass

class slow_down: pass:
    
class keep_lane: pass

class emergency: pass

class bypass:
    class moving_out: pass
    class waiting_for_bypass: pass:
    class moving_in: pass
    bypass_cur_state=moving_out


class back_on_track: pass


logging.basicConfig(level=10)


class Bypasser(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')


class BackOnTrack(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')


class KeepLaneDummy(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')


class SwitchLaneDummy(LowLevelDriverBase):
    def drive(self, state, control):
        self._logger.debug('here')


class OurDriver(Driver):
    def __init__(self, *args, **kwargs):

        Driver.__init__(self, *args, **kwargs)
        self._logger = logging.getLogger().getChild(self.__class__.__name__)
        self.keeper = KeepLaneDummy()  # KeepLane(-0.333, 1. / 3.)
        self.switcher = SwitchLaneDummy() #SwitchLane(-1., -0.3333, 0.5, 0.0, 0, 5)
        self.slowdown  = SlowDownDummy()
        self.emergency  = EmegencyDummy()
        self.bypass = Bypasser()
        self.back_on_track = BackOnTrack()
        self._curstate = keep_lane

    def drive(self, msg):
        self.state.setFromMsg(msg)
        self.drive_from_state(self.state, self.control)
        return self.control.toMsg()

    def drive_from_state(self, sensor_state, control):
        def get_driver(curstate):
            return {emergency: self.emergency,
                    keep_lane: self.keeper,
                    bypass: self.bypass,
                    back_on_track: self.back_on_track}[curstate]

        self._curstate = self._determine_state(self._curstate, sensor_state)

        lowlevel_driver = get_driver(self._curstate)
        lowlevel_driver.drive(sensor_state, control)

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
        if self.is_stuck():
            next_state = back_on_track
        elif self.ifIsGoingToCrash():
            next_state = emergency
        elif cur_state in [keep_lane,slow_dow ]:
            if self.ifCarAhead():
                if self.ifReadyToBypass():
                    next_state = bypass
                else:
                    next_state = slow_down
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
