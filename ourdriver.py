import logging

from driver import Driver
from keeplane import SwitchLane, KeepLane
from lowlevel import LowLevelDriverBase


class switch_lane: pass


class keep_lane: pass


class bypass: pass


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
        self.bypass = Bypasser()
        self.back_on_track = BackOnTrack()
        self._curstate = 'keep_lane'

    def drive(self, msg):
        self.state.setFromMsg(msg)
        self.drive_from_state(self.state, self.control)
        return self.control.toMsg()

    def drive_from_state(self, sensor_state, control):
        def get_driver(curstate):
            return {switch_lane: self.switcher,
                    keep_lane: self.keeper,
                    bypass: self.bypass,
                    back_on_track: self.back_on_track}[curstate]

        self._curstate = self._determine_state(sensor_state)

        lowlevel_driver = get_driver(self._curstate)
        lowlevel_driver.drive(sensor_state, control)

        self.gear()
        if not lowlevel_driver.speed_override:
            self.speed()

    def _determine_state(self, sensor_state):
        curstate = keep_lane
        if self.is_stuck():
            curstate = back_on_track
        return curstate
