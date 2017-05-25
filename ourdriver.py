import logging

from driver import Driver, STUCK_TIME
from keeplane import KeepLane
from lowlevel import LowLevelDriverBase


class in_turning: pass


class switch_lane: pass


class slow_down: pass


class keep_lane: pass


class emergency: pass


class bypass:
    class moving_out: pass

    class waiting_for_bypass: pass

    class moving_in: pass

    bypass_cur_state = moving_out


class back_on_track: pass


class EmergencyDummy(LowLevelDriverBase):
    def drive(self, sensors, control):
        self._logger.debug('here')


class BypasserDummy(LowLevelDriverBase):
    def drive(self, sensors, control):
        self._logger.debug('here')


class SlowDownDummy(LowLevelDriverBase):
    def drive(self, sensors, control):
        self._logger.debug('here')


class BackOnTrackDummy(LowLevelDriverBase):

    def drive(self, sensors, control):
        self._logger.debug('here')
        if sensors.angle * sensors.trackPos > 0.0:
            control.gear = 1
            control.steer = - sensors.angle / 4
        # Back of car is pointing into direction of street
        else:
            # to bring car parallel to track axis
            control.steer = sensors.angle / 4  # steerLock;
            control.gear = -1  # gear R

        if self.parent.bringingCartBack < 5:

            control.accel = 0
            control.brake = 1
            control.gear = 0
            control.steer = 0

        else:

            # Build a CarControl variable and return it

            control.accel = 0.3
            control.brake = 0
        control.clutch = 0
        control.focus = 0
        control.meta = 0


class KeepLaneDummy(KeepLane):
    def __init__(self, parent):
        super(KeepLaneDummy, self).__init__(parent, -1, 1)

    def drive(self, sensors, control):
        self._logger.debug('here')
        super(KeepLaneDummy, self).drive(sensors, control)


class TurnerDummy(KeepLaneDummy):
    def drive(self, sensors, control):
        self._logger.debug('here')


class SwitchLaneDummy(LowLevelDriverBase):
    def drive(self, sensors, control):
        self._logger.debug('here')


class OurDriver(Driver):
    def __init__(self, *args, **kwargs):

        logging.basicConfig(level=10)
        Driver.__init__(self, *args, **kwargs)
        self.max_speed = 200
        self._logger = logging.getLogger().getChild(self.__class__.__name__)
        self._curstate = keep_lane
        self.lowlevel_driver = self.get_driver(self._curstate)

    def get_driver(self, state):
        cls = {emergency: EmergencyDummy,
               keep_lane: KeepLaneDummy,
               bypass: BypasserDummy,
               in_turning: TurnerDummy,
               slow_down: SlowDownDummy,
               back_on_track: BackOnTrackDummy}[state]
        self._logger.debug(cls)
        return cls(self)

    def drive(self, msg):
        self.state.setFromMsg(msg)
        self.drive_from_state(self.state, self.control)
        return self.control.toMsg()

    def drive_from_state(self, sensors, control):
        self._logger.debug('cur_state %s', self._curstate)
        next_state = self._determine_state(self._curstate, sensors)
        self._logger.debug('next_state %s', next_state)

        if next_state != self._curstate:  # replace the driver
            self._logger.debug('state changed: %s', next_state)
            self.lowlevel_driver = self.get_driver(next_state)
            self._logger.debug('lowlevel_driver %s', self.lowlevel_driver)
            self._curstate = next_state

        self.lowlevel_driver.drive(sensors, control)

        self.gear()
        if not self.lowlevel_driver.speed_override:
            self.speed()

    def _determine_state(self, cur_state, sensor_state):
        next_state = cur_state
        if self.ifIsStuck(sensor_state):
            next_state = back_on_track
        elif self.ifIsGoingToCrash():
            next_state = emergency
        elif cur_state in [keep_lane, slow_down]:
            if self.ifCarAhead():
                if self.ifReadyToBypass():
                    next_state = bypass
                else:
                    next_state = slow_down
            elif self.checkIfIsInTurn():
                next_state = in_turning
            else:
                next_state = keep_lane  # not going to crash
        elif cur_state in [in_turning]:
            if self.ifCarAhead():
                next_state = slow_down
            elif self.checkIfIsInTurn():
                next_state = in_turning
            else:
                next_state = keep_lane
        elif cur_state in [emergency]:
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
            elif cur_state.bypass_state == bypass.waiting_for_bypass:
                if self.readyToFinishBypassing():
                    next_state.bypass_state = bypass.moving_in

        return next_state

    def ifInBypassingLane(self):
        pass

    def ifInProperLane(self):
        pass

    def readyToFinishBypassing(self):
        pass

    def ifCarAhead(self):
        pass

    def checkIfIsInTurn(self):
        pass

    def ifIsGoingToCrash(self):
        pass

    def ifReadyToBypass(self):
        pass

    def ifIsStuck(self, sensors):
        if sensors.trackPos > 1.0 or sensors.trackPos < -1.0:
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
