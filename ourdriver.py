import logging
import numpy

from driver import Driver, STUCK_TIME
from keeplane import KeepLane, SwitchVelocity, KeepVelocity
from lowlevel import LowLevelDriverBase
from lowleveldriver import LowLevelDriver


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
        # self._logger.debug('here')
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


class Turner(KeepLane):
    def __init__(self, parent, accel_max=10.):
        """
        Arguments:
        lane_left, lane_right - in trackPos units, e.g. on a 3-lane road
            keeping right is between 1/3 and 1
        """
        LowLevelDriver.__init__(self, parent)
        curvature = self.findCurve(parent.sensors)

        self.calculateLanesData(parent.sensors)
        left, right = self.findLaneBorders(self.lane)
        self._logger.debug('left: %s, right: %s', left, right)
        self._center = 0.5 * (left + right)
        self._steer_max = 0.366519
        
        self._logger.debug('IN CURVE!!!!')
        overide_velocity = curvature * parent.steer_lock * 1
        self._logger.debug('setting override velocity: %s', overide_velocity)
        
        self.accelerateTime = abs(parent.sensors.speedX - overide_velocity) / accel_max
        self.startTime = parent.sensors.curLapTime
        self.velSwitcher = SwitchVelocity(
            parent.sensors.speedX, overide_velocity, self.startTime, self.accelerateTime)
        self.velocityKeeper = KeepVelocity()
        self.parent = parent
        
    def drive(self, sensors, control):
        self._logger.debug('here')
        angle = sensors.angle
        dist = sensors.trackPos - self._center
        self._logger.debug('sensors.trackPos = %s; center = %s; dist = %s', sensors.trackPos, self._center, dist)

        control.steer = (angle - dist * 0.5) / self._steer_max
        
        if sensors.curLapTime - self.startTime < self.accelerateTime:
            self._logger.debug('decelerating (time)!')
            target_vel = self.velSwitcher.get_target_velocity(sensors.curLapTime)
            self.velocityKeeper.setVelocity(target_vel)
        else:
            curvature = self.findCurve(sensors)
            overide_velocity = curvature * self.parent.steer_lock * 1
            self.velocityKeeper.setVelocity(overide_velocity)
        self.velocityKeeper.drive(sensors, control)
        

class SwitchLaneDummy(LowLevelDriverBase):
    def drive(self, sensors, control):
        self._logger.debug('here')


class OurDriver(Driver):
    def __init__(self, *args, **kwargs):
        self.angles = numpy.zeros(19)
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15

        for i in range(5, 9):
            self.angles[i] = -20 + (i - 5) * 5
            self.angles[18 - i] = 20 - (i - 5) * 5

        logging.basicConfig(level=10)
        Driver.__init__(self, *args, **kwargs)
        self.max_speed = kwargs.get('max_speed', 100)
        self._logger = logging.getLogger().getChild(self.__class__.__name__)
        self._curstate = keep_lane
        self.lowlevel_driver = None

        self.util_driver = LowLevelDriver(self)


    def get_driver(self, state):
        cls = {emergency: EmergencyDummy,
               keep_lane: KeepLane,
               bypass: BypasserDummy,
               in_turning: Turner,
               slow_down: SlowDownDummy,
               back_on_track: BackOnTrackDummy}[state]
        self._logger.debug(cls)
        return cls(self)

    def drive(self, msg):
        self.state.setFromMsg(msg)
        try:
            self.drive_from_state(self.state, self.control)
        except Exception as err:
            self._logger.debug('sensor values caused exception: %s', err)
        return self.control.toMsg()

    def drive_from_state(self, sensors, control):
        self.sensors = sensors
        # self._logger.debug('cur_state %s', self._curstate)
        next_state = self._determine_state(self._curstate, sensors)
        # self._logger.debug('next_state %s', next_state)
        if self.lowlevel_driver is None:
            self.lowlevel_driver = self.get_driver(next_state)

        if next_state != self._curstate:  # replace the driver
            self._logger.info('state changed: %s', next_state)
            self.lowlevel_driver = self.get_driver(next_state)
            self._logger.info('lowlevel_driver %s', self.lowlevel_driver)
            self._curstate = next_state

        self.lowlevel_driver.drive(sensors, control)

        self.gear()
        #if not self.lowlevel_driver.speed_override:
        #    self.speed()

    def _determine_state(self, cur_state, sensors):
        next_state = cur_state
        if self.ifIsStuck(sensors):
            next_state = back_on_track
        elif self.ifIsGoingToCrash():
            next_state = emergency
        elif cur_state in [keep_lane, slow_down]:
            if self.ifCarAhead():
                if self.ifReadyToBypass():
                    next_state = bypass
                else:
                    next_state = slow_down
            elif self.checkIfIsInTurn(sensors):
                next_state = in_turning
            else:
                next_state = keep_lane  # not going to crash
        elif cur_state in [in_turning]:
            if self.ifCarAhead():
                next_state = slow_down
            elif self.checkIfIsInTurn(sensors):
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

    def checkIfIsInTurn(self, sensors):
        curvature = self.util_driver.findCurve(sensors)
        return self.util_driver.isCurve(curvature, self.max_speed, self.steer_lock)

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
