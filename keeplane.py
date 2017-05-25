from lowleveldriver import LowLevelDriver
from scipy.interpolate import splrep, splev
import numpy as np

from lowlevel import LowLevelDriverBase


class KeepLane(LowLevelDriverBase):
    def __init__(self, lim_left, lim_right):
        """
        Arguments:
        lane_left, lane_right - in trackPos units, e.g. on a 3-lane road 
            keeping right is between 1/3 and 1
        """
        super(KeepLane, self).__init__()
        self._left = lim_left
        self._right = lim_right
        self._center = 0.5 * (self._left + self._right)
        self._width = 5.

        self._steer_max = 0.366519

    def drive(self, state, control):
        angle = state.angle
        dist = state.trackPos - self._center

        control.steer = (angle - dist * 0.5) / self._steer_max


class SwitchLane(LowLevelDriverBase):
    def __init__(self, new_left, new_right, start_pos, start_angle, start_time, duration):
        super(SwitchLane, self).__init__()
        self._new_cent = 0.5 * (new_left + new_right)
        self._start_t = start_time

        t0, x0 = start_time, start_pos
        t1, x1 = start_time + duration, self._new_cent
        dx0, dx1 = start_angle, 0
        tmargin = (t1 - t0) / 1.5
        pt = [t0 - tmargin, t0, t1, t1 + tmargin]
        px = [x0 - tmargin * dx0, x0, x1, x1 + tmargin * dx1]
        self._spline_params = splrep(pt, px)

    def get_steer(self, time):
        xeval = splev(time, self._spline_params, der=1)
        return np.arctan(xeval)

    def drive(self, state, control):
        steer = state.angle - self.get_steer(state.curLapTime)
        control.steer = steer


if __name__ == "__main__":
    sl = SwitchLane(-1., -0.3333, 0.5, 0.0, 0, 5)
    state = object()
    steers = np.empty(10)

    for t in xrange(10):
        steers[t] = sl.get_steer(t)

    print steers
