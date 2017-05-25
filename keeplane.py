from lowleveldriver import LowLevelDriver


class KeepLane(LowLevelDriver):
    def __init__(self, lim_left, lim_right):
        """
        Arguments:
        lane_left, lane_right - in trackPos units, e.g. on a 3-lane road 
            keeping right is between 1/3 and 1
        """
        self._left = lim_left
        self._right = lim_right
        self._center = 0.5*(self._left + self._right)
        self._width = 5.
        
        self._steer_max = 0.366519
        
    def drive(self, state, control):
        angle = state.angle
        dist = state.trackPos - self._center
        
        control.steer = (angle - dist*0.5)/self._steer_max
