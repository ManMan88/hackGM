import numpy as np

from enum import Enum

from lowlevel import LowLevelDriverBase


class Direction(Enum):
    FORWARD = 1
    OPPOSITE = -1


class LowLevelDriver(LowLevelDriverBase):
    def __init__(self, parent):
        super(LowLevelDriver, self).__init__(parent)
        self.angles = parent.angles
        self.lanes = parent.lanes
        self.lane = -1

    def calculateLanesData(self, sensors):
        self.numOfLanes = len(self.lanes)
        laneWidth = 2 / self.numOfLanes
        self.lanesPositions = [(-1 + i * laneWidth) for i in range(0, self.numOfLanes)]
        if sensors.trackPos > 1  or sensors.trackPos < -1:
            raise ValueError()
        else:
            self.lane = np.flatnonzero(np.array(self.lanesPositions) < sensors.trackPos)[0]

    def findLaneBorders(self, lane):
        leftSide = self.lanesPositions[lane]
        if (lane == self.numOfLanes - 1):
            rightSide = 1
        else:
            rightSide = self.lanesPositions[lane + 1]
        return leftSide, rightSide

    def calculateTrackEdges(self, state):
        """
        Convert sensor data to cartesian lane edges.
        """
        track_y = np.array(state.track) * np.sin(np.deg2rad(self.angles))
        track_x = np.array(state.track) * np.cos(np.deg2rad(self.angles))
        track_edges = np.zeros((19, 2))
        counter = 0
        for y, x in zip(track_y, track_x):
            track_edges[counter] = [x, y]
            counter += 1
        return track_edges

    def findCurve(self, sensors):
        track_edges = self.calculateTrackEdges(sensors)

        leftMax = np.argmax(sensors.track)
        rightMax = len(sensors.track) - np.argmax(sensors.track[::-1]) - 1

        rightTrack = track_edges[rightMax + 1:, :]
        leftTrack = track_edges[:leftMax, :]
        if not len(rightTrack) or not len(leftTrack):
            raise ValueError('empty left or right track sensors')
        
        rightPoly = np.polyfit(rightTrack[::-1, 0], rightTrack[::-1, 1], 2)
        leftPoly = np.polyfit(leftTrack[:, 0], leftTrack[:, 1], 2)

        rightCurve = ((1 + (2 * rightPoly[0] * rightTrack[-1, 0] + rightPoly[1]) ** 2) ** 1.5) / np.absolute(2 * rightPoly[0])
        leftCurve = ((1 + (2 * leftPoly[0] * leftTrack[0, 0] + leftPoly[1]) ** 2) ** 1.5) / np.absolute(2 * leftPoly[0])
        if leftMax > 9:
            curve = leftCurve
        elif rightMax < 9:
            curve = rightCurve
        else:
            curve = (rightCurve + leftCurve) / 2
        self._logger.debug('computed curvature: %s', curve)
        return curve

    def isCurve(self, curvature, max_speed, steer_lock):
        """
        True if angular velocity > steer max.
        """

        return max_speed / curvature > steer_lock

    def findTrackPoly(self, state):
        track_edges = self.calculateTrackEdges(state)

        leftMax = np.argmax(state.track)
        rightMax = len(state.track) - np.argmax(state.track[::-1]) - 1

        rightTrack = track_edges[rightMax + 1:, :]
        leftTrack = track_edges[:leftMax, :]

        rightPoly = np.polyfit(rightTrack[::-1, 0], rightTrack[::-1, 1], 2)
        leftPoly = np.polyfit(leftTrack[:, 0], leftTrack[:, 1], 2)

        return rightPoly, leftPoly

    def findDriversLocations(self, state):
        # py 2.7
        angles = range(-170, 181, 10)  # check that it creats list
        drivers_y = np.array(state.opponets) * np.sin(np.deg2rad(self.angles))
        drivers_x = np.array(state.opponets) * np.cos(np.deg2rad(self.angles))

        drivers_edges = np.c_[drivers_x, drivers_y]
        return drivers_edges

    def locateDrivers(self, sensors, drivers_edges):
        right_poly, left_poly = self.findTrackPoly(sensors)
        indexes = sensors.opponets < 200
        drivers = []
        for index in np.nonzero(indexes)[0]:
            x_driver = drivers_edges[index, 0]
            y_driver = drivers_edges[index, 1]
            y_right = np.polyval(right_poly, x_driver)
            y_left = np.polyval(left_poly, x_driver)
            self.numOfLanes = len(self.lanes)
            laneWidth = 2 / self.numOfLanes
            bins = np.linspace(y_left, y_right, self.numOfLanes + 1)

            driver_lane = np.digitize(y_driver, bins[::-1]) - 1
            drivers.append([driver_lane,sensors.opponets[index]])
        return drivers
    
    def isCarAhead(self,drivers,lane):
        areThere = False
        min_distance = -1
        drivers_lanes = drivers[:,0]
        drivers_in_my_lane = np.nonzero(drivers_lanes == lane)[0]
        if len(drivers_in_my_lane):
            areThere = True
            min_distance = np.min(drivers_lanes(drivers_in_my_lane))
        return areThere, min_distance