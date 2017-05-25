from enum import Enum
import numpy as np
from scipy.constants import degree

class Direction(Enum):
    FORWARD = 1
    OPPOSITE = -1

class LowLevelDriver(object):

    def __init__(self, lanes_str):
        self.angles = np.zeros(19)
        
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15

        for i in range(5, 9):
            self.angles[i] = -20 + (i-5) * 5
            self.angles[18 - i] = 20 - (i-5) * 5

        self.lanes = [Direction[x] for x in lanes_str.split(',')]
        self.lane = -1
        
    def calculateLanesData(self):
        self.numOfLanes = len(self.lanes)
        laneWidth = 2/self.numOfLanes
        self.lanesPositions = [(-1 + i * laneWidth) for i in range(0, self.numOfLanes)]
        #if(state.trackpos > 1  or state.trackpos < -1):
        #    pass            #todo
        #else:
        #    self.lane = np.where(self.lanesPositions <state.trackpos)[0]
        
    def findLaneBorders(self, lane):
        leftSide=self.lanesPositions[lane]
        if(lane == self.numOfLanes - 1):
            rightSide=1
        else:
            rightSide=self.lanesPositions[lane+1]
        return leftSide, rightSide
    
    def calculateTrackEdges(self, state):
        """
        Convert sensor data to cartesian lane edges.
        """
        track_y = np.array(state.track)*np.sin(np.deg2rad(self.angles))
        track_x = np.array(state.track)*np.cos(np.deg2rad(self.angles))
        track_edges=np.zeros((19,2))
        counter=0
        for y,x in zip(track_y,track_x):
            track_edges[counter]=[x,y]
            counter+=1
        return track_edges
        
    def findCurve(self, state):
        track_edges = self.calculateTrackEdges(state)
        
        print track_edges
        
        leftMax =  np.argmax(state.track)
        rightMax =  len(state.track) - np.argmax(state.track[::-1]) -1
        
        rightTrack = track_edges[rightMax+1:,:]
        leftTrack = track_edges[:leftMax,:]
        
        rightPoly = np.polyfit(rightTrack[::-1,0],rightTrack[::-1,1],2)
        leftPoly =  np.polyfit(leftTrack[:,0],leftTrack[:,1],2)
        
        rightCurve = ((1 + (2*rightPoly[0]*rightTrack[-1,0] + rightPoly[1])**2)**1.5) / np.absolute(2*rightPoly[0])
        leftCurve = ((1 + (2*leftPoly[0]*leftTrack[0,0] + leftPoly[1])**2)**1.5) / np.absolute(2*leftPoly[0])
        curve = (rightCurve+leftCurve)/2
        return curve
        
        
    
        
        
        
        
