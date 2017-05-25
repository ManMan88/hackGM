from enum import Enum
import numpy as np

class Direction(Enum):
    FORWARD = 1
    OPPOSITE = -1

class LowLevelDriver(object):

    def __init__(self, lanes_str, state):
        self.angles = np.zeros(19)
        
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15

        for i in range(5, 9):
            self.angles[i] = -20 + (i-5) * 5
            self.angles[18 - i] = 20 - (i-5) * 5

        self.lanes = [Direction[x] for x in lanes_str.split(',')]
        self.state = state
        self.lane = -1
        self.calculateLanesData()
        
    def calculateLanesData(self):
        numOfLanes=len(self.lanes)
        laneWidth = 2/numOfLanes
        self.lanesPositions = [(-1 + i * laneWidth) for i in range(0, numOfLanes)]
        if(self.state.trackpos > 1  or self.state.trackpos < -1):
            pass            #todo
        else:
            self.lane = np.where(self.lanesPositions <self.state.trackpos)[0]
    def findLaneBorders(self):
        leftSide=self.lanesPositions[self.lane]
        if(self.lane == self.numOfLanes - 1):
            rightSide=1
        else:
            rightSide=self.lanesPositions[self.lane+1]
        return (leftSide, rightSide)
    
    def calculateTrackEdges(self):
        track_y = np.array(self.state.track)*np.cos(self.state.track)
        track_x = np.array(self.state.track)*np.sin(self.state.track)
        track_edges=np.zeros((19,2))
        counter=0
        for y,x in zip(track_y,track_x):
            track_edges[counter]=[x,y]
            counter+=1
        return track_edges
        
    def findGeneralDirection(self):
        leftMax =  np.argmax(self.state.track)
        rightMax =  len(self.state.track) - np.argmax(np.flip(self.state.track) -1
        rightTrack = track_edges[rightMax+1:,:]
        leftTrack = track_edges[:leftMax,:]
        rightPoly = np.polyfit(rightTrack[:,0],rightTrack[:,1])
        leftPoly =  np.polyfit(leftTrack[:,0],leftTrack[:,1])
        rightCurve = ((1 + (2*rightPoly[0]*self.rightTrack[:,0] + rightPoly[1])**2)**1.5) / np.absolute(2*rightPoly[0])
        leftCurve = ((1 + (2*leftPoly[0]*self.leftTrack[:,0] + leftPoly[1])**2)**1.5) / np.absolute(2*leftPoly[0])
        curve = (rightCurve+leftCurve)/2
        return curve
        
        
    
        
        
        
        