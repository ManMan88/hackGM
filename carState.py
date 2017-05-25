"""
Created on Apr 5, 2012

@author: lanquarden
Edited by General Motors
"""
import msgParser

class CarState(object):
    """
    Class that hold all the car state variables
    """
    def __init__(self):

        """Constructor"""
        self.parser = msgParser.MsgParser()
        self.sensors = None
        self._angle = None
        self._curLapTime = None
        self._damage = None
        self._distFromStart = None
        self._distRaced = None
        self._focus = None
        self._fuel = None
        self._gear = None
        self._lastLapTime = None
        self._opponents = None
        self._racePos = None
        self._rpm = None
        self._speedX = None
        self._speedY = None
        self._speedZ = None
        self._track = None
        self._trackPos = None
        self._wheelSpinVel = None
        self._z = None
    
    def setFromMsg(self, str_sensors):
        self.sensors = self.parser.parse(str_sensors)
        
        self.setAngleD()
        self.setCurLapTimeD()
        self.setDamageD()
        self.setDistFromStartD()
        self.setDistRacedD()
        self.setFocusD()
        self.setFuelD()
        self.setGearD()
        self.setLastLapTimeD()
        self.setOpponentsD()
        self.setRacePosD()
        self.setRpmD()
        self.setSpeedXD()
        self.setSpeedYD()
        self.setSpeedZD()
        self.setTrackD()
        self.setTrackPosD()
        self.setWheelSpinVelD()
        self.setZD()
    
    def toMsg(self):
        self.sensors = {}
        
        self.sensors['angle'] = [self._angle]
        self.sensors['curLapTime'] = [self._curLapTime]
        self.sensors['damage'] = [self._damage]
        self.sensors['distFromStart'] = [self._distFromStart]
        self.sensors['distRaced'] = [self._distRaced]
        self.sensors['focus'] = self._focus
        self.sensors['fuel'] = [self._fuel]
        self.sensors['gear'] = [self._gear]
        self.sensors['lastLapTime'] = [self._lastLapTime]
        self.sensors['opponents'] = self._opponents
        self.sensors['racePos'] = [self._racePos]
        self.sensors['rpm'] = [self._rpm]
        self.sensors['speedX'] = [self._speedX]
        self.sensors['speedY'] = [self._speedY]
        self.sensors['speedZ'] = [self._speedZ]
        self.sensors['track'] = self._track
        self.sensors['trackPos'] = [self._trackPos]
        self.sensors['wheelSpinVel'] = self._wheelSpinVel
        self.sensors['z'] = [self._z]
        
        return self.parser.stringify(self.sensors)
    
    def getFloatD(self, name):
        try:
            val = self.sensors[name]
        except KeyError:
            val = None
        
        if val != None:
            val = float(val[0])
        
        return val
    
    def getFloatListD(self, name):
        try:
            val = self.sensors[name]
        except KeyError:
            val = None
        
        if val != None:
            l = []
            for v in val:
                l.append(float(v))
            val = l
        
        return val
    
    def getIntD(self, name):
        try:
            val = self.sensors[name]
        except KeyError:
            val = None
        
        if val != None:
            val = int(val[0])
        
        return val

    def setAngleD(self):        
        self._angle = self.getFloatD('angle')

    @property
    def angle(self):
        return self._angle
    
    def setCurLapTimeD(self):
        self._curLapTime = self.getFloatD('curLapTime')

    @property
    def curLapTime(self):
        return self._curLapTime
    
    def setDamageD(self):
        self._damage = self.getFloatD('damage')

    @property
    def damage(self):
        return self._damage
    
    def setDistFromStartD(self):
        self._distFromStart = self.getFloatD('distFromStart')

    @property
    def distFromStart(self):
        return self._distFromStart
    
    def setDistRacedD(self):
        self._distRaced = self.getFloatD('distRaced')

    @property
    def distRaced(self):
        return self._distRaced
    
    def setFocusD(self):
        self._focus = self.getFloatListD('focus')
    
    def setFuelD(self):
        self._fuel = self.getFloatD('fuel')

    @property
    def fuel(self):
        return self._fuel
    
    def setGearD(self):
        self._gear = self.getIntD('gear')

    @property
    def gear(self):
        return self._gear

    def setLastLapTimeD(self):
        self._lastLapTime = self.getFloatD('lastLapTime')

    def setOpponentsD(self):
        self._opponents = self.getFloatListD('opponents')

    @property
    def opponents(self):
        return self._opponents
    
    def setRacePosD(self):
        self._racePos = self.getIntD('racePos')

    @property
    def racePos(self):
        return self._racePos
    
    def setRpmD(self):
        self._rpm = self.getFloatD('rpm')

    @property
    def rpm(self):
        return self._rpm

    def setSpeedXD(self):
        self._speedX = self.getFloatD('speedX')

    @property
    def speedX(self):
        return self._speedX
    
    def setSpeedYD(self):
        self._speedY = self.getFloatD('speedY')

    @property
    def speedY(self):
        return self._speedY
    
    def setSpeedZD(self):
        self._speedZ = self.getFloatD('speedZ')

    @property
    def speedZ(self):
        return self._speedZ
    
    def setTrackD(self):
        self._track = self.getFloatListD('track')

    @property
    def track(self):
        return self._track
    
    def setTrackPosD(self):
        self._trackPos = self.getFloatD('trackPos')

    @property
    def trackPos(self):
        return self._trackPos
    
    def setWheelSpinVelD(self):
        self._wheelSpinVel = self.getFloatListD('wheelSpinVel')

    @property
    def wheelSpinVel(self):
        return self._wheelSpinVel
    
    def setZD(self):
        self._z = self.getFloatD('z')

    @property
    def z(self):
        return self._z
    
    