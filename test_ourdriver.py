import logging

from carControl import CarControl
from carState import CarState
from ourdriver import OurDriver

def getDriver():
    return OurDriver(3, 'FORWARD,FORWARD', 0)

def test_keeplane():
    driver = getDriver()
    control = CarControl()
    sensor_state = CarState()
    driver.is_stuck = lambda: False
    sensor_state._angle = 0.5
    sensor_state._trackPos = 0.3
    driver.drive_from_state(sensor_state, control)
    print control.toMsg()
    assert True

def test_emergency():
    driver = getDriver()
    control = CarControl()
    sensor_state = CarState()
    driver.ifIsGoingToCrash = lambda: True
    driver.drive_from_state(sensor_state, control)
    assert True

def test_turn():
    driver = getDriver()
    control = CarControl()
    sensor_state = CarState()
    driver.checkIfIsInTurn = lambda: True
    driver.drive_from_state(sensor_state, control)
    assert True
    
def main():
    # test_emergency()
    # test_turn()
    test_keeplane()

if __name__ == '__main__':
    logging.basicConfig(level=10)
    main()
