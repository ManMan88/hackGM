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
    sensor_state._track = [10, 10, 10, 10, 10, 10, 50, 60, 100, 100, 200, 10, 10, 10, 50, 10, 10, 10, 10]

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
    driver.checkIfIsInTurn = lambda s: True
    driver.drive_from_state(sensor_state, control)
    assert True


def test_backOnTrack():
    driver = getDriver()
    control = CarControl()
    sensor_state = CarState()
    driver.ifIsStuck = lambda s: True
    sensor_state._angle = 0.5
    sensor_state._trackPos = 0.3
    driver.drive_from_state(sensor_state, control)
    assert True


def main():
    test_emergency()
    test_turn()
    test_keeplane()
    test_backOnTrack()


# if __name__ == "__main__":
#     sl = SwitchLane(-1., -0.3333, 0.5, 0.0, 0, 5)
#     state = object()
#     steers = np.empty(10)
#
#     for t in xrange(10):
#         steers[t] = sl.get_steer(t)
#
#     print steers

if __name__ == '__main__':
    main()
