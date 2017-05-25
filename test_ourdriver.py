from carControl import CarControl
from carState import CarState
from ourdriver import OurDriver


def main():
    driver = OurDriver(3, 'FORWARD,FORWARD', 0)
    control = CarControl()
    sensor_state = CarState()
    driver.drive_from_state(sensor_state, control)


if __name__ == '__main__':
    main()