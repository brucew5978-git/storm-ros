from storm_lib import storm
import time


if __name__ == '__main__':
    robot = storm.Storm()
    for i in range(0,100):
        robot.set_vel(0.1,0.0)
        time.sleep(1)
        robot.set_vel(0.0,0.0)
        time.sleep(1)
