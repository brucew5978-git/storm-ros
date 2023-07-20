import storm_lib
import time


if __name__ == '__main__':
    robot = storm_lib.Storm()
    for i in range(0,100):
        robot.set_vel(0.1,0.0)
        time.sleep(100)
        robot.set_vel(0.0,0.0)
        time.sleep(100)