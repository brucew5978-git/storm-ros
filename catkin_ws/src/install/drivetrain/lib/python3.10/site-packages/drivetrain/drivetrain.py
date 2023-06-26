#! /home/odroid/code/catkin_ws/src/drivetrain/venv/bin/python3

import board
import busio
import time
import adafruit_motorkit
import board


def main():
    print('drivetrain main')
    print("making motor kit")
    i2c0 = busio.I2C(board.SCL0, board.SDA0)
    kit = adafruit_motorkit.MotorKit(i2c=i2c0)
    
    for i in range(100):
        print("step")
        kit.stepper1.onestep()


if __name__ == '__main__':
    main()
