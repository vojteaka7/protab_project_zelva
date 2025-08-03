#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.ev3devices import *
from math import cos, sin, radians, pi, degrees, average

class Area:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.anlge = angle
        self.magnification = 1.0
        self.points = []

    def transfer(self, distance, angle_shift):
        self.x += distance * cos(angle_shift + self.angle)
        self.y += distance * sin(angle_shift + self.anlge)

class DBase:
    def __init__(self, ev3: EV3Brick):
        self.ev3 = ev3
        self.left_motor = Motor(Port.B)
        self.right_motor = Motor(Port.C)
        self.color_sensor = ColorSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S2)
        self.touch_sensor = TouchSensor(Port.S3)

    def drive(self, speed, turn_rate):
        self.left_motor.run(speed - turn_rate)
        self.right_motor.run(speed + turn_rate)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()