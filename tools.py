#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.ev3devices import *
from pybricks.robotics import *
from math import cos, sin, tan, radians, pi, degrees, sqrt

class Area:

    def __init__(self, x = 0 , y = 0, angle = 0):
        # self.reset()
        self.magnification = 1.0
        self.points = []

        self.x = x
        self.y = y
        self.angle = angle

    def transfer(self, distance, angle_shift):
        self.x += distance * cos(radians(angle_shift + self.angle))
        self.y += distance * sin(radians(angle_shift + self.angle))
        self.angle += angle_shift
    
    def set_pos(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def reset(self):
        self.set_pos(0, 0, 0)

class BetterMotor(Motor):
    def __init__(self, port, positive_direction=Direction.CLOCKWISE):
        super().__init__(port, positive_direction)
        self.mangle = 0
        self.reset_angle(0)
        self.positive_direction = positive_direction

    def angle_shift(self):
        shift = self.angle() - self.mangle
        self.mangle = self.angle()
        return shift

class DBase:
    def __init__(self, hub: EV3Brick, Lw: BetterMotor, Rw: BetterMotor, Pw: BetterMotor, wheel_radius=22, axle_track=175):
        self.hub = hub
        self.Lw = Lw
        self.Rw = Rw
        self.Pw = Pw # pen motor
        self.wheel_radius = wheel_radius
        self.axle_track = axle_track
        self.is_pen_up = False
        self.active_areas = [Area()] #those are areas where the position is tracked
        
    def set_pos(self, x, y, angle, area_N = 0):
        """for manual locate
        area_N: 0 ... main area"""
        self.active_areas[area_N].set_pos(x, y, angle)

    def track(self):
        angle = abs(self.Lw.angle_shift() - self.Rw.angle_shift()) * self.wheel_radius / self.axle_track
        distance = (self.Lw.angle_shift() + self.Rw.angle_shift()) * self.wheel_radius * pi
        return distance, angle

    def locate(self):
        distance, angle_shift = self.track()
        for area in self.active_areas:
            area.transfer(distance, angle_shift)

    def topos(self, pos: tuple, speed = 500, area_N = 0):
        '''move to position (x, y)'''

        # aktuální pozice
        x1 = self.active_areas[area_N].x
        y1 = self.active_areas[area_N].y

        # nová pozice
        x2, y2 = pos

        # relativní pozice
        dx = x2 - x1
        dy = y2 - y1

        distance = sqrt(dx**2 + dy**2)

        angle1 = self.active_areas[area_N].angle
        angle2 = degrees(tan(dy, dx))

        dangle = angle2 - angle1

        # TODO: zakomponovat tyhle dvě fce do fce topos

    def drive_forward(self, distance, speed = 500):
        angle = (distance * 360) / (2 * pi * self.wheel_radius)
        self.Lw.run_angle(speed, angle, wait=False)
        self.Rw.run_angle(speed, angle)
    
    def rotate(self, target_angle, speed = 500):
        wheel_angle = ( self.axle_track * target_angle ) / ( self.wheel_radius * 2 )
        self.Lw.run_angle(speed, wheel_angle, wait=False)
        self.Rw.run_angle(speed, -wheel_angle)

    def move_pen_up(self):
        if not self.is_pen_up:
            self.Pw.run_angle(500, 20)
            self.is_pen_up = True
    
    def move_pen_down(self):
        if self.is_pen_up:
            self.Pw.run_angle(500, -20)
            self.is_pen_up = False