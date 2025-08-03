#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.ev3devices import *
from math import cos, sin, radians, pi, degrees, average

class Area:
    def __init__(self):
        self.reset()
        self.magnification = 1.0
        self.points = []

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
    def __init__(self, port, positive_direction=True):
        super().__init__(port, positive_direction)
        self.mangle = 0
        self.reset_angle(0)

    def angle_shift(self):
        shift = self.angle() - self.mangle
        self.mangle = self.angle()
        return shift

    def reset_angle(self, angle):
        super().reset_angle(angle * (1 if self.positive_direction else -1))

class DBase:
    def __init__(self, hub: EV3Brick, Lw: BetterMotor, Rw: BetterMotor, Pw: BetterMotor, wheel_radius=22, axle_track=175):
        self.hub = hub
        self.Lw = Lw
        self.Rw = Rw
        self.Pw = Pw # pen motor
        self.wheel_radius = wheel_radius
        self.axle_track = axle_track
        self.is_pen_up = False
        self.active_areas = [Area(0, 0, 0)] #those are areas where the position is tracked
        
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

    def topos(self, pos, speed = 500, area_N = 0):
        '''move to position'''

    def move_pen_up(self):
        if not self.is_pen_up:
            self.Pw.run_angle(500, 20)
    
    def move_pen_down(self):
        if self.is_pen_up:
            self.Pw.run_angle(500, -20)
        
