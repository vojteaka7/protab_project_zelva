#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.ev3devices import *
from math import cos, sin, tan, radians, pi, degrees, sqrt, atan2

def sign(value: float, zero: bool = True):
    if value == 0 and zero:
        sign = 0
    elif value < 0:
        sign = -1
    else:
        sign = 1
    return sign

def avrg(*value: float):
    avrg = sum(value)/len(value)
    return avrg

def angle_mod(value: float):
    value = (value + 180) % 360 - 180
    return value

def clamp(value: float, maximum: float, minimum: float):
    """
    keeps the value between maximum and minimum

    Parameters:
        - value: float
        - maximum: float
        - minimum: float

    Returns:
        - value between maximum and minimum
    """
    if maximum >= value >= minimum:
        final_value = value
    elif maximum < value:
        final_value = maximum
    elif minimum > value:
        final_value = minimum
    return final_value

def absclamp(value: float, maximum: float, minimum: float):
    """
    keeps absolute value of the value between maximum and minimum

    Parameters:
        - value: float
        - maximum: float
        - minimum: float

    Returns:
        - value float
    """
    maximum = abs(maximum)
    minimum = abs(minimum)
    vsign = sign(value)
    if maximum >= abs(value) >= minimum:
        final_value = value
    elif value == 0:
        final_value = 0
    elif maximum < abs(value):
        final_value = maximum*vsign
    elif minimum > abs(value):
        final_value = minimum*vsign
    return final_value

def motor_corector(L_angle, R_angle, speed, ratio: Number=1, extra_condition=True):#not shure how useful
    """
    - this function calculates speed of one motor (right) from difference of motor angles
    - this could be use only for functions which use both wheels
    - 'vyrovnávač pohybu'

    Parameters:
        - L_angle: Number - deg - motor angle of the motor whose speed isn't calculated by this function (left)
        - R_angle: Number - deg - motor angle of the motor whose speed is calulated by this function (right)
        - speed: Number - deg/s
        - extra_condition: if you have some conditions when to calculate and when not, write them here
    """
    constant = 1
    # this constant is how strong the motor corection might be
    if extra_condition:
        R_speed = absclamp(((L_angle-R_angle)*constant)+speed, 2*speed, speed/2)*ratio
        #print(((L_angle-R_angle)*constant)+speed)
    else:
        R_speed = speed
    return R_speed

class Area:
    def __init__(self):
        self.reset()
        self.magnification = 1.0
        self.points = []

    def transfer(self, distance, angle_shift):
        self.x += distance * cos(radians(angle_shift + self.angle))
        self.y += distance * sin(radians(angle_shift + self.angle))
        self.angle += angle_shift
        return self.x, self.y, self.angle

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
    def __init__(self, hub: EV3Brick, Lw: BetterMotor, Rw: BetterMotor, Pw: BetterMotor, wheel_radius=22, axle_track=175, acceleration: float = 1, 
        deceleration: float = 1):
        self.hub = hub
        self.Lw = Lw
        self.Rw = Rw
        self.Pw = Pw # pen motor
        self.wheel_radius = wheel_radius
        self.axle_track = axle_track
        self.is_pen_up = False
        self.active_areas = [Area(), Area()] #those are areas where the position is tracked
        self.acceleration = acceleration  # in m/s^2
        self.deceleration = deceleration  # in m/s^2
        
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
        x2 = pos[0]
        y2 = pos[1]

        # relativní pozice
        dx = x2 - x1
        dy = y2 - y1

        distance = sqrt(dx**2 + dy**2)

        angle1 = self.active_areas[area_N].angle
        angle2 = tan(dy, dx)

        dangle = angle2 - angle1

        # TODO: fce "jeď dopředu" a fce "otoč se o daný úhel"
        # TODO: zakomponovat tyhle dvě fce do fce topos

    def topos_II(self, pos, speed = 500, area_N = 0, corector_cons = 10):
        '''move to position (x, y) with angle'''
        
        # aktuální pozice
        x1 = self.active_areas[area_N].x
        y1 = self.active_areas[area_N].y
        angle1 = self.active_areas[area_N].angle

        # nová pozice
        x2 = pos[0]
        y2 = pos[1]

        # relativní pozice
        dx = x2 - x1
        dy = y2 - y1

        track_angle = degrees(atan2(dx, dy))
        while True:
            #self.lacate()
            xi, yi = self.active_areas[area_N].x, self.active_areas[area_N].y
            corector = sin(radians(track_angle)) * (yi / tan(radians(track_angle - xi)))*corector_cons
            Lw_speed = speed + corector
            Rw_speed = speed - corector

    def move_pen_up(self):
        if not self.is_pen_up:
            self.Pw.run_angle(500, 20)
    
    def move_pen_down(self):
        if self.is_pen_up:
            self.Pw.run_angle(500, -20)

    def motor_driver(self, L_speed: float, R_speed: float):
        """
        - this function turns of motor based on logic imput

        Parameters
            - L_speed: Number - deg/s
            - R_speed: Number - deg/s
        """
        self.Lw.run(L_speed)
        self.Rw.run(R_speed)

    def motor_braker(self, stop):
        if stop:
            self.Lw.brake()
            self.Rw.brake()

    def accelerator(
        self,
        actual_motor_angle: float, 
        motor_angle: float, 
        speed: int, 
        min_speed: int = 50,
        initial_speed: int = 50,
        terminal_speed: int = 50
        ):
        """
        - This function works with the robot's speed and modifies it for a smooth start and stop.
        - This function is designed for being part of a loop.
        - It's recomentded to place this before motor corector.
        - This function could cause slower reactions of robot.

        Parameters:
            - actual_motor_angle: float ... in deg ... mesured motor angle, if you work with 2 motor movement, place here average angle of both motors.
            - motor_angle: float ... in deg ... AIM angle
            - speed: int ... in deg/s ... raw speed
            - min_speed: int ... in deg/s

        uses: calmp, absclamp
        """
        
        acceleration = self.acceleration
        deceleration = self.deceleration

        if deceleration == 0 or motor_angle == 0:
            max_speed = speed
        else:
            max_speed = absclamp((motor_angle - actual_motor_angle)/(deceleration*0.36), speed, terminal_speed) #0.36 = 360/1000; 1000 = max speed

        if acceleration == 0:
            new_speed = max_speed
        else:
            new_speed = absclamp((actual_motor_angle/(acceleration*3.6))**2 + abs(initial_speed) + min_speed, max_speed, min_speed)

        new_speed = new_speed * sign(motor_angle)

        return new_speed
    
    #fatal danger!!!
    def speed_calculator(self, motor_angle, speed, terminal_speed, g_cons, corector_cons):
        new_speed = self.accelerator(self.local_avr_motor_angle, motor_angle, speed, initial_speed=self.avr_initial_speed, terminal_speed=terminal_speed)
        gyro_corection = self.local_orientation * g_cons
        shift_corection = self.local_y * corector_cons
        self.L_speed = new_speed + gyro_corection + shift_corection
        self.R_speed = new_speed - gyro_corection - shift_corection  
        #print(new_speed, ";", self.L_speed, ";", self.R_speed)

    def straight_position(self, x: float, y: float, direction: int, terminal_speed: Number = 50, speed = 500, Area_N: int = 0):
        """
        extra precise straight movement
        
        Parameters:
            - x: float
            - y: float
            - direction: int
            - terminal_speed: Number - in deg/s
            - skippable: bool
            - speed: Number - in deg/s
            - taks: object - this is a looppart function that you would like to incorporate into this function (eg. print() -> print ; and it'll start printing empti lines every turn)
        
        uses: accelerator, motor_controler, motor_driver, clamp, absclamp
        """

        #setup
        g_cons = 20 # gyrocorector constant (its not recomended to set below 2 and above 50)
        corector_cons = 10 * direction
        terminal_speed = clamp(terminal_speed, 1000, 50)
        stop = terminal_speed == 50
        start_angle = self.active_areas[Area_N].angle
        direction = clamp(direction, 1, -1)
        x_shift = (x - self.active_areas[Area_N].x)*direction
        y_shift = (y - self.y)
        #print(self.x, self.y, x_shift, y_shift)
        
        #trajectory calculator
        track_angle = angle_mod(degrees(atan2(y_shift, x_shift)))
        #if direction == 0:
        #    direction = sign(90 - abs(angle_mod(track_angle - start_angle)), zero=False)
        track_angle = track_angle * direction
        distance = sqrt(x_shift**2 + y_shift**2)*direction
        if distance == 0:
            print("start=cíl")
            return None 
        motor_angle = (distance*360/ (2*pi*self.wheel_radius))
        self.active_areas[1].set_pos(0, 0, track_angle)

        while True:
            self.locate()
            self.speed_calculator(motor_angle, speed, terminal_speed, g_cons, corector_cons)
            self.motor_driver(self.L_speed, self.R_speed)

            #motor breaker
            if abs(self.active_areas[1].x) > abs(distance): 
                self.motor_braker(stop)
                #print(stop, self.x, self.y)
                break