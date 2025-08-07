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
    #value = (value + 180) % 360 - 180
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
        self.i_angle = 0

    def transfer(self, distance, angle):
        self.angle = angle - self.i_angle
        self.x += distance * cos(radians(self.angle))
        self.y += distance * sin(radians(self.angle))
        return self.x, self.y, self.angle

    def set_pos(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def reset(self):
        self.set_pos(0, 0, 0)

    def dir_reset(self, i_angle):
        self.i_angle = i_angle
        self.reset()

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
        self.Lw.mangle = 0
        self.Rw.mangle = 0
        self.Lw.reset_angle(0)
        self.Rw.reset_angle(0)
        self.gyros = []  # list of gyro sensors
        self.m_angle = 0

    def add_gyro(self, gyro: GyroSensor):
        self.gyros.append(gyro)

    def read_gyros(self, m_angle, odchlka=1000):
        angle = 0
        if self.gyros != []:
            for gyro in self.gyros:
                g_angle = gyro.angle()
                if abs(g_angle - m_angle) < odchlka:
                    angle += g_angle
                else:
                    angle += m_angle
            return angle/len(self.gyros)
        else:
            return m_angle


    def set_pos(self, x, y, angle, area_N = 0):
        """for manual locate
        area_N: 0 ... main area"""
        self.active_areas[area_N].set_pos(x, y, angle)

    def track(self):
        Lw_angle_shift = self.Lw.angle() - self.Lw.mangle
        Rw_angle_shift = self.Rw.angle() - self.Rw.mangle
        self.Lw.mangle = self.Lw.angle()
        self.Rw.mangle = self.Rw.angle()
        self.m_angle += (Rw_angle_shift - Lw_angle_shift) * self.wheel_radius / self.axle_track
        angle = self.read_gyros(self.m_angle)
        distance = (Lw_angle_shift + Rw_angle_shift) * self.wheel_radius * pi / 360
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
    
    def speed_calculator(self, motor_angle, speed, terminal_speed, g_cons, corector_cons):
        new_speed = self.accelerator(self.Lw.angle() - self.start_motor_angle, motor_angle, speed, initial_speed=self.average_motor_speed, terminal_speed=terminal_speed)
        gyro_corection = self.active_areas[1].angle * g_cons
        shift_corection = self.active_areas[1].y * corector_cons
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
        y_shift = (y - self.active_areas[Area_N].y)
        self.start_motor_angle = self.Lw.angle()
        print("navigation running")
        
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
        self.active_areas[1].dir_reset(track_angle)
        self.average_motor_speed = (self.Lw.speed() + self.Rw.speed()) / 2
        
        self.locate()
        print("to travel: ", x, y, " distance: ", distance, " ang: ", track_angle)
        print("i_ang: ", self.active_areas[1].i_angle, "  angle1: ", self.active_areas[1].angle, "angle0: ", self.active_areas[0].angle)

        while True:
            self.locate()
            #print("actual position: ", self.active_areas[0].x, self.active_areas[0].y, "  angle: ", self.active_areas[0].angle)
            self.speed_calculator(motor_angle, speed, terminal_speed, g_cons, corector_cons)
            #print("L_speed: ", self.L_speed, "  R_speed: ", self.R_speed, " X: ", self.active_areas[1].x, "  Y: ", self.active_areas[1].y, "  angle: ", self.active_areas[1].angle)
            self.motor_driver(self.L_speed, self.R_speed)

            #motor breaker
            if abs(self.active_areas[1].x) > abs(distance): 
                self.motor_braker(stop)
                #print(stop, self.x, self.y)
                print("task done", self.active_areas[1].x, distance)
                break


hub = EV3Brick()
gyro1 = GyroSensor(Port.S1)
gyro2 = GyroSensor(Port.S4)
Lw = Motor(Port.A)
Rw = Motor(Port.B)
Pw = Motor(Port.C)  
hub.speaker.beep(1000, 200)


drive = DBase(hub, Lw, Rw, Pw)
drive.add_gyro(gyro1)
drive.add_gyro(gyro2)

print("on position: ", drive.active_areas[0].x, drive.active_areas[0].y, "  angle: ", drive.active_areas[0].angle)
drive.straight_position(400, 0, 1)
drive.straight_position(400, 400, 1)
drive.straight_position(0, 400, 1)
drive.straight_position(0, 0, 1)

#while True:
#    #print(drive.track())
#    drive.locate()
#    print(drive.active_areas[0].x, drive.active_areas[0].y, "  angle: ", drive.active_areas[0].angle)
#    wait(10)