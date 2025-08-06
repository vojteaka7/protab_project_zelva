#!/usr/bin/env pybricks-micropython
#from tools import *
from pybricks.hubs import EV3Brick
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.ev3devices import *

hub = EV3Brick()
gyro1 = GyroSensor(Port.S1)
gyro2 = GyroSensor(Port.S4)
#Lw = BetterMotor(Port.A)
#Rw = BetterMotor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
#Pw = BetterMotor(Port.C)  
hub.speaker.beep(1000, 1000)
while True:
    ang1 = gyro1.angle()
    ang2 = gyro2.angle()
    print("angle = ", (ang1 + ang2)/2, "  angle1 = ", ang1, "  angle2 = ", ang2)



drive = DBase(hub, Lw, Rw, Pw)
hub.speaker.beep(1000, 1000)
drive.straight_position(100, 0, 1)