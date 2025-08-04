from tools import *
hub = None#EV3Brick()
Lw = None#BetterMotor(Port.A, positive_direction=True)
Rw = None#BetterMotor(Port.B, positive_direction=False)
Pw = None#BetterMotor(Port.C, positive_direction=True)  
drive = DBase(hub, Lw, Rw, Pw)

drive.set_pos(0, 0, 0)
speed = 500
pos=(100, 100)
corector_cons = 10
# aktuální pozice
x1 = drive.active_areas[0].x
y1 = drive.active_areas[0].y
angle1 = drive.active_areas[0].angle

# nová pozice
x2 = pos[0]
y2 = pos[1]

# relativní pozice
dx = x2 - x1
dy = y2 - y1

track_angle = degrees(atan2(dx, dy))
print("track_angle = ", track_angle)
while True:
    print(drive.active_areas[0].transfer(int(input("distance = ")), int(input("angle_shift = "))))
    xi, yi = drive.active_areas[0].x, drive.active_areas[0].y
    corector = sin(radians(track_angle)) * (yi / tan(radians(track_angle - xi)))*corector_cons
    print("corector = ", corector/5)
    Lw_speed = speed - corector
    Rw_speed = speed + corector
    print("Lw_speed = ",Lw_speed, " Rw_speed = ",Rw_speed)