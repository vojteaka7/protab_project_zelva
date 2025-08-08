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

class BetterMotor(Motor):
    def __init__(self, port, positive_direction=Direction.CLOCKWISE):
        self.mangle = 0
        self.reset_angle(0)
        self.positive_direction = positive_direction
        super().__init__(port, positive_direction)
        

    def angle_shift(self):
        shift = self.angle() - self.mangle
        self.mangle = self.angle()
        return shift

    def straight_g(distance, angle, terminal_speed: Number = 50, speed = 500, Area_N: int = 0):
        """
        extra precise straight movement
        
        Parameters:
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