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