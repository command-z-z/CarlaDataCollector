"""
#Values    Name      Description
----------------------------------------------------------------------------
   1    type                 Describes the type of object: 'Car', 'Pedestrian', ‘Vehicles’
                            ‘Vegetation’, 'TrafficSigns', etc.
   3    velocity             velocity of the object, returns three absolute values of the components x, y and z.
   3    acceleration         acceleration of the objects, returns three absolute values of the components x, y and z.
   3    angular_velocity     angular_velocity of the objects, returns three absolute values of the components x, y and z.
"""

class CarlaLabel:
    def __init__(self):
        self.type = None
        self.velocity = None
        self.acceleration = None
        self.angular_velocity = None


    def set_type(self, obj_type: str):
        self.type = obj_type

    def set_velocity(self, velocity):
        self.velocity = velocity

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration

    def set_angular_velocity(self, angular_velocity):
        self.angular_velocity = angular_velocity

    def __str__(self):
        return "{} {} {} {}".format(self.type, self.velocity, self.acceleration, self.angular_velocity)

