from .ordinary_car import Ordinary_Car
from .. import config

class Controller:
    # Constructor
    def __init__(self, ref: float = config.DEF_REF):
        self.car = Ordinary_Car()
        self.ref = ref
        self.Kp = config.Kp

    # Main function to follow the target
    def follow_target(self):
        print("Following the target...")
        while True:
            dist = self.car.sensor.get_distance()
            speed = (dist-self.ref) * self.Kp;
            duty = int(speed * config.SPEED_TO_DUTY_RATIO)
            self.car.set_motor_model(duty, duty, duty, duty); 

    # Distructor
    def close(self):
        self.car.close()