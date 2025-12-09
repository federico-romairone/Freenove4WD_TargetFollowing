from .ordinary_car import Ordinary_Car
import config

class Controller:
    # Constructor
    def __init__(self, ref: float = config.DEF_REF):
        self.car = Ordinary_Car()
        self.ref = ref

    # Main function to follow the target
    def follow_target(self):
        dist = self.car.sensor.get_distance()
        print("Initial distance: %.2f", dist)

    # Distructor
    def close(self):
        self.car.close()