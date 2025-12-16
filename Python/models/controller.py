from .ordinary_car import Ordinary_Car
from .. import config
from .. import utility
import time

class Controller:
    # Constructor
    def __init__(self, ref: float = config.DEF_REF):
        self.car = Ordinary_Car()
        self.ref = ref
        self.Kp = config.Kp

    # Main function to follow the target
    def follow_target(self):
        """
        In an infinite loop, this function leads the motors in order to
        follow a target, mantaining a specific reference distance.
        """
        print("Following the target...")
        old_dist = self.car.sensor.get_distance()
        while True:
            dist = self.car.sensor.get_distance()
            # prevent small oscillations
            dist = utility.suppress_oscillations(old_dist, dist, config.EPS)
            speed = (dist-self.ref) * self.Kp
            # staurate the speed
            speed = utility.saturation(speed, config.MAX_SPEED)
            duty = int(speed * config.SPEED_TO_DUTY_RATIO)
            # saturate the duty
            duty = int(utility.saturation(duty, config.MAX_DUTY))
            duties = [duty] * 4
            # calibrate, if you need to (I needed)
            if config.CALIBRATE:
                utility.apply_calibration(duties)
            # debug
            print("dist=", dist, end="\r")
            self.car.set_motor_model(duties); 
            old_dist = dist
    
    def test(self):
        duties = [config.MAX_DUTY]*4
        if config.CALIBRATE:
            utility.apply_calibration(duties)
            print("Calibration applied!")
        self.car.set_motor_model(duties)
        time.sleep(2.5)
        self.car.set_motor_model([0]*4)

    # Distructor
    def close(self):
        self.car.close()
