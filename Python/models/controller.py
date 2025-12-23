from .ordinary_car import Ordinary_Car
from .. import config
from .. import utility
import time
import csv

class Controller:
    # Constructor
    def __init__(self, ref: float = config.DEF_REF):
        self.car = Ordinary_Car()
        self.ref = ref
        self.Kp = config.Kp
        self.out_file = None
        self.writer = None
        
        if config.WRITE_OUT:
            # init writer object
            filename = f"{time.strftime("%Y%m%d_%H%M%S")}_{config.FOUT_NAME}.{config.FOUT_EXT}"
            self.out_file = open(filename, 'w', newline='') 
            self.writer = csv.writer(self.out_file)
            # header 
            self.writer.writerow(['Elapsed time (s)','Distance (cm)','Speed (cm/s)','Duty (PWM)'])
            if config.DEBUG:
                print(f"Ready to write on '{filename}'!")

    # Main function to follow the target
    def follow_target(self):
        """
        In an infinite loop, this function leads the motors in order to
        follow a target, mantaining a specific reference distance.
        """
        print("Following the target...")
        
        # simulation start timestamp
        start_time = time.time()
        
        old_dist = self.car.sensor.get_distance()
        while True:
            dist = self.car.sensor.get_distance()
            # prevent small oscillations
            dist = utility.suppress_oscillations(old_dist, dist, config.EPS)
            speed = (self.ref-dist) * self.Kp
            # staurate the speed
            if config.SATURATE:
                speed = utility.saturation(speed, config.MAX_SPEED)
            duty = int(speed * config.SPEED_TO_DUTY_RATIO)
            # saturate the duty
            if config.SATURATE:
                duty = int(utility.saturation(duty, config.MAX_DUTY))
            duties = [duty] * 4
            # calibrate, if you need to (I needed)
            if config.CALIBRATE:
                utility.apply_calibration(duties)
            # debug
            if config.DEBUG:
                elapsed_time = time.time() - start_time
                print(f"Elapsed time = {utility.get_time_format(elapsed_time)} Dist = {dist:5.2f} cm\nSpeed = {speed:5.2f} cm/s\nDuty = {duty:5d} PWM", end="\r")
            # write data on csv output file
            if config.WRITE_OUT and self.writer and self.out_file:
                elapsed_time = time.time() - start_time
                self.writer.writerow([elapsed_time, dist, speed, duty])
                self.out_file.flush()
            
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
        if self.out_file:
            self.out_file.close()
        self.car.close()
