from models.ordinary_car import Ordinary_Car
import config
import utility
import time
import csv

class Controller:
    # Constructor
    def __init__(self, ref: float = config.DEF_REF):
        self.car = Ordinary_Car()
        self.ref = ref
        self.Kp = config.Kp
        self.filename = None
        self.out_file = None
        self.writer = None
        
        if config.WRITE_OUT:
            # init writer object
            self.filename = f"{time.strftime('%Y%m%d_%H%M%S')}_{config.FOUT_NAME}.{config.FOUT_EXT}"
            self.out_file = open(self.filename, 'w', newline='') 
            self.writer = csv.writer(self.out_file)
            # header 
            self.writer.writerow(['Real elapsed time (s)', 'Elapsed time (s)','Distance (cm)','Speed (cm/s)','Duty (PWM)'])
            if config.DEBUG:
                print(f"Ready to write on '{self.filename}'!")

    # Main function to follow the target
    def follow_target(self):
        """
        In an infinite loop, this function leads the motors in order to
        follow a target, mantaining a specific reference distance.
        """
        print("Following the target...")
        
        # simulation start timestamp
        start_time = time.time()
        last_elapsed = 0
        
        old_dist = self.car.sensor.get_distance()
        while True:

            ## SAMPLING AND ELABORATION

            dist = self.car.sensor.get_distance()
            # prevent small oscillations
            dist = utility.suppress_oscillations(old_dist, dist, config.EPS)
            speed = (self.ref-dist) * self.Kp
            # staurate the speed
            if config.SATURATE:
                speed = utility.saturation(speed, config.MAX_SPEED)
            duty = utility.speed_to_PWM(speed)
            # saturate the duty
            if config.SATURATE:
                duty = int(utility.saturation(duty, config.MAX_PWM))
            duties = [duty] * 4
            # calibrate, if you need to (I needed)
            if config.CALIBRATE:
                utility.apply_calibration(duties)

            ## DEBUG AND OUTPUT

            # getting real elapsed time
            real_elapsed_time = time.time() - start_time
            # wait for sampling time
            delta = (time.time()-start_time)-last_elapsed
            while (delta < config.SAMPLING_PERIOD):
                delta = (time.time()-start_time)-last_elapsed
            elapsed_time = last_elapsed + delta
            # debug
            if config.DEBUG:
                print(f"Real elapsed time = {utility.get_time_format(real_elapsed_time)} Elapsed time = {utility.get_time_format(elapsed_time)} Dist = {dist:5.2f} cm\nSpeed = {speed:5.2f} cm/s\nDuty = {duty:5d} PWM", end="\r")
            # write data on csv output file
            if config.WRITE_OUT and self.writer and self.out_file:
                self.writer.writerow([real_elapsed_time, elapsed_time, dist, speed, duty])
                self.out_file.flush()
            
            self.car.set_motor_model(duties)
            last_elapsed = elapsed_time
            old_dist = dist
    
    def test(self):
        duties = [config.MAX_PWM]*4
        if config.CALIBRATE:
            utility.apply_calibration(duties)
            print("Calibration applied!")
        self.car.set_motor_model(duties)
        time.sleep(2.5)
        self.car.set_motor_model([0]*4)

    # Distructor
    def close(self):
        self.car.close()
        if config.WRITE_OUT:
            utility.post_processing(self.filename)
            if self.out_file: self.out_file.close()