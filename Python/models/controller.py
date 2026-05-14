from models.ordinary_car import Ordinary_Car
import config
import utility
import time
import csv

class Controller:
    # Constructor
    def __init__(self, ref: float = config.DEF_REF, react: int = config.REACTIVITY):
        self.car = Ordinary_Car()
        self.ref = ref
        self.react = react
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
        if config.DEBUG:
            print(f"\nReference distance: {self.ref} cm")
            print(f"Controller reactivity: {self.react}")
            print(f"Sampling period: {config.SAMPLING_PERIOD*1000:.0f} ms\n")
            
        start_time = time.time()
        last_elapsed = 0

        # for speed and error:
        # element in position i is the one retarded of i samples
        error = [0.0]*3
        speed = [0.0]*3
        
        duties = [0]*4
        a = config.a[self.react - 1];
        b = config.b[self.react - 1];
        c = config.c[self.react - 1];
        d = config.d[self.react - 1];
        e = config.e[self.react - 1];

        while True:
            
            # SAMPLING

            dist = self.car.sensor.get_distance()
            # dist = utility.suppress_oscillations(old_dist, dist, config.EPS)
            error[0] = self.ref - dist

            # CONTROL LOGIC (from loop-shaping)

            # controller            
            speed[0] = a * error[0] + b * error[1] + c * error[2] + d * speed[1] + e * speed[2]
            if config.SATURATE:
                speed[0] = int(utility.saturation(speed[0], config.MAX_SPEED))
            # For sign explanation, view Simulink model
            input_speed = -speed[0]

            # plant
            duty = utility.force_dead_zone(in_val=utility.speed_to_PWM(input_speed))
            if config.SATURATE:
                duty = int(utility.saturation(duty, config.MAX_PWM))
            duties = [duty]*4
            if config.CALIBRATE:
                utility.apply_calibration(duties)

            # DEBUG AND OUTPUT

            real_elapsed_time = time.time() - start_time
            delta = real_elapsed_time - last_elapsed
            while delta < config.SAMPLING_PERIOD:
                delta = (time.time() - start_time) - last_elapsed
            elapsed_time = last_elapsed + delta
            if config.DEBUG:
                #print(f"t={utility.get_time_format(elapsed_time)} dist={dist:5.2f} err={error:+.2f} duty={duties[0]:5d}")
                print(f"{input_speed:.2f} = -1 * ({a} * {error[0]:.2f} + {b} * {error[1]:.2f} + {c} * {error[2]:.2f} + {d} * {speed[1]:.2f} + {e} * {speed[2]:.2f})")
            # write data on csv output file
            if config.WRITE_OUT and self.writer and self.out_file:
                self.writer.writerow([real_elapsed_time, elapsed_time, dist, input_speed, duties[0]])
                self.out_file.flush()
            
            self.car.set_motor_model(duties)
            last_elapsed = elapsed_time
            error[2] = error[1]
            error[1] = error[0]
            speed[2] = speed[1]
            speed[1] = speed[0]
        
    def test(self):
        duties = [1500]*4
        if config.CALIBRATE:
            utility.apply_calibration(duties)
            print("Calibration applied!")
        self.car.set_motor_model(duties)
        time.sleep(10)
        self.car.set_motor_model([0]*4)

    # Distructor
    def close(self):
        self.car.close()
        if config.WRITE_OUT:
            utility.post_processing(self.filename)
            if self.out_file: self.out_file.close()
