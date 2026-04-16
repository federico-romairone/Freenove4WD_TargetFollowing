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
        start_time = time.time()
        last_elapsed = 0
        bypass_suppression = False

        # FSM for burst zone
        IDLE = 0                # no burst is occurring, controller ready for evaluation
        BURST_ON = 1            # pulse is active, motors are receiving a low input 
        BURST_OFF = 2           # pulse is ended, motors are stopped, new evaluation in the next cycle
        burst_state = IDLE
        burst_timer = 0.0

        old_dist = self.car.sensor.get_distance()
        duties = [0]*4

        while True:

            loop_start_time = time.time()
            
            ## SAMPLINGg

            dist = self.car.sensor.get_distance()
            dist = utility.suppress_oscillations(old_dist, dist, config.EPS, bypass_suppression)
            bypass_suppression = False
            error = self.ref - dist

            ## CONTROL LOGIC

            # DEAD ZONE -> not move

            if abs(error) < config.CONTROLLER_DEAD_BAND:
                burst_state = IDLE
                duties = [0] * 4

            # BURST ZONE -> pulse move

            elif abs(error) < config.BURST_ZONE:

                # current state evaluation
                
                if burst_state == IDLE:
                    # start pulse
                    sign = 1 if error * self.Kp > 0 else -1  # evaluation of e(t)*Kp for corrispondance with the proportional control logic
                    duty = sign * config.BURST_PWM
                    duties = [duty]*4
                    if config.CALIBRATE:
                        utility.apply_calibration(duties)
                    burst_state = BURST_ON
                    burst_timer = loop_start_time
                
                elif burst_state == BURST_ON:
                    # pulse ended -> stop and start pause
                    if (time.time()-burst_timer) >= config.BURST_DURATION:
                        duties = [0]*4
                        burst_state = BURST_OFF
                        burst_timer = loop_start_time
                        bypass_suppression = True
                
                elif burst_state == BURST_OFF:
                    # pulse was ended in the previous cycle -> stop and wait for evaluation
                    duties = [0]*4
                    if (loop_start_time-burst_timer) >= config.BURST_PAUSE:
                        burst_state = IDLE
            
            # PROPORTIONAL ZONE

            else:

                # evaluation state
                burst_state = IDLE                
                # prop control loop logic
                speed = error * self.Kp
                if config.SATURATE:
                    speed = utility.saturation(speed, config.MAX_SPEED)
                duty = utility.speed_to_PWM(speed)
                duty = utility.dead_zone_corrector(duty)
                if config.SATURATE:
                    duty = int(utility.saturation(duty, config.MAX_PWM))
                duties = [duty]*4
                if config.CALIBRATE:
                    utility.apply_calibration(duties)

            ## DEBUG AND OUTPUT

            real_elapsed_time = time.time() - start_time
            delta = real_elapsed_time - last_elapsed
            while delta < config.SAMPLING_PERIOD:
                delta = (time.time() - start_time) - last_elapsed
            elapsed_time = last_elapsed + delta
            if config.DEBUG:
                if abs(error) < config.CONTROLLER_DEAD_BAND:
                    zone = "DEAD"
                    state_str = ""
                elif abs(error) < config.BURST_ZONE:
                    zone = "BURST state="
                    state_str = ["IDLE", "ON", "OFF"][burst_state]
                else:
                    zone = "PROPORTIONAL"
                    state_str = ""
                print(f"[{zone}{state_str}] t={utility.get_time_format(elapsed_time)} dist={dist:5.2f} err={error:+.2f} duty={duties[0]:5d}")
            # write data on csv output file
            if config.WRITE_OUT and self.writer and self.out_file:
                self.writer.writerow([real_elapsed_time, elapsed_time, dist, error * self.Kp, duties[0]])
                self.out_file.flush()
            
            self.car.set_motor_model(duties)
            last_elapsed = elapsed_time
            old_dist = dist
        
    def test(self):
        duties = [720]*4
        if config.CALIBRATE:
            utility.apply_calibration(duties)
            print("Calibration applied!")
        self.car.set_motor_model(duties)
        time.sleep(7)
        self.car.set_motor_model([0]*4)

    # Distructor
    def close(self):
        self.car.close()
        if config.WRITE_OUT:
            utility.post_processing(self.filename)
            if self.out_file: self.out_file.close()
