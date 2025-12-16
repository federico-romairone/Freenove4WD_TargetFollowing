# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

# tuning
Kp = 1.8

# speed-duty convertion
MAX_2W = 66.140							        # cm/s, from measurements with 2 wheels working
#MAX_4W = 						    	        # cm/s, from measurements with 4 wheels working
MAX_SPEED = MAX_2W                              # in other parts of the code, I'll use only this max speed variable
MAX_DUTY = 4095				 				    # PWM, from manual
EPS = 1.5 									    # cm, useful to avoid oscillations
SPEED_TO_DUTY_RATIO = MAX_DUTY/MAX_SPEED     	# convertion ratio

# motor indexing
LU = 0
LL = 1
RU = 2
RL = 3

# calibration: adjust based on (!!)MY(!!) machine model
CALIBRATE = False
MOTOR_CALIBRATION_FACTORS = [1.0]*4
MOTOR_CALIBRATION_FACTORS[LU] = 1.0
MOTOR_CALIBRATION_FACTORS[LL] = 1.0
MOTOR_CALIBRATION_FACTORS[RU] = 1.0
MOTOR_CALIBRATION_FACTORS[RL] = 1.0

__all__ = [
    "MIN_REF", "MAX_REF", "DEF_REF", 
    "Kp", "MAX_SPEED", "MAX_DUTY", "EPS",
    "SPEED_TO_DUTY_RATIO", 
    "LU", "LL", "RU", "RL",
    "CALIBRATE", "MOTOR_CALIBRATION_FACTORS"
]
