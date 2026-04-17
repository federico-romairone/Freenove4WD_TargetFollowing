# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

# tuning
Kp = -2.7

# upper limit parameters
MAX_SPEED = 66.14
MAX_PWM = 4095

# error for oscillations
EPS = 0.5

# time in seconds for each cycle in control loop based on simulation data (file "20260331_123755_Distance_Speed_PWM_out_postprocessed.xlsx")
SAMPLING_PERIOD = 0.030;

# speed-duty coefficients convertion function based on cubic interpolation of measurements + origin
direct_conv_fun_coeff = [0.0172, -0.904, 47.148, 1.2836]
# duty-speed coefficients convertion function based on cubic interpolation of measurements + origin
inverse_conv_fun_coeff = [6e-10, -7e-6, 0.0352, -0.5423]

# convertion dead zone (abs(PWM) < 715 --> speed = 0)
DEAD_ZONE = 1

# control zones: can't use only a proportional controller due to uncapability to move at very low PWM input
CONTROLLER_DEAD_BAND = 1.0          # if greated then abs(error), do not move
BURST_ZONE = 4.0                    # under this threshold, controller goes in burst mode
BURST_PWM = 1000                    # minimum effective PWM
BURST_DURATION = SAMPLING_PERIOD    # pulse duration (from tuning)
BURST_PAUSE = 0.01                  # pause between pulses (from tuning)

# motor indexing
LU = 0
LL = 1
RU = 2
RL = 3

# saturation
SATURATE = True

# debugger
DEBUG = True

# data writer
FOUT_NAME = "Distance_Speed_PWM_out"
FOUT_EXT = "csv"
WRITE_OUT = True

# calibration: adjust based on (!!)MY(!!) machine model
CALIBRATE = False
MOTOR_CALIBRATION_FACTORS = [1.0]*4
MOTOR_CALIBRATION_FACTORS[LU] = 1.0
MOTOR_CALIBRATION_FACTORS[LL] = 1.0
MOTOR_CALIBRATION_FACTORS[RU] = 1.0
MOTOR_CALIBRATION_FACTORS[RL] = 1.0

__all__ = [
    "MIN_REF", "MAX_REF", "DEF_REF", 
    "Kp", "MAX_SPEED", "MAX_PWM", "EPS",
    "direct_conv_fun_coeff", "inverse_conv_fun_coeff",
    "DEAD_ZONE", "CONTROLLER_DEAD_BAND", 
    "BURST_ZONE", "BURST_PWM", "BURST_DURATION", "BURST_PAUSE",
    "LU", "LL", "RU", "RL",
    "SATURATE", "DEBUG", "FOUT_NAME", "FOUT_EXT", "WRITE_OUT",
    "CALIBRATE", "MOTOR_CALIBRATION_FACTORS",
    "SAMPLING_PERIOD"
]