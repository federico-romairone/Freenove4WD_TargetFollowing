# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

# tuning
Kp = -2.7

# upper limit parameter
MAX_PWM = 4095

# error for oscillations
EPS = 0.5

# time in seconds for each cycle in control loop based on simulation data (file "20260331_123755_Distance_Speed_PWM_out_postprocessed.xlsx")
SAMPLING_PERIOD = 0.030;

# speed-duty coefficients convertion function based on cubic interpolation of measurements + origin
direct_conv_fun_coeff = [0.0077, -1e-14, 27.628, 9e-12]
# duty-speed coefficients convertion function based on cubic interpolation of measurements + origin
inverse_conv_fun_coeff = [-6e-10, 2e-19, 0.0257, 3e-12]

# convertion dead zone (abs(PWM) < 715 --> speed = 0)
DEAD_ZONE_WIDTH = 715

# control zones: can't use only a proportional controller due to uncapability to move at very low PWM input
CONTROLLER_DEAD_BAND = 1.0          # if greated then abs(error), do not move
BURST_ZONE = 4.0                    # under this threshold, controller goes in burst mode
BURST_PWM = 1000                    # pulse intensity
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
    "Kp", "MAX_PWM", "EPS",
    "direct_conv_fun_coeff", "inverse_conv_fun_coeff",
    "DEAD_ZONE_WIDTH", "CONTROLLER_DEAD_BAND", 
    "BURST_ZONE", "BURST_PWM", "BURST_DURATION", "BURST_PAUSE",
    "LU", "LL", "RU", "RL",
    "SATURATE", "DEBUG", "FOUT_NAME", "FOUT_EXT", "WRITE_OUT",
    "CALIBRATE", "MOTOR_CALIBRATION_FACTORS",
    "SAMPLING_PERIOD"
]