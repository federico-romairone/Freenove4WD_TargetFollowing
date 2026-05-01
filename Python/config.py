# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

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

# reactivity regulation for controller
MIN_REACTIVITY = 1
MAX_REACTIVITY = 12
TUNING = True
REACTIVITY = 1;
# u(k) = u(k-1) + a * e(k) - b * e(k)
a = [32.41, 38.89, 45.38, 51.86, 58.34, 64.82, 71.31, 77.79, 84.27, 90.75, 97.24, 103.7]
b = [32.22, 38.66, 45.11, 51.55, 57.99, 64.44, 70.88, 77.32, 83.77, 90.21, 96.66, 103.1] 

__all__ = [
    "MIN_REF", "MAX_REF", "DEF_REF", "MAX_SPEED", "MAX_PWM", "EPS",
    "direct_conv_fun_coeff", "inverse_conv_fun_coeff",
    "LU", "LL", "RU", "RL",
    "SATURATE", "DEBUG", "FOUT_NAME", "FOUT_EXT", "WRITE_OUT",
    "CALIBRATE", "MOTOR_CALIBRATION_FACTORS",
    "SAMPLING_PERIOD", "MIN_REACTIVITY", "MAX_REACTIVITY", "TUNING", 
    "REACTIVITY", "a", "b"
]