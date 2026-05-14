# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

# upper limit parameter
MAX_PWM = 4095
# MAX_SPEED = 66.14   # 2 wheels
MAX_SPEED = 80.68   # 4 wheels

# error for oscillations
EPS = 0.5

# time in seconds for each cycle in control loop based on simulation data (file "20260331_123755_Distance_Speed_PWM_out_postprocessed.xlsx")
SAMPLING_PERIOD = 0.030;

# speed-duty coefficients convertion function based on cubic interpolation of measurements + origin
direct_conv_fun_coeff = [0.0043, -5e-14, 20.347, 1e-10]
# duty-speed coefficients convertion function based on cubic interpolation of measurements + origin
inverse_conv_fun_coeff = [-8e-10, 8e-20, 0.0325, 1e-12]

# convertion dead zone (abs(PWM) < DEAD_ZONE_WIDTH --> speed = 0)
DEAD_ZONE_WIDTH = 625

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
MAX_REACTIVITY = 1
TUNING = False
REACTIVITY = 1;
# u(k) = a * e(k) + b * e(k-1) + c * e(k-2) + d * u(k-1) + e * u(k-2)
a = [9.779]
b = [-17.78] 
c = [8.03]
d = [1.847]
e = [-0.8475]

__all__ = [
    "MIN_REF", "MAX_REF", "DEF_REF", "MAX_PWM", "MAX_SPEED", 
    "EPS", "direct_conv_fun_coeff", "inverse_conv_fun_coeff",
    "DEAD_ZONE_WIDTH", "LU", "LL", "RU", "RL", "SATURATE", 
    "DEBUG", "FOUT_NAME", "FOUT_EXT", "WRITE_OUT", "CALIBRATE", 
    "MOTOR_CALIBRATION_FACTORS", "SAMPLING_PERIOD", 
    "MIN_REACTIVITY", "MAX_REACTIVITY", "TUNING", "REACTIVITY", 
    "a", "b", "c", "d", "e"
]