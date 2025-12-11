# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

# tuning
Kp = 1.2

# speed-duty convertion
MAX_SPEED = 59.35							# cm/s, from measurements
MAX_DUTY = 4095				 				# PWM, from manual
EPS = 1.5 									# cm, useful to avoid oscillations

SPEED_TO_DUTY_RATIO = MAX_DUTY/MAX_SPEED 	# convertion ratio

__all__ = [
    "MIN_REF", "MAX_REF", "DEF_REF", 
    "Kp", "MAX_SPEED", "MAX_DUTY", "EPS",
    "SPEED_TO_DUTY_RATIO"
]
