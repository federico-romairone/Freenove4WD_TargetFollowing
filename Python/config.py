# reference
MIN_REF = 0
MAX_REF = 50 # cm
DEF_REF = 10 # cm

# tuning
Kp =  2

# speed-duty convertion
MAX_SPEED = 50 								# cm/s
MAX_DUTY = 3500				 				# actual max is 4095
SPEED_TO_DUTY_RATIO = MAX_DUTY/MAX_SPEED 	# convertion ratio

__all__ = ["MIN_REF", "MAX_REF", "DEF_REF", "Kp", "SPEED_TO_DUTY_RATIO"]