from .pca9685 import PCA9685
from .servo import Servo
from .ultrasonic import Ultrasonic
from .. import config
from typing import List

class Ordinary_Car:
    # Constructor
    def __init__(self):
        # setting wheels motors
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
        # setting frontal position of the sensor
        self.servo = Servo();
        self.servo.set_servo_pwm('0', 90)
        self.servo.set_servo_pwm('1', 90)
        # setting ultrasonic sensor
        self.sensor = Ultrasonic()
    
    # Saturation handling
    def duty_range(self, duties: List[int]):
        if len(duties) != 4:
            raise ValueError("PWM values must be exacly four: one for each wheel.");
        for i in range(4):
            if duties[i] > config.MAX_DUTY:
                duties[i] = config.MAX_DUTY
            elif duties[i] < -config.MAX_DUTY:
                duties[i] = -config.MAX_DUTY
    
    # L-U wheel engine
    def left_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(0,0)
            self.pwm.set_motor_pwm(1,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(1,0)
            self.pwm.set_motor_pwm(0,abs(duty))
        else:
            self.pwm.set_motor_pwm(0,4095)
            self.pwm.set_motor_pwm(1,4095)
    
    # L-L wheel engine
    def left_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(3,0)
            self.pwm.set_motor_pwm(2,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(2,0)
            self.pwm.set_motor_pwm(3,abs(duty))
        else:
            self.pwm.set_motor_pwm(2,4095)
            self.pwm.set_motor_pwm(3,4095)
    
    # R-U wheel engine
    def right_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(6,0)
            self.pwm.set_motor_pwm(7,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(7,0)
            self.pwm.set_motor_pwm(6,abs(duty))
        else:
            self.pwm.set_motor_pwm(6,4095)
            self.pwm.set_motor_pwm(7,4095)
    
    # R-L wheel engine
    def right_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(4,0)
            self.pwm.set_motor_pwm(5,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(5,0)
            self.pwm.set_motor_pwm(4,abs(duty))
        else:
            self.pwm.set_motor_pwm(4,4095)
            self.pwm.set_motor_pwm(5,4095)

    # Engine settings for wheels motors
    def set_motor_model(self, duties: List[int]):
        if len(duties) != 4:
            raise ValueError("PWM values must be exacly four: one for each wheel.");
        self.duty_range(duties)
        self.left_upper_wheel(duties[config.LU])
        self.left_lower_wheel(duties[config.LL])
        self.right_upper_wheel(duties[config.RU])
        self.right_lower_wheel(duties[config.RL])

    def close(self):
        self.set_motor_model([0]*4)
        self.pwm.close()

