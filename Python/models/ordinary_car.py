from .pca9685 import PCA9685
from .servo import Servo
from .ultrasonic import Ultrasonic
from .. import config

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
    def duty_range(self, duty1, duty2, duty3, duty4):
        if duty1 > config.MAX_DUTY:
            duty1 = config.MAX_DUTY
        elif duty1 < -config.MAX_DUTY:
            duty1 = -config.MAX_DUTY        
        if duty2 > config.MAX_DUTY:
            duty2 = config.MAX_DUTY
        elif duty2 < -config.MAX_DUTY:
            duty2 = -config.MAX_DUTY  
        if duty3 > config.MAX_DUTY:
            duty3 = config.MAX_DUTY
        elif duty3 < -config.MAX_DUTY:
            duty3 = -config.MAX_DUTY
        if duty4 > config.MAX_DUTY:
            duty4 = config.MAX_DUTY
        elif duty4 < -config.MAX_DUTY:
            duty4 = -config.MAX_DUTY
        return duty1,duty2,duty3,duty4
    
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
    def set_motor_model(self, duty1, duty2, duty3, duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)

    def close(self):
        self.set_motor_model(0,0,0,0)
        self.pwm.close()
