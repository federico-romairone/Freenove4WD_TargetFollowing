# Freenove4WD_TargetFollowing

Development of a closed loop control system in order to make the Freenove 4WD smart car be able to follow a target by means of Matlab (simulation) and Python (implementation).

# DIRECTORY ORGANIZATION

Matlab/
    ...

Python/
    config.py           # Constants
    main.py             # Application    
    models/             # Class definitions
        __init__.py             
        ordinary_car.py # Car model with sensors
        pca9685.py      # Wheels motors
        servo.py        # Servo motors
        ultrasonic.py   # Ultrasonic sensor
    utility.py          # Utility functions
