# Freenove 4WD Target Following -- Python & MATLAB Project

This repository contains the software for controlling the **Freenove 4WD
Smart Car** by means of a **target-following system**, motor drivers,
servo control, ultrasonic sensing, and control logic.

The project includes:

-   **Control impelementation in Python**
-   **MATLAB scripts for system simulation**

------------------------------------------------------------------------

# Directory Structure

    Matlab/
        ...                 # MATLAB analysis, scripts, and models

    Python/
        config.py           # Global constants and settings
        main.py             # Main application entry point
        models/             # Core robot components (class definitions)
            __init__.py
            ordinary_car.py # Car model with integrated sensors
            pca9685.py      # Wheels motor driver (PWM)
            servo.py        # Servo motor control
            ultrasonic.py   # Ultrasonic distance sensor
        utility.py          # Helper functions

------------------------------------------------------------------------

## Requirements

-   Python 3.10+
-   Raspberry Pi GPIO dependencies:

    ``` bash
    sudo apt install python3-rpi.gpio python3-pigpio python3-smbus
    ```
-   PCA9685 library:

    ``` bash
    pip install adafruit-circuitpython-pca9685
    ```

------------------------------------------------------------------------

# Running the Application

⚠️ Do **not** run `main.py` directly due to relative imports.

## Correct command:

``` bash
cd Freenove4WD_TargetFollowing
python -m Python.main
```

This ensures Python recognizes the `Python/` folder as a package.

------------------------------------------------------------------------
