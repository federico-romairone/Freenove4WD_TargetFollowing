# Freenove 4WD Target Following -- Python & MATLAB Project

This repository contains the software for controlling the **Freenove 4WD
Smart Car** by means of a **target-following system**, motor drivers,
servo control, ultrasonic sensing, and control logic.

The project includes:

-   **Control impelementation in Python**
-   **MATLAB scripts for system simulation**

--------------------------------------------------------------------------------------------

# Directory Structure

    Matlab/
        loop_shaping.m      # Computations for loop-shaping controller design
        model_sim.slx       # Simulink model for the closed-loop control system
        myngridst.m         # Plot of the forbidden zone in the Nichols plot (loop shaping)
        nyquist1.m          # Nyquist plot for a polynomial transfer function
        simulation_data.m   # Handler for model simulation and relative computations

    Measurements/
        SpeedToPwm.xlsx     # Transfer function study by means of interpolation
        videos_duration.sh  # Bash script useful for measurements

    Python/
        config.py           # Global constants and settings
        main.py             # Main application entry point
        models/             # Core robot components (class definitions)
            __init__.py         # Turns "models" into a package (easier imports)
            controller.py       # Controller class for target following
            ordinary_car.py     # Car model with integrated sensors
            pca9685.py          # Wheels motor driver (PWM)
            servo.py            # Servo motor control
            ultrasonic.py       # Ultrasonic distance sensor
        utility.py          # Helper functions
    
    Controller_project.pdf  # Complete loop-shaping design with theoretical additions

--------------------------------------------------------------------------------------------

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

--------------------------------------------------------------------------------------------

# Running the Application

-   Open the terminal.
-   Go to the path '../Freenove4WD_TargetFollowing/Python'
-   Run the following command:

    ``` bash
    cd Freenove4WD_TargetFollowing
    python -m Python.main
    ```

--------------------------------------------------------------------------------------------
