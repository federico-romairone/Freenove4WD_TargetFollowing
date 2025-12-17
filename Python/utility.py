from typing import List
from . import config

def is_numeric(value) -> bool:
    """
    Checks whether 'value' is a numeric input.
    """
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False

def saturation(value: float, upper: float) -> float:
    """
    Saturate a value in a specific range [lower, upper].
    """
    saturated = value
    if saturated < -upper: saturated = -upper
    elif saturated > upper: saturated = upper
    return saturated
    
def suppress_oscillations(prev: float, succ: float, eps: float) -> float:
    """
    If the current value is quite the same as the previous one, this function make it exactly as the previous one.
    """
    if abs(succ-prev) < eps: succ = prev
    return succ

def apply_calibration(duties: List[int]) -> None:
    for i in range(4):
        duties[i] = int(duties[i]*config.MOTOR_CALIBRATION_FACTORS[i])

__all__ = ["is_numeric", "saturation", 
           "suppress_oscillations", 
           "apply_calibration"]