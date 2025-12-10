def is_numeric(value) -> bool:
    """
    Checks whether 'value' is a numeric input.
    """
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False

def saturation(value: float, upper: float, lower: float = 0) -> float:
    """
    Saturate a value in a specific range [lower, upper].
    """
    saturated = value
    if saturated < lower: saturated = lower
    elif saturated > upper: saturated = upper
    return saturated
    
def suppress_oscillations(prev: float, succ: float, eps: float) -> float:
    """
    If the current value is quite the same as the previous one, this function make it exactly as the previous one.
    """
    if abs(succ-prev) < eps: succ = prev
    return succ

__all__ = ["is_numeric", "saturation"]