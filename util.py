def sign(value : float) -> int:
    """Returns the sign of a given value.

    Args:
        value (float): The input value.

    Returns:
        int: 1 if the value is positive, -1 if negative, 0 if zero.
    """
    if value > 0:
        return 1
    elif value < 0:
        return -1
    return 0


def clamp(n: float, v_min: float, v_max: float) -> float:
    """Clamp a value between a minimum and maximum.

    Args:
        n (float): The value to clamp.
        v_min (float): The minimum value.
        v_max (float): The maximum value.

    Returns:
        float: The clamped value.
    """
    return min(v_max, max(v_min, n))
