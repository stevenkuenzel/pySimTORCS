"""
Numba-optimized geometry functions for track and collision detection.
"""

import math
import numpy as np
from numba import njit


@njit
def adjacent_point_on_segment(
    px: float, py: float,
    seg_start_x: float, seg_start_y: float,
    seg_end_x: float, seg_end_y: float,
) -> tuple:
    """
    Find the closest point on a line segment to a given point.
    
    Args:
        px, py: Point coordinates
        seg_start_x, seg_start_y: Segment start point
        seg_end_x, seg_end_y: Segment end point
    
    Returns:
        Tuple of (closest_x, closest_y)
    """
    a = px - seg_start_x
    b = py - seg_start_y
    c = seg_end_x - seg_start_x
    d = seg_end_y - seg_start_y
    
    dot = a * c + b * d
    length_squared = c * c + d * d
    
    if length_squared > 0.0:
        param = dot / length_squared
    else:
        param = -1.0
    
    if param < 0.0:
        return seg_start_x, seg_start_y
    elif param > 1.0:
        return seg_end_x, seg_end_y
    else:
        return seg_start_x + param * c, seg_start_y + param * d


@njit
def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate Euclidean distance between two points."""
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


@njit
def distance_squared(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate squared distance between two points (faster when comparison is sufficient)."""
    dx = x2 - x1
    dy = y2 - y1
    return dx * dx + dy * dy


@njit
def sign_of_cross_product(
    ax: float, ay: float,
    bx: float, by: float,
    cx: float, cy: float,
) -> float:
    """
    Calculate the sign of cross product (b-a) × (c-a).
    Positive: c is left of line from a to b
    Negative: c is right of line from a to b
    Zero: c is on line from a to b
    """
    return (by - ay) * (cx - ax) - (bx - ax) * (cy - ay)
