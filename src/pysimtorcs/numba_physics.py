"""
Numba-optimized physics calculations for car simulation.
These functions are JIT-compiled for maximum performance.
"""

import math
import numpy as np
from numba import njit, prange


# Physics constants
PHYSICS_MASS = 1150.0
PHYSICS_GRAVITY = 9.81
PHYSICS_INERTIA_SCALE = 1.0
PHYSICS_CG_TO_FRONT_AXLE = 1.22
PHYSICS_CG_TO_REAR_AXLE = 1.42
PHYSICS_CG_HEIGHT = 0.25
PHYSICS_TIRE_GRIP = 1.6
PHYSICS_WEIGHT_TRANSFER = 0.2
PHYSICS_CORNER_STIFFNESS_FRONT = 5.0
PHYSICS_CORNER_STIFFNESS_REAR = 5.2
PHYSICS_AIR_RESIST = 0.4032
PHYSICS_ROLL_RESIST = 12.096
PHYSICS_INERTIA = PHYSICS_MASS * PHYSICS_INERTIA_SCALE
PHYSICS_WHEEL_BASE = PHYSICS_CG_TO_FRONT_AXLE + PHYSICS_CG_TO_REAR_AXLE
PHYSICS_AXLE_WEIGHT_RATIO_FRONT = PHYSICS_CG_TO_REAR_AXLE / PHYSICS_WHEEL_BASE
PHYSICS_AXLE_WEIGHT_RATIO_REAR = PHYSICS_CG_TO_FRONT_AXLE / PHYSICS_WHEEL_BASE
PHYSICS_ENGINE_FORCE = 11900.0
PHYSICS_BRAKE_FORCE = 26300.0


@njit
def sign(x: float) -> float:
    """Return the sign of a number: -1, 0, or 1."""
    if x > 0:
        return 1.0
    elif x < 0:
        return -1.0
    else:
        return 0.0


@njit
def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value between min and max."""
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    else:
        return value


@njit
def calculate_local_velocity(
    heading: float,
    velocity_x: float,
    velocity_y: float,
) -> tuple:
    """
    Transform velocity from world coordinates to local car coordinates.

    Args:
        heading: Car heading in radians
        velocity_x: World velocity x component
        velocity_y: World velocity y component

    Returns:
        Tuple of (velocity_local_x, velocity_local_y)
    """
    cs = math.cos(heading)
    sn = math.sin(heading)

    local_x = cs * velocity_x + sn * velocity_y
    local_y = cs * velocity_y - sn * velocity_x

    return local_x, local_y


@njit
def calculate_absolute_velocity(velocity_x: float, velocity_y: float) -> float:
    """Calculate the magnitude of velocity."""
    return math.sqrt(velocity_x * velocity_x + velocity_y * velocity_y)


@njit
def update_physics_step(
    heading: float,
    velocity_x: float,
    velocity_y: float,
    yaw_rate: float,
    throttle: float,
    brake: float,
    steer_angle: float,
    dt: float,
) -> tuple:
    """
    Perform a single physics update step.

    Args:
        heading: Current heading in radians
        velocity_x: Current world velocity x
        velocity_y: Current world velocity y
        yaw_rate: Current yaw rate in rad/s
        throttle: Throttle input [0, 1]
        brake: Brake input [0, 1]
        steer_angle: Steering angle in radians
        dt: Time step in seconds

    Returns:
        Tuple of (new_heading, new_velocity_x, new_velocity_y, new_yaw_rate,
                  new_position_x, new_position_y, new_absolute_velocity)
    """
    # Pre-calculate trigonometric values
    cs = math.cos(heading)
    sn = math.sin(heading)

    # Transform velocity to local car coordinates
    velocity_local_x = cs * velocity_x + sn * velocity_y
    velocity_local_y = cs * velocity_y - sn * velocity_x

    # Calculate axle weights
    # This is a simplified calculation - we need acceleration_local_x for the full model
    # Using a placeholder value based on current acceleration estimate
    accel_est_x = (throttle - brake) * 5.0  # Rough estimate

    axle_weight_front = PHYSICS_MASS * (
        PHYSICS_AXLE_WEIGHT_RATIO_FRONT * PHYSICS_GRAVITY
        - PHYSICS_WEIGHT_TRANSFER * accel_est_x * PHYSICS_CG_HEIGHT / PHYSICS_WHEEL_BASE
    )
    axle_weight_rear = PHYSICS_MASS * (
        PHYSICS_AXLE_WEIGHT_RATIO_REAR * PHYSICS_GRAVITY
        + PHYSICS_WEIGHT_TRANSFER * accel_est_x * PHYSICS_CG_HEIGHT / PHYSICS_WHEEL_BASE
    )

    # Calculate yaw speeds
    yaw_speed_front = PHYSICS_CG_TO_FRONT_AXLE * yaw_rate
    yaw_speed_rear = -PHYSICS_CG_TO_REAR_AXLE * yaw_rate

    # Calculate slip angles
    vel_x_abs = abs(velocity_local_x)
    if vel_x_abs > 0.001:
        slip_angle_front = (
            math.atan2(velocity_local_y + yaw_speed_front, vel_x_abs)
            - sign(velocity_local_x) * steer_angle
        )
        slip_angle_rear = math.atan2(velocity_local_y + yaw_speed_rear, vel_x_abs)
    else:
        slip_angle_front = 0.0
        slip_angle_rear = 0.0

    # Calculate lateral friction forces
    friction_force_front_cy = (
        clamp(
            -PHYSICS_CORNER_STIFFNESS_FRONT * slip_angle_front,
            -PHYSICS_TIRE_GRIP,
            PHYSICS_TIRE_GRIP,
        )
        * axle_weight_front
    )
    friction_force_rear_cy = (
        clamp(
            -PHYSICS_CORNER_STIFFNESS_REAR * slip_angle_rear,
            -PHYSICS_TIRE_GRIP,
            PHYSICS_TIRE_GRIP,
        )
        * axle_weight_rear
    )

    # Calculate forces
    brake_force = brake * PHYSICS_BRAKE_FORCE
    throttle_force = throttle * PHYSICS_ENGINE_FORCE

    # Traction forces
    traction_force_cx = throttle_force - brake_force * sign(velocity_local_x)
    traction_force_cy = 0.0

    # Drag and rolling resistance
    drag_force_cx = (
        -PHYSICS_ROLL_RESIST * velocity_local_x
        - PHYSICS_AIR_RESIST * velocity_local_x * abs(velocity_local_x)
    )
    drag_force_cy = (
        -PHYSICS_ROLL_RESIST * velocity_local_y
        - PHYSICS_AIR_RESIST * velocity_local_y * abs(velocity_local_y)
    )

    # Total forces in local coordinates
    total_force_cx = drag_force_cx + traction_force_cx
    total_force_cy = (
        drag_force_cy
        + traction_force_cy
        + math.cos(steer_angle) * friction_force_front_cy
        + friction_force_rear_cy
    )

    # Acceleration in local coordinates
    accel_local_x = total_force_cx / PHYSICS_MASS
    accel_local_y = total_force_cy / PHYSICS_MASS

    # Transform acceleration back to world coordinates
    accel_x = cs * accel_local_x - sn * accel_local_y
    accel_y = sn * accel_local_x + cs * accel_local_y

    # Update velocity
    new_velocity_x = velocity_x + accel_x * dt
    new_velocity_y = velocity_y + accel_y * dt

    # Calculate absolute velocity
    new_absolute_velocity = calculate_absolute_velocity(new_velocity_x, new_velocity_y)

    # Stop logic
    if abs(new_absolute_velocity) < 0.5 and throttle == 0.0:
        new_velocity_x = 0.0
        new_velocity_y = 0.0
        new_absolute_velocity = 0.0
        angular_torque = 0.0
    else:
        # Calculate angular torque
        angular_torque = (
            friction_force_front_cy + traction_force_cy
        ) * PHYSICS_CG_TO_FRONT_AXLE - friction_force_rear_cy * PHYSICS_CG_TO_REAR_AXLE

    # Update yaw
    angular_accel = angular_torque / PHYSICS_INERTIA
    new_yaw_rate = yaw_rate + angular_accel * dt
    new_heading = heading + new_yaw_rate * dt

    # Update position
    position_dx = new_velocity_x * dt
    position_dy = new_velocity_y * dt

    return (
        new_heading,
        new_velocity_x,
        new_velocity_y,
        new_yaw_rate,
        position_dx,
        position_dy,
        new_absolute_velocity,
    )


@njit
def update_sensor_angles(heading: float, sensor_angles: np.ndarray) -> np.ndarray:
    """
    Calculate sensor direction vectors in world coordinates.

    Args:
        heading: Car heading in radians
        sensor_angles: Array of sensor angles relative to car heading

    Returns:
        Array of direction vectors (x, y pairs)
    """
    adjusted_angles = heading + sensor_angles
    # Normalize angles to [-pi, pi]
    adjusted_angles = np.where(
        adjusted_angles > 0,
        (adjusted_angles + math.pi) % (2 * math.pi) - math.pi,
        (adjusted_angles + math.pi) % (2 * math.pi) - math.pi,
    )

    x = np.cos(adjusted_angles)
    y = np.sin(adjusted_angles)

    return np.column_stack((x, y))


@njit
def line_intersection(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    x3: float,
    y3: float,
    x4: float,
    y4: float,
) -> tuple:
    """
    Calculate intersection of two line segments.

    Args:
        x1, y1: Start point of first line segment
        x2, y2: End point of first line segment
        x3, y3: Start point of second line segment
        x4, y4: End point of second line segment

    Returns:
        Tuple of (intersection_x, intersection_y, found) where found is True if intersection exists
    """
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3

    denom = dx1 * dy2 - dy1 * dx2
    if abs(denom) < 1e-10:
        return 0.0, 0.0, False

    dx3 = x3 - x1
    dy3 = y3 - y1

    t = (dx3 * dy2 - dy3 * dx2) / denom
    u = (dx3 * dy1 - dy3 * dx1) / denom

    if 0 <= t <= 1 and 0 <= u <= 1:
        ix = x1 + t * dx1
        iy = y1 + t * dy1
        return ix, iy, True

    return 0.0, 0.0, False


@njit
def point_distance_squared(px: float, py: float, qx: float, qy: float) -> float:
    """Calculate squared distance between two points."""
    dx = px - qx
    dy = py - qy
    return dx * dx + dy * dy


@njit
def point_within_polygon(
    px: float, py: float, polygon_x: np.ndarray, polygon_y: np.ndarray
) -> bool:
    """
    Check if a point is within a polygon using ray casting.

    Args:
        px, py: Point coordinates
        polygon_x: Array of polygon vertex x coordinates
        polygon_y: Array of polygon vertex y coordinates

    Returns:
        True if point is within polygon
    """
    n = len(polygon_x)
    if n < 3:
        return False

    intersections = 0

    for i in range(n):
        p1x, p1y = polygon_x[i], polygon_y[i]
        p2x, p2y = polygon_x[(i + 1) % n], polygon_y[(i + 1) % n]

        # Check for horizontal edge
        if abs(p1y - p2y) < 1e-10 and abs(p1y - py) < 1e-10:
            if min(p1x, p2x) <= px <= max(p1x, p2x):
                return True

        # Check for point on vertex
        if abs(p1x - px) < 1e-10 and abs(p1y - py) < 1e-10:
            return True

        # Ray casting logic
        if (p1y <= py < p2y) or (p2y <= py < p1y):
            if abs(p2x - p1x) < 1e-10:
                x_intersection = p2x
            else:
                x_intersection = (p2x - p1x) * (py - p1y) / (p2y - p1y) + p1x

            if x_intersection > px:
                intersections += 1

    return intersections % 2 == 1


@njit
def find_line_intersection_distance(
    car_pos_x: float,
    car_pos_y: float,
    sensor_x: float,
    sensor_y: float,
    sensor_range: float,
    line_x1: float,
    line_y1: float,
    line_x2: float,
    line_y2: float,
) -> float:
    """
    Find intersection distance of a sensor ray with a line segment.

    Args:
        car_pos_x, car_pos_y: Car position
        sensor_x, sensor_y: Sensor direction (unit vector)
        sensor_range: Maximum sensor range
        line_x1, line_y1: Line segment start
        line_x2, line_y2: Line segment end

    Returns:
        Distance to intersection (sensor_range if no intersection)
    """
    # Sensor end point
    sensor_to_x = car_pos_x + sensor_x * sensor_range
    sensor_to_y = car_pos_y + sensor_y * sensor_range

    # Line intersection calculation
    dx1 = sensor_to_x - car_pos_x
    dy1 = sensor_to_y - car_pos_y
    dx2 = line_x2 - line_x1
    dy2 = line_y2 - line_y1

    denom = dx1 * dy2 - dy1 * dx2
    if abs(denom) < 1e-10:
        return sensor_range

    dx3 = line_x1 - car_pos_x
    dy3 = line_y1 - car_pos_y

    t = (dx3 * dy2 - dy3 * dx2) / denom
    u = (dx3 * dy1 - dy3 * dx1) / denom

    if 0 <= t <= 1 and 0 <= u <= 1:
        # Intersection found
        ix = car_pos_x + t * dx1
        iy = car_pos_y + t * dy1
        dist_sq = (ix - car_pos_x) ** 2 + (iy - car_pos_y) ** 2
        return math.sqrt(dist_sq)

    return sensor_range


@njit(parallel=True)
def update_multiple_cars_parallel(
    headings: np.ndarray,
    velocity_x: np.ndarray,
    velocity_y: np.ndarray,
    yaw_rates: np.ndarray,
    throttles: np.ndarray,
    brakes: np.ndarray,
    steer_angles: np.ndarray,
    dt: float,
) -> tuple:
    """
    Update physics for multiple cars in parallel.

    Args:
        headings: Array of car headings
        velocity_x: Array of car velocity x components
        velocity_y: Array of car velocity y components
        yaw_rates: Array of car yaw rates
        throttles: Array of throttle inputs
        brakes: Array of brake inputs
        steer_angles: Array of steering angles
        dt: Time step

    Returns:
        Tuple of arrays: (headings, velocity_x, velocity_y, yaw_rates,
                         position_dx, position_dy, absolute_velocities)
    """
    num_cars = len(headings)

    new_headings = np.empty(num_cars, dtype=np.float64)
    new_velocity_x = np.empty(num_cars, dtype=np.float64)
    new_velocity_y = np.empty(num_cars, dtype=np.float64)
    new_yaw_rates = np.empty(num_cars, dtype=np.float64)
    position_dx = np.empty(num_cars, dtype=np.float64)
    position_dy = np.empty(num_cars, dtype=np.float64)
    absolute_velocities = np.empty(num_cars, dtype=np.float64)

    for i in prange(num_cars):
        result = update_physics_step(
            headings[i],
            velocity_x[i],
            velocity_y[i],
            yaw_rates[i],
            throttles[i],
            brakes[i],
            steer_angles[i],
            dt,
        )

        new_headings[i] = result[0]
        new_velocity_x[i] = result[1]
        new_velocity_y[i] = result[2]
        new_yaw_rates[i] = result[3]
        position_dx[i] = result[4]
        position_dy[i] = result[5]
        absolute_velocities[i] = result[6]

    return (
        new_headings,
        new_velocity_x,
        new_velocity_y,
        new_yaw_rates,
        position_dx,
        position_dy,
        absolute_velocities,
    )
