import math

from pygame import Vector2

import pysimtorcs.settings as settings
from pysimtorcs.controller import CarController
from pysimtorcs.segments import Segment, Turn
from pysimtorcs.sensor import SensorInformation
from pysimtorcs.util import clamp, sign
from pysimtorcs import numba_physics
import numpy as np

RANGE_TRACK_EDGE_SENSOR_LEFT = -45
RANGE_TRACK_EDGE_SENSOR_RIGHT = 45
ANGLE_BETWEEN_TRACK_EDGE_SENSORS = 5


def define_sensor_angles(from_angle: int, to_angle: int, step_size: int) -> np.ndarray:
    angles = [
        (-x / 180.0) * math.pi for x in range(from_angle, to_angle + 1, step_size)
    ]
    return np.array(angles, dtype=float)


# Maximum steering angle in radians
STEER_MAX = 0.366519

# PHYSICS RELATED CONSTANTS

# Car configuration. Based on the properties of car1-trb1 in TORCS.
PHYSICS_GRAVITY = 9.81  # m/s^2
PHYSICS_MASS = 1150.0  # kg
PHYSICS_INERTIA_SCALE = 1.0  # Multiply by mass for inertia
PHYSICS_CG_TO_FRONT_AXLE = 1.22  # Centre gravity to front axle
PHYSICS_CG_TO_REAR_AXLE = 1.42  # Centre gravity to rear axle
PHYSICS_CG_HEIGHT = 0.25  # Centre gravity height
PHYSICS_TIRE_GRIP = 1.6  # How much grip tires have. Was originally 2.0
PHYSICS_WEIGHT_TRANSFER = (
    0.2  # How much weight is transferred during acceleration/braking
)
PHYSICS_CORNER_STIFFNESS_FRONT = 5.0
PHYSICS_CORNER_STIFFNESS_REAR = 5.2
PHYSICS_AIR_RESIST = 0.4032  # 2.5  # air resistance (* vel)
PHYSICS_ROLL_RESIST = 12.096  # 8.0  # rolling resistance force (* vel)
PHYSICS_INERTIA = PHYSICS_MASS * PHYSICS_INERTIA_SCALE
PHYSICS_WHEEL_BASE = PHYSICS_CG_TO_FRONT_AXLE + PHYSICS_CG_TO_REAR_AXLE
PHYSICS_AXLE_WEIGHT_RATIO_FRONT = (
    PHYSICS_CG_TO_REAR_AXLE / PHYSICS_WHEEL_BASE
)  # % car weight on the front axle
PHYSICS_AXLE_WEIGHT_RATIO_REAR = (
    PHYSICS_CG_TO_FRONT_AXLE / PHYSICS_WHEEL_BASE
)  # % car weight on the rear axle

# Constants determined according to the description in the dissertation.
PHYSICS_ENGINE_FORCE = 11900.0
PHYSICS_BRAKE_FORCE = 26300.0


class Car:
    def __init__(
        self,
        id: int,
        controller: CarController,
        heading: float,
        position: Vector2,
    ):
        self.id: int = id
        self.controller: CarController = controller

        self.heading: float = heading
        self.position: Vector2 = position

        self.total_distance_from_track = 0.0
        self.total_speed = 0.0

        self.disqualified = False

        self.sensors = None
        self.sensor_angles: np.ndarray = define_sensor_angles(
            RANGE_TRACK_EDGE_SENSOR_LEFT,
            RANGE_TRACK_EDGE_SENSOR_RIGHT,
            ANGLE_BETWEEN_TRACK_EDGE_SENSORS,
        )
        self.sensor_information = SensorInformation(len(self.sensor_angles))

        self.current_segment: Segment = None
        self.last_valid_segment: Segment = None
        self.previous_segment: Segment = None

        # Input / Control.
        self.throttle = 0.0
        self.brake = 0.0
        self.steer_angle = 0.0

        # The real distance moved. Useful for fitness calculation.
        self.distance_moved = 0.0

        # Physics.
        self.velocity = Vector2()
        self.velocity_local = Vector2()
        self.acceleration = Vector2()
        self.acceleration_local = Vector2()
        self.absolute_velocity = 0.0
        self.previous_absolute_velocity = 0.0  # For Fitness.
        self.yaw_rate = 0.0

        # ABS flag.
        self.last_brake_loosened = True

    def update(self, dt: float, track_length: float):
        """Update controller input, fitness info, and physics."""
        input = self.controller.control(self.sensor_information)
        target_steer = input.left - input.right

        self.update_fitness_related_information(dt, target_steer, track_length)

        self.throttle = input.throttle
        self.brake = self.filter_abs(input.brake)
        self.steer_angle = target_steer * STEER_MAX

        self.update_physics(dt)

    def update_fitness_related_information(
        self, dt: float, target_steer: float, track_length: float
    ):
        # Fitness: Turn speed score.
        if self.current_segment is not None:
            car_is_in_turn_or_approaching: bool = False

            if self.current_segment.in_turn is not None:
                # The car is currently in a turn.
                car_is_in_turn_or_approaching = True
                possible_speed = self.current_segment.in_turn.speed_max
            else:
                next_turn: Turn = self.current_segment.next_turn

                if next_turn.speed_max >= settings.SPEED_MAX:
                    # The car is on a straight or the next turn is close to straight.
                    car_is_in_turn_or_approaching = False
                    possible_speed = settings.SPEED_MAX

                else:
                    if next_turn.segments[0].id < self.current_segment.id:
                        distance = (
                            track_length
                            - (
                                self.current_segment.length_track_total
                                + self.sensor_information.segment_position
                            )
                            + next_turn.segments[0].length_track_total
                        )
                    else:
                        distance = next_turn.segments[0].length_track_total - (
                            self.current_segment.length_track_total
                            + self.sensor_information.segment_position
                        )
                    distance = min(settings.TURN_SPEED_DISTANCE_MAX, distance)

                    car_is_in_turn_or_approaching = (
                        distance < settings.TURN_SPEED_DISTANCE_MAX
                    )

                    # Reduce the approx. max. speed linearly in relation to the distance to the turn.
                    possible_speed = next_turn.max_speed + (
                        settings.SPEED_MAX - next_turn.max_speed
                    ) * (distance / settings.TURN_SPEED_DISTANCE_MAX)

            possible_speed *= settings.TURN_SPEED_MULTIPLIER

            if car_is_in_turn_or_approaching:
                # Update the fitness value. Note: A speed higher than possible_speed contributes positively to the fitness value.
                self.sensor_information.too_low_turn_speed += (
                    possible_speed - self.absolute_velocity
                )
                self.sensor_information.ticks_in_or_before_turns += 1

        self.sensor_information.total_steering += abs(self.steer_angle) * dt

        if abs(target_steer) <= settings.THRESHOLD_DRIVING_STRAIGHT:
            self.sensor_information.length_driven_straight += (
                self.absolute_velocity * dt
            )

    def update_sensor_target_vectors(self) -> list[Vector2]:
        # Vectorized version using pygame.Vector2
        angles = self.heading + self.sensor_angles
        angles = (angles + math.pi) % (2 * math.pi) - math.pi

        # Use numpy for fast sin/cos, then create Vector2 in bulk
        x = np.cos(angles)
        y = np.sin(angles)

        # pygame.Vector2 does not support bulk creation, but we can use list comprehension efficiently
        return [Vector2(xi, yi) for xi, yi in zip(x, y)]

    def update_physics(self, dt: float):
        """Update car physics using Numba-compiled physics engine."""
        # Call the compiled physics function
        (
            new_heading,
            new_velocity_x,
            new_velocity_y,
            new_yaw_rate,
            position_dx,
            position_dy,
            new_absolute_velocity,
        ) = numba_physics.update_physics_step(
            self.heading,
            self.velocity.x,
            self.velocity.y,
            self.yaw_rate,
            self.throttle,
            self.brake,
            self.steer_angle,
            dt,
        )

        # Update state variables
        self.heading = new_heading
        self.velocity.x = new_velocity_x
        self.velocity.y = new_velocity_y
        self.yaw_rate = new_yaw_rate
        self.previous_absolute_velocity = self.absolute_velocity
        self.absolute_velocity = new_absolute_velocity

        # Update position
        self.position.x += position_dx
        self.position.y += position_dy

        # Update local velocity for sensor readings
        cs = math.cos(self.heading)
        sn = math.sin(self.heading)
        self.velocity_local.x = cs * self.velocity.x + sn * self.velocity.y
        self.velocity_local.y = cs * self.velocity.y - sn * self.velocity.x

        # Update acceleration for fitness calculation (simplified estimate)
        self.acceleration_local.x = (
            (new_velocity_x - self.velocity.x) / dt if dt > 0 else 0.0
        )
        self.acceleration_local.y = (
            (new_velocity_y - self.velocity.y) / dt if dt > 0 else 0.0
        )
        self.acceleration.x = (
            cs * self.acceleration_local.x - sn * self.acceleration_local.y
        )
        self.acceleration.y = (
            sn * self.acceleration_local.x + cs * self.acceleration_local.y
        )

        # Accumulate total distance for statistics (m)
        self.distance_moved += self.absolute_velocity * dt

    def filter_abs(self, brake: float) -> float:
        if brake >= 0.5:
            result = brake if self.last_brake_loosened else 0.0
            self.last_brake_loosened = not self.last_brake_loosened
            return result

        if brake < 0.1:
            self.last_brake_loosened = True

        return brake
