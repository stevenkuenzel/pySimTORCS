import math

import pysimtorcs.settings as settings
from pysimtorcs.controller import CarController, TestController
from pysimtorcs.geometry import Vector2, create_vector2_from_rad
from pysimtorcs.segments import Segment, Turn
from pysimtorcs.sensor import SensorInformation
from pysimtorcs.util import clamp, sign
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
PHYSICS_CG_TO_FRONT_AXLE = 0.97  # Centre gravity to front axle
PHYSICS_CG_TO_REAR_AXLE = 0.97  # Centre gravity to rear axle
PHYSICS_CG_HEIGHT = 0.25  # Centre gravity height
PHYSICS_TIRE_GRIP = 2.0  # How much grip tires have
PHYSICS_WEIGHT_TRANSFER = (
    0.2  # How much weight is transferred during acceleration/braking
)
PHYSICS_CORNER_STIFFNESS_FRONT = 5.0
PHYSICS_CORNER_STIFFNESS_REAR = 5.2
PHYSICS_AIR_RESIST = 2.5  # air resistance (* vel)
PHYSICS_ROLL_RESIST = 8.0  # rolling resistance force (* vel)
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
        noisy_sensors: bool,
        heading: float,
        position: Vector2,
    ):
        self.id: int = id
        self.controller: CarController = controller
        self.noisy_sensors: bool = noisy_sensors

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
        self.sensor_information = SensorInformation(
            noisy_sensors, len(self.sensor_angles)
        )

        self.current_segment: Segment = None
        self.last_valid_segment: Segment = None
        self.previous_segment: Segment = None

        # Input / Control.
        self.throttle = 0.0
        self.brake = 0.0
        self.steer_angle = 0.0

        # Fitness.
        self.distance_raced = 0.0
        self.speed_reached_max = 0.0

        # The real distance moved. Useful for fitness calculation.
        self.distance_moved = 0.0

        # Physics.
        self.velocity = Vector2()
        self.velocity_local = Vector2()
        self.acceleration = Vector2()
        self.acceleration_local = Vector2()
        self.absolute_velocity = 0.0
        self.previous_absolute_velocity = 0.0 # For Fitness.
        self.yaw_rate = 0.0

        # ABS flag.
        self.last_brake_loosened = True

        self.reward = 0.0

        self.reward_velocity_factor = 0.1
        self.reward_track_center_factor = 0.25
        self.penalty_steering_factor = 1.0
        self.penalty_speed_change_factor = 1.0

    def update(self, dt: float, track_length: float):
        input = self.controller.control(self.sensor_information)
        target_steer = input.left - input.right

        self.update_fitness_related_information(dt, target_steer, track_length)

        self.throttle = input.throttle
        self.brake = self.filter_abs(input.brake)
        self.steer_angle = target_steer * STEER_MAX

        self.update_physics(dt)
        self.update_reward()

    def update_reward(self):
        # Velocity:
        reward_velocity = self.reward_velocity_factor* self.absolute_velocity# / settings.SPEED_MAX

        # Track center:
        reward_track_center = self.reward_track_center_factor * math.exp(-.5 * (self.sensor_information.distance_to_track_axis**2))

        # Steering penalty:
        penalty_steering = -self.penalty_steering_factor * abs(self.steer_angle)

        # Speed change penalty:
        penalty_speed_change = -self.penalty_speed_change_factor * abs(self.absolute_velocity - self.previous_absolute_velocity)

        # print(f"Velocity reward: {reward_velocity:.3f}, Track center reward: {reward_track_center:.3f}, Steering penalty: {penalty_steering:.3f}, Speed change penalty: {penalty_speed_change:.3f}")

        next_reward = reward_velocity + reward_track_center + penalty_steering + penalty_speed_change
        self.reward += next_reward

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

    def update_sensor_target_vectors(self) -> list:
        # TODO: CAN THIS BE FASTER?
        targets_angles = self.heading + self.sensor_angles
        targets_angles = (targets_angles + np.pi) % (2 * np.pi) - np.pi
        targets = [create_vector2_from_rad(angle) for angle in targets_angles]
        return targets
        # targets = []
        # for i in range(len(self.sensor_angles)):
        #     target = self.heading + self.sensor_angles[i]
        #     if target > math.pi:
        #         target -= 2.0 * math.pi
        #     if target < -math.pi:
        #         target += 2.0 * math.pi
        #     targets.append(create_vector2_from_rad(target))
        # return targets

    def update_physics(self, dt: float):
        # Calculate sine and cosine of heading for coordinate transforms
        sn = math.sin(self.heading)
        cs = math.cos(self.heading)

        # Transform velocity to local car coordinates (m/s)
        self.velocity_local.x = cs * self.velocity.x + sn * self.velocity.y
        self.velocity_local.y = cs * self.velocity.y - sn * self.velocity.x

        # Calculate axle weights (N)
        axle_weight_front = PHYSICS_MASS * (
            PHYSICS_AXLE_WEIGHT_RATIO_FRONT * PHYSICS_GRAVITY
            - PHYSICS_WEIGHT_TRANSFER
            * self.acceleration_local.x
            * PHYSICS_CG_HEIGHT
            / PHYSICS_WHEEL_BASE
        )
        axle_weight_rear = PHYSICS_MASS * (
            PHYSICS_AXLE_WEIGHT_RATIO_REAR * PHYSICS_GRAVITY
            + PHYSICS_WEIGHT_TRANSFER
            * self.acceleration_local.x
            * PHYSICS_CG_HEIGHT
            / PHYSICS_WHEEL_BASE
        )

        # Calculate yaw speeds at front and rear axles (rad/s)
        yaw_speed_front = PHYSICS_CG_TO_FRONT_AXLE * self.yaw_rate
        yaw_speed_rear = -PHYSICS_CG_TO_REAR_AXLE * self.yaw_rate

        # Calculate slip angles (rad)
        slip_angle_front = (
            math.atan2(
                self.velocity_local.y + yaw_speed_front, abs(self.velocity_local.x)
            )
            - sign(self.velocity_local.x) * self.steer_angle
        )
        slip_angle_rear = math.atan2(
            self.velocity_local.y + yaw_speed_rear, abs(self.velocity_local.x)
        )

        # Tire grip coefficients (unitless)
        tire_grip_front = PHYSICS_TIRE_GRIP
        tire_grip_rear = PHYSICS_TIRE_GRIP

        # Lateral friction forces at front and rear tires (N)
        friction_force_front_cy = (
            clamp(
                -PHYSICS_CORNER_STIFFNESS_FRONT * slip_angle_front,
                -tire_grip_front,
                tire_grip_front,
            )
            * axle_weight_front
        )
        friction_force_rear_cy = (
            clamp(
                -PHYSICS_CORNER_STIFFNESS_REAR * slip_angle_rear,
                -tire_grip_rear,
                tire_grip_rear,
            )
            * axle_weight_rear
        )

        # Get brake and throttle forces (N)
        brake = self.brake * PHYSICS_BRAKE_FORCE
        throttle = self.throttle * PHYSICS_ENGINE_FORCE

        # Traction forces (N)
        # Only rear wheel drive (RWD) is modeled
        traction_force_cx = throttle - brake * sign(self.velocity_local.x)
        traction_force_cy = 0.0

        # Drag and rolling resistance forces (N)
        drag_force_cx = (
            -PHYSICS_ROLL_RESIST * self.velocity_local.x
            - PHYSICS_AIR_RESIST * self.velocity_local.x * abs(self.velocity_local.x)
        )
        drag_force_cy = (
            -PHYSICS_ROLL_RESIST * self.velocity_local.y
            - PHYSICS_AIR_RESIST * self.velocity_local.y * abs(self.velocity_local.y)
        )

        # Total force in local car coordinates (N)
        total_force_cx = drag_force_cx + traction_force_cx
        # Lateral force includes tire friction and drag
        total_force_cy = (
            drag_force_cy
            + traction_force_cy
            + math.cos(self.steer_angle) * friction_force_front_cy
            + friction_force_rear_cy
        )

        # Acceleration in local car coordinates (m/s^2)
        self.acceleration_local.x = (
            total_force_cx / PHYSICS_MASS
        )  # forward / reverse acceleration
        self.acceleration_local.y = (
            total_force_cy / PHYSICS_MASS
        )  # lateral acceleration

        # Transform acceleration to world coordinates (m/s^2)
        self.acceleration.x = (
            cs * self.acceleration_local.x - sn * self.acceleration_local.y
        )
        self.acceleration.y = (
            sn * self.acceleration_local.x + cs * self.acceleration_local.y
        )

        # Update velocity in world coordinates (m/s)
        self.velocity.x += self.acceleration.x * dt
        self.velocity.y += self.acceleration.y * dt

        # Calculate absolute velocity (m/s)
        self.previous_absolute_velocity = self.absolute_velocity
        self.absolute_velocity = self.velocity.length()

        # Calculate rotational (yaw) torque (N*m)
        angular_torque = (
            friction_force_front_cy + traction_force_cy
        ) * PHYSICS_CG_TO_FRONT_AXLE - friction_force_rear_cy * PHYSICS_CG_TO_REAR_AXLE

        # Stop the car if velocity is very low and no throttle is applied
        if abs(self.absolute_velocity) < 0.5 and throttle == 0.0:
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            self.absolute_velocity = 0.0
            angular_torque = 0.0
            self.yaw_rate = 0.0

        # Calculate angular acceleration (rad/s^2)
        angular_accel = angular_torque / PHYSICS_INERTIA

        # Update yaw rate (rad/s) and heading (rad)
        self.yaw_rate += angular_accel * dt
        self.heading += self.yaw_rate * dt

        # Update position in world coordinates (m)
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt

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
