import math

from pygame import Vector2

import pysimtorcs.settings as settings
from pysimtorcs.car import STEER_MAX, Car
from pysimtorcs.controller import CarController, TestController
from pysimtorcs.geometry import (
    LineSegment,
    adjacent_point_on_segment,
    point_within_polygon,
)
from pysimtorcs.segments import Segment
from pysimtorcs.sensor import SensorInformation
from pysimtorcs.track import Track
from pysimtorcs.util import sign
import numpy as np


class Race:
    def __init__(self, track: Track, noise: bool, disqualify_after_one_round:bool, time_max_sec: float = 300):
        self.track = track
        self.noise = noise
        self.disqualify_after_one_round = disqualify_after_one_round
        self.time_max_sec: float = time_max_sec
        self.time_now: float = 0
        self.cars: list[Car] = []
        self.race_finished = False

    def run(self):
        DT = 1.0 / settings.FPS
        while self.time_now < self.time_max_sec and not self.race_finished:
            self.update(DT)

    def create_car(self, controller: CarController = TestController(50.0)) -> Car:
        car = Car(
            id=len(self.cars),
            controller=controller,
            noisy_sensors=self.noise,
            heading=self.track.starting_angle,
            position=self.track.starting_point.copy(),
        )
        car.current_segment = (
            self.track.segments[0] if len(self.track.segments) > 0 else None
        )
        self.cars.append(car)
        return car

    def update(self, dt: float):
        if self.race_finished:
            return

        for car in self.cars:
            if car.disqualified:
                continue
            # max_pos = self.time_now * STEER_MAX
            # print("Steering total:", car.sensor_information.total_steering, " of ", max_pos)
            

            self.__update_car_state(car)

            car.update(dt, self.track.length)


        if all(car.disqualified for car in self.cars):
            self.race_finished = True

        self.time_now += dt

        if self.time_now >= self.time_max_sec:
            self.race_finished = True

    def __update_car_state(self, car: Car) -> None:
        sensor_info: SensorInformation = car.sensor_information
        sensor_info.absolute_velocity = car.absolute_velocity

        self.__determine_car_segment(car)
        if car.current_segment is not None:
            car.last_valid_segment = car.current_segment
            sensor_info.lap_position = car.last_valid_segment.length_track_total

            if car.previous_segment != car.current_segment:
                if car.previous_segment is None:
                    # First update. Or after loosing track.
                    pass
                else:
                    if (
                        # TODO: THIS IS NOT WORKING IF DT IS LARGE AND CAR JUMPS OVER THE NEXT SEGMENT.
                        car.previous_segment.id == self.track.segments[-1].id
                        and car.current_segment.id == 0
                    ):
                        sensor_info.rounds_finished += 1

                        if self.disqualify_after_one_round:
                            car.disqualified = True
                    else:
                        segment_diff = car.current_segment.id - car.previous_segment.id
                        if segment_diff < 0:
                            car.disqualified = True

                car.previous_segment = car.current_segment

            sensor_info.track_edge_sensors = self.__update_track_edge_sensors(car)

            segment: Segment = car.current_segment

            # Update Sensors: Angle to track axis, i.e. angle necessary to turn towards, to follow track axis.
            sensor_info.angle_to_track_axis = segment.segment_angle - car.heading
            if sensor_info.angle_to_track_axis < -math.pi:
                sensor_info.angle_to_track_axis += 2.0 * math.pi
            if sensor_info.angle_to_track_axis > math.pi:
                sensor_info.angle_to_track_axis -= 2.0 * math.pi

            # Update Sensors: Current position concerning track length.
            det_axis = sign(
                (segment.center_end.x - segment.center_start.x)
                * (car.position.y - segment.center_start.y)
                - (segment.center_end.y - segment.center_start.y)
                * (car.position.x - segment.center_start.x)
            )
            projected_on_axis = (
                car.position
                if det_axis == 0
                else adjacent_point_on_segment(car.position, segment.axis)
            )
            sensor_info.segment_position = segment.center_start.distance_to(
                projected_on_axis
            )

            # Determine the relative position on the track and its width at that position.
            ratio: float = sensor_info.segment_position / segment.length_measured
            width_at_point: float = (
                segment.width_start * (1 - ratio) + segment.width_end * ratio
            )

            sensor_info.distance_to_track_axis = (
                det_axis
                * car.position.distance_to(projected_on_axis)
                / (0.5 * width_at_point)
                if det_axis != 0
                else 0.0
            )
        else:
            car.disqualified = True

        if abs(sensor_info.distance_to_track_axis) > 0.9:
            car.total_distance_from_track += abs(sensor_info.distance_to_track_axis)

        if not car.disqualified:
            car.total_speed += car.absolute_velocity

            if car.absolute_velocity > car.speed_reached_max:
                car.speed_reached_max = car.absolute_velocity

        sensor_info.perturb_if_necessary()

        return sensor_info

    def __determine_car_segment(self, car: Car) -> None:
        segments_to_check = self.track.grid.get_segments_at_position(car.position)
        # segments_to_check = self.__segment_indices_to_check(car)

        for segment in segments_to_check:
            # for segment_id in segments_to_check:
            # segment = self.track.segments[segment_id]

            if point_within_polygon(car.position, segment.to_polygon()):
                car.current_segment = segment
                return

        car.current_segment = None

    # def __segment_indices_to_check(self, car: Car) -> list[int]:
    #     if car.current_segment is None:
    #         return []

    #     segment_index = car.current_segment.id if car.current_segment is not None else 0
    #     num_segments = len(self.track.segments)
    #     indices = [segment_index - 1, segment_index, segment_index + 1]
    #     # Ensure indices are within valid range using modulo for wrap-around
    #     return [i % num_segments for i in indices]

    
    
    def __update_track_edge_sensors(self, car: Car) -> list[float]:
        if car.disqualified:
            return []

        sensors = car.update_sensor_target_vectors()
        num_sensors = len(sensors)
        data = [1.0] * num_sensors
        sensor_range = settings.SENSOR_RANGE

        segments = self.track.segments
        num_segments = len(segments)

        car_pos = car.position
        current_seg_idx = car.current_segment.id

        for index, sensor in enumerate(sensors):
            sensor_to = car_pos + sensor * sensor_range
            d_min = sensor_range
            found = False

            total_distance = 0.0
            seg_idx = current_seg_idx

            # Only check enough segments to cover the sensor range
            while total_distance < sensor_range:
                segment = segments[seg_idx]
                seg_length = segment.length_measured
                total_distance += seg_length

                for line in segment.segment_lines:
                    intersection = line.intersects(car_pos, sensor_to)
                    if intersection is not None:
                        found = True
                        d_min = car_pos.distance_to(intersection)
                        break
                if found:
                    break

                seg_idx = (seg_idx + 1) % num_segments

            if found:
                data[index] = d_min / sensor_range

        return data
