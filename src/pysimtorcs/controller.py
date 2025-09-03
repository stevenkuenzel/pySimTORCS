import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from pysimtorcs.sensor import SensorInformation


@dataclass
class CarInput:
    left: float = 0.0
    right: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


class CarController(ABC):
    @abstractmethod
    def control(self, si: SensorInformation):
        """Compute CarInput from SensorInformation"""
        pass


class TestController(CarController):
    def __init__(self, target_speed: float):
        self.target_speed: float = target_speed

    def control(self, si: SensorInformation) -> CarInput:
        target_steer = si.angle_to_track_axis - si.distance_to_track_axis * 0.5
        acceleration_and_brake = (
            2.0 / (1.0 + math.exp(si.absolute_velocity - self.target_speed)) - 1.0
        )

        left = target_steer if target_steer > 0.0 else 0.0
        right = -target_steer if target_steer < 0.0 else 0.0
        throttle = acceleration_and_brake if acceleration_and_brake > 0.0 else 0.0
        brake = -acceleration_and_brake if acceleration_and_brake < 0.0 else 0.0

        return CarInput(left=left, right=right, throttle=throttle, brake=brake)
