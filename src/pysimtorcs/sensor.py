import random

from pysimtorcs.settings import MAX_RANDOM_DEVIATION


class SensorInformation:
    """
    Python translation of the simtorcs.car.SensorInformation class.
    """

    def __init__(self, noise: bool, num_of_sensors: int):
        self.noise = noise
        self.num_of_sensors = num_of_sensors

        # Equivalent in TORCS SCR: angle.
        self.angle_to_track_axis = 0.0

        # Equivalent in TORCS SCR: trackPos.
        self.distance_to_track_axis = 0.0

        # Equivalent in TORCS SCR: speed.
        self.absolute_velocity = 0.0

        # Equivalent in TORCS SCR: track.
        self.track_edge_sensors: list[float] = [0.0 for _ in range(num_of_sensors)]

        # Attributes in TORCS SCR to: distRaced.
        self.rounds_finished = 0
        self.lap_position = 0.0
        self.segment_position = 0.0

        # The summed speed difference in or before turns (to the respective maximum possible speed).
        self.ticks_in_or_before_turns = 0

        # The number of ticks driven in or before turns.
        self.too_low_turn_speed = 0.0

        # The summed distance in meters driven without steering.
        self.length_driven_straight = 0.0

        # The summed steering angle (absolute value).
        self.total_steering = 0.0

        # Use the Mersenne Twister random number generator.
        self._random = random.Random()

    def get_distance_raced(self, track_length: float) -> float:
        return (
            self.rounds_finished * track_length
            + self.lap_position
            + self.segment_position
        )

    def perturb_if_necessary(self):
        if self.noise:
            for index in range(len(self.track_edge_sensors)):
                actual_value = self.track_edge_sensors[index]
                perturbed_value = (
                    actual_value
                    + MAX_RANDOM_DEVIATION * (self._random.random() - 0.5) * 2.0
                )
                perturbed_value = min(max(perturbed_value, 0.0), 1.0)
                self.track_edge_sensors[index] = perturbed_value
