MAX_RANDOM_DEVIATION = 0.1
"""
The maximum random deviation applied to sensor readings.
"""



FPS = 100
# DT = 1.0 / FPS

# Maximum and minimum speed considered (in m/s)
SPEED_MIN = 50.0 / 3.6

# The maximum speed of the car (approx. max. speed of car1-trb1 in TORCS)
SPEED_MAX = 330.0 / 3.6

# The maximum range covered by the track edge sensors (in meters)
SENSOR_RANGE = 200.0

# The maximum deviation of the track edge sensor values in percent
MAX_RANDOM_DEVIATION = 0.05

# FITNESS RELATED CONSTANTS

# The maximum possible turn speed is multiplied with that constant.
# If the car is driving faster through the turn than the resulting value,
# it positively contributes to the corresponding fitness value.
TURN_SPEED_MULTIPLIER = 0.95

# The distance at which the possible speed is linearly reduced to the turn speed (in meters)
TURN_SPEED_DISTANCE_MAX = 50.0

# The maximum inclination of the steering wheel to be considered as driving straight
# (in percent of STEER_MAX radians)
THRESHOLD_DRIVING_STRAIGHT = 0.05