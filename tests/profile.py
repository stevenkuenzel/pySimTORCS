from pysimtorcs import settings
from pysimtorcs.controller import TestController
from pysimtorcs.track import Track, import_from_torcs
from pysimtorcs.race import Race

num_of_cars = 10


track: Track = import_from_torcs("Brondehach", 1)

race: Race = Race(track, False, False)

for i in range(num_of_cars):
    target_speed = 10 + i * 5
    race.create_car(TestController(target_speed))

while race.time_max_sec > race.time_now and not race.race_finished:
    race.update(1.0 / settings.FPS)

print("Time elapsed:", race.time_now, "sec")