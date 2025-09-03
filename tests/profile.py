import random
from pysimtorcs import settings
from pysimtorcs.controller import TestController
from pysimtorcs.track import Track, import_from_torcs
from pysimtorcs.race import Race


t: Track = import_from_torcs("Brondehach", 1)
r: Race = Race(t, False)
for _ in range(10):
    r.create_car(TestController(random.uniform(20, 60)))

while r.t_max > r.t_now and not r.race_finished:
    r.update(1.0 / settings.FPS)
