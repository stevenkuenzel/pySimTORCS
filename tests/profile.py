from pysimtorcs import settings
from pysimtorcs.track import Track, import_from_torcs
from pysimtorcs.race import Race


t: Track = import_from_torcs("Brondehach", 1)
r: Race = Race(t, False)
r.create_car()

while r.t_max > r.t_now and not r.race_finished:
    r.update(1.0 / settings.FPS)
