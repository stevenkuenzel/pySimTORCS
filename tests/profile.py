from pysimtorcs import settings
from pysimtorcs.controller import TestController
from pysimtorcs.track import Track, import_from_torcs
from pysimtorcs.race import Race


def benchmark():
    track: Track = import_from_torcs("Brondehach", 1)

    num_of_cars = 10
    fps = 30

    race: Race = Race(track, False, False)

    for i in range(num_of_cars):
        target_speed = 10 + i * 5
        race.create_car(TestController(target_speed))

    while race.time_max_sec > race.time_now and not race.race_finished:
        race.update(1.0 / fps)

    print("Time elapsed:", race.time_now, "sec")


if __name__ == "__main__":
    from pysimtorcs.game import Game

    if True:
        game: Game = Game("Brondehach", TestController(10.0), False, 300)
        game.run()
    else:
        benchmark()
