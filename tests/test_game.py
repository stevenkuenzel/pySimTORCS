from pysimtorcs.controller import TestController
from pysimtorcs.game import Game


game: Game = Game("Brondehach", TestController(15.0), False, 300)

game.run()
