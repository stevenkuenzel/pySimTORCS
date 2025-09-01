import pygame
from pygame.locals import *

from geometry import create_vector2_from_rad
from race import Race
from segments import CoordinateSegment, TurnDirection
from track import Track, import_from_torcs

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

class App:
    def __init__(self, race: Race):
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = 1280, 1024
        self.race = race
        self.track = race.track

        self.draw_size = round(self.height * 0.8)

    def on_init(self):
        pygame.init()
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._display_surf.fill(WHITE)
        self._running = True
 
    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False
    def on_loop(self):
        pass
    def on_render(self):
        self._display_surf.fill(WHITE)

        color = BLACK
        for segment in self.track.segments:
            if isinstance(segment, CoordinateSegment):
                if segment.turn_direction == TurnDirection.Right and abs(segment.turn_angle) >= self.track.min_turn_rad and segment.length_measured <= 50:
                    color = RED
                elif segment.turn_direction == TurnDirection.Left and abs(segment.turn_angle) >= self.track.min_turn_rad and segment.length_measured <= 50:
                    color = BLUE
                else:
                    color = GREEN
            else:
                color = BLACK

            for line in segment.get_normalized_line_segments(self.track.x_min, self.track.x_max, self.track.y_min, self.track.y_max):  # Updated line
                x1 = line.from_point.x * self.draw_size
                y1 = line.from_point.y * self.draw_size
                x2 = line.to_point.x * self.draw_size
                y2 = line.to_point.y * self.draw_size

                # print(f"Drawing line from ({x1}, {y1}) to ({x2}, {y2}) with color {color}")

                pygame.draw.line(self._display_surf, color, (x1, y1), (x2, y2), 1)
        
        for car in self.race.cars:
            x = round((car.position.x - self.track.x_min) / (self.track.x_max - self.track.x_min) * self.draw_size)
            y = round((car.position.y - self.track.y_min) / (self.track.y_max - self.track.y_min) * self.draw_size)
            pygame.draw.circle(self._display_surf, BLACK, (x, y), 5)

            looking_dir = create_vector2_from_rad(car.heading) 
            end_point = car.position + looking_dir * 10.0

            x_to = round((end_point.x - self.track.x_min) / (self.track.x_max - self.track.x_min) * self.draw_size)
            y_to = round((end_point.y - self.track.y_min) / (self.track.y_max - self.track.y_min) * self.draw_size)

            pygame.draw.line(self._display_surf, RED, (x, y), (x_to, y_to), 2)



        pygame.display.flip()

    def on_cleanup(self):
        pygame.quit()
 
    def on_execute(self):
        self.on_init()

        while( self._running ):
            r.update()

            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()

            # if r.race_finished:
            #     break
        self.on_cleanup()
 
if __name__ == "__main__" :
    t : Track = import_from_torcs("Brondehach", 1)
    r : Race = Race(t, False)
    r.create_car()
    theApp = App(r)
    theApp.on_execute()