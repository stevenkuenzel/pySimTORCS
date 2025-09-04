import pygame
from pygame.locals import *

from pysimtorcs.controller import CarController, TestController
import pysimtorcs.settings as settings
from pysimtorcs.geometry import create_vector2_from_rad
from pysimtorcs.race import Race
from pysimtorcs.segments import CoordinateSegment, TurnDirection
from pysimtorcs.track import Track, import_from_torcs

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)


class GameGUI:
    def __init__(self, race: Race):
        self._running = True
        self._paused = False
        self._display_surf = None
        self.clock = None
        self.size = self.width, self.height = 1600, 900
        self.race = race
        self.track = race.track
        self.speed_modifier: float = 1.0

        self.draw_size = round(self.height * 0.8)

    def on_init(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self._display_surf = pygame.display.set_mode(
            self.size, pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self._display_surf.fill(WHITE)
        self._running = True

    def on_event(self, event: pygame.event.Event):
        if event.type == pygame.QUIT:
            self._running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                self.speed_modifier /= 2.0
            elif event.key == pygame.K_d:
                self.speed_modifier *= 2.0

    def on_loop(self):
        pass

    def write_text(
        self, text: str, x: int, y: int, size: int = 36, bold: bool = False, color=BLACK
    ):
        font = pygame.font.Font(None, size)
        font.set_bold(bold)
        text_surf = font.render(text, True, color)
        self._display_surf.blit(text_surf, (x, y))

    def on_render(self):
        self._display_surf.fill(WHITE)

        color = BLACK
        for segment in self.track.segments:
            if isinstance(segment, CoordinateSegment):
                if (
                    segment.turn_direction == TurnDirection.Right
                    and abs(segment.turn_angle) >= self.track.min_turn_rad
                    and segment.length_measured <= 50
                ):
                    color = RED
                elif (
                    segment.turn_direction == TurnDirection.Left
                    and abs(segment.turn_angle) >= self.track.min_turn_rad
                    and segment.length_measured <= 50
                ):
                    color = BLUE
                else:
                    color = GREEN
            else:
                color = BLACK

            for line in segment.get_normalized_line_segments(
                self.track.x_min, self.track.x_max, self.track.y_min, self.track.y_max
            ):  # Updated line
                x1 = line.from_point.x * self.draw_size
                y1 = line.from_point.y * self.draw_size
                x2 = line.to_point.x * self.draw_size
                y2 = line.to_point.y * self.draw_size

                pygame.draw.line(self._display_surf, color, (x1, y1), (x2, y2), 1)

        # Write some text
        text_x = self.draw_size + 50
        text_y = 50
        self.write_text("Race", text_x, text_y, 36, True)
        text_y += 50
        self.write_text(
            f"Time: {self.race.time_now:.2f} / {self.race.time_max_sec:.2f}",
            text_x,
            text_y,
            24,
            True,
        )
        text_y += 50
        self.write_text("Cars", text_x, text_y, 36, True)

        for car in self.race.cars:
            text_y += 40
            self.write_text(f"Car X", text_x + 10, text_y, 24, True)
            text_y += 30
            self.write_text(
                f"  Lap: {car.sensor_information.lap_position}", text_x + 10, text_y, 24
            )
            text_y += 30
            self.write_text(f"  Pos: {car.position}", text_x + 10, text_y, 24)
            text_y += 30
            self.write_text(
                f"  Heading: {car.heading:.2f} rad", text_x + 10, text_y, 24
            )
            text_y += 30
            self.write_text(
                f"  Segment: {car.current_segment}", text_x + 10, text_y, 24
            )
            text_y += 30
            self.write_text(
                f"  Speed: {car.sensor_information.absolute_velocity:.0f} kph",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30

            # SENSORS
            self.write_text(
                "  Track Sensors: ",
                text_x + 10,
                text_y,
                24,
            )

            directions = car.update_sensor_target_vectors()
            lengths = car.sensor_information.track_edge_sensors

            # Draw the sensor directions
            center_x = text_x + 200
            center_y = text_y + 200

            pygame.draw.circle(self._display_surf, BLACK, (center_x, center_y), 3)
            pygame.draw.circle(self._display_surf, BLACK, (center_x, center_y), 202, 1)

            for direction in directions:
                end_x = center_x + direction.x * 200
                end_y = center_y + direction.y * 200
                pygame.draw.line(
                    self._display_surf, BLACK, (center_x, center_y), (end_x, end_y), 1
                )
                # Draw the length
                length = lengths[directions.index(direction)]
                length_x = center_x + direction.x * length * 200
                length_y = center_y + direction.y * length * 200
                pygame.draw.circle(self._display_surf, RED, (length_x, length_y), 3)

            x = round(
                (car.position.x - self.track.x_min)
                / (self.track.x_max - self.track.x_min)
                * self.draw_size
            )
            y = round(
                (car.position.y - self.track.y_min)
                / (self.track.y_max - self.track.y_min)
                * self.draw_size
            )
            pygame.draw.circle(self._display_surf, BLACK, (x, y), 5)

            looking_dir = create_vector2_from_rad(car.heading)
            end_point = car.position + looking_dir * 8.0

            x_to = round(
                (end_point.x - self.track.x_min)
                / (self.track.x_max - self.track.x_min)
                * self.draw_size
            )
            y_to = round(
                (end_point.y - self.track.y_min)
                / (self.track.y_max - self.track.y_min)
                * self.draw_size
            )

            pygame.draw.line(self._display_surf, BLACK, (x, y), (x_to, y_to), 2)

        self.write_text("Speed:", 200, self.height - 150, 36, True)
        self.write_text(
            f"A <<   {self.speed_modifier:.3f}x   >> D",
            200,
            self.height - 100,
            36,
            False,
        )

        pygame.display.flip()

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self):
        self.on_init()

        while self._running:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_SPACE]:
                self._paused = True
            else:
                self._paused = False

            for event in pygame.event.get():
                self.on_event(event)

            dt = self.clock.tick(settings.FPS) / 1000.0
            if not self._paused:
                print(f"dt: {dt:.4f} s, FPS: {self.clock.get_fps():.2f}")
                self.race.update(dt * self.speed_modifier)

                self.on_loop()
                self.on_render()

            # if r.race_finished:
            #     break
        self.on_cleanup()


class Game:
    def __init__(self, track_name: str, controller: CarController, noise: bool):
        self.track_name = track_name
        self.controller = controller
        self.noise = noise

    def run(self):
        track = import_from_torcs(self.track_name, 1)
        race = Race(track, self.noise)
        race.create_car(self.controller)

        gui = GameGUI(race)
        gui.on_execute()


if __name__ == "__main__":
    game: Game = Game("Brondehach", TestController(15), False)
    game.run()
