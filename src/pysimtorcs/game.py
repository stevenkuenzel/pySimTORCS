from math import atan2, cos, hypot, sin
import math

import pygame
from pygame.locals import *
from pygame.math import Vector2

from pysimtorcs.geometry import create_vector2_from_rad
import pysimtorcs.settings as settings
from pysimtorcs.car import Car
from pysimtorcs.controller import CarController, TestController
from pysimtorcs.race import Race
from pysimtorcs.segments import CoordinateSegment, TurnDirection
from pysimtorcs.track import import_from_torcs

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
LIGHT_RED = (255, 150, 150)
LIGHT_BLUE = (150, 150, 255)
LIGHT_GREEN = (150, 255, 150)
LIGHT_GRAY = (200, 200, 200)


class GameGUI:
    @staticmethod
    def draw_dashed_line(
        surf, color, start_pos, end_pos, width=2, dash_length=10, space_length=5
    ):
        x1, y1 = start_pos
        x2, y2 = end_pos
        dx = x2 - x1
        dy = y2 - y1
        distance = hypot(dx, dy)
        angle = atan2(dy, dx)
        steps = int(distance // (dash_length + space_length))
        for i in range(steps + 1):
            start_x = x1 + (dx * ((i * (dash_length + space_length)) / distance))
            start_y = y1 + (dy * ((i * (dash_length + space_length)) / distance))
            end_x = start_x + cos(angle) * min(
                dash_length, distance - (i * (dash_length + space_length))
            )
            end_y = start_y + sin(angle) * min(
                dash_length, distance - (i * (dash_length + space_length))
            )
            if hypot(end_x - x1, end_y - y1) > distance:
                end_x, end_y = x2, y2
            pygame.draw.line(surf, color, (start_x, start_y), (end_x, end_y), width)

    def __init__(self, race: Race):
        self._running = True
        self._paused = False
        self._display_surf = None
        self.clock = None
        self.size = self.width, self.height = 1600, 900
        self.race = race
        self.track = race.track
        self.speed_modifier: float = 1.0
        self.draw_whole_track = True
        self.save_next_frame = False

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
                self.speed_modifier = max(self.speed_modifier, 0.125)
            elif event.key == pygame.K_d:
                self.speed_modifier *= 2.0
                self.speed_modifier = min(self.speed_modifier, 16.0)
            elif event.key == pygame.K_m:
                self.draw_whole_track = not self.draw_whole_track
            elif event.key == pygame.K_p:
                self.save_next_frame = True

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

        if not self.draw_whole_track:
            # Zoom in on the car and draw only the car and the next N segments larger
            ZOOM_FACTOR = 15  # How much to zoom in (increase for more zoom)
            # SEGMENTS_AHEAD =16 # How many segments ahead to draw
            LENGTH_AHEAD = settings.SENSOR_RANGE  # How many meters ahead to draw

            car = self.race.cars[0]
            car_pos = car.position

            # Center of the screen for drawing
            center_x = self.draw_size // 2
            center_y = self.draw_size // 2

            def world_to_screen(pos):
                # Translate world coordinates to screen, centered on car, with zoom
                dx = (
                    (pos.x - car_pos.x)
                    * ZOOM_FACTOR
                    * self.draw_size
                    / (self.track.x_max - self.track.x_min)
                )
                dy = (
                    (pos.y - car_pos.y)
                    * ZOOM_FACTOR
                    * self.draw_size
                    / (self.track.y_max - self.track.y_min)
                )
                return (int(center_x + dx), int(center_y + dy))

            # Draw only the next N segments ahead of the car
            car_segment = car.current_segment
            try:
                seg_idx = self.track.segments.index(car_segment)
            except ValueError:
                seg_idx = 0

            current_length_drawn = 0
            next_draw_segment_idx = seg_idx
            while current_length_drawn < LENGTH_AHEAD:
                segment = self.track.segments[next_draw_segment_idx]
                current_length_drawn += segment.length_measured
                next_draw_segment_idx = (next_draw_segment_idx + 1) % len(
                    self.track.segments
                )

                if isinstance(segment, CoordinateSegment):
                    if (
                        segment.turn_direction == TurnDirection.Right
                        and abs(segment.turn_angle) >= self.track.min_turn_rad
                        and segment.length_measured <= 50
                    ):
                        color = LIGHT_RED
                    elif (
                        segment.turn_direction == TurnDirection.Left
                        and abs(segment.turn_angle) >= self.track.min_turn_rad
                        and segment.length_measured <= 50
                    ):
                        color = LIGHT_BLUE
                    else:
                        color = LIGHT_GREEN
                else:
                    color = LIGHT_GRAY

                for line in segment.segment_lines:
                    x1, y1 = world_to_screen(line.from_point)
                    x2, y2 = world_to_screen(line.to_point)
                    pygame.draw.line(self._display_surf, color, (x1, y1), (x2, y2), 4)

            # Draw the car at the center
            pygame.draw.circle(self._display_surf, BLACK, (center_x, center_y), 12)

            # Draw the sensors.
            directions = car.update_sensor_target_vectors()
            lengths = car.sensor_information.track_edge_sensors

            # Draw the sensor directions from the car to the track edge
            for i, direction in enumerate(directions):
                # Sensor starts at car center (center_x, center_y)
                # Sensor ends at the intersection with the track edge
                # Find the actual intersection point with the track edge
                length = lengths[i] * settings.SENSOR_RANGE
                # The length is in meters, so we need to convert it to screen coordinates
                # Use the same scaling as world_to_screen, but for a vector
                end_world = car.position + direction * length
                end_x, end_y = world_to_screen(end_world)
                # Draw a dashed line from the car to the sensor endpoint

                GameGUI.draw_dashed_line(
                    self._display_surf,
                    BLACK,
                    (center_x, center_y),
                    (end_x, end_y),
                    2,
                    10,
                    5,
                )
                # Draw a small circle at the end point (track edge)
                color = GREEN
                if i < len(directions) // 2:
                    color = RED
                elif i > len(directions) // 2:
                    color = BLUE
                pygame.draw.circle(
                    self._display_surf, color, (int(end_x), int(end_y)), 5
                )

            # Save the current display surface to a file
            if self.save_next_frame:
                pygame.image.save(self._display_surf, "screenshot.png")
                self.save_next_frame = False
        else:
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
                    self.track.x_min,
                    self.track.x_max,
                    self.track.y_min,
                    self.track.y_max,
                ):  # Updated line
                    x1 = line.from_point.x * self.draw_size
                    y1 = line.from_point.y * self.draw_size
                    x2 = line.to_point.x * self.draw_size
                    y2 = line.to_point.y * self.draw_size

                    pygame.draw.line(self._display_surf, color, (x1, y1), (x2, y2), 1)

            for car in self.race.cars:
                car_x = round(
                    (car.position.x - self.track.x_min)
                    / (self.track.x_max - self.track.x_min)
                    * self.draw_size
                )
                car_y = round(
                    (car.position.y - self.track.y_min)
                    / (self.track.y_max - self.track.y_min)
                    * self.draw_size
                )
                pygame.draw.circle(self._display_surf, BLACK, (car_x, car_y), 5)

                car_heading = create_vector2_from_rad(car.heading)
                end_point = car.position + car_heading * 8

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

                pygame.draw.line(
                    self._display_surf, BLACK, (car_x, car_y), (x_to, y_to), 1
                )

        # Write some text
        text_x = self.width - 420
        text_y = 50
        self.write_text("Race", text_x, text_y, 36, True)
        text_y += 50
        self.write_text(
            f"Time: {self.race.time_now:.2f} / {self.race.time_max_sec:.2f} s",
            text_x,
            text_y,
            24,
            True,
        )
        text_y += 50

        def write_car_info(car: Car, text_x: int, text_y: int, draw_sensors: bool):
            self.write_text(f"Car {car.id}", text_x + 10, text_y, 36, True)
            text_y += 40
            self.write_text(
                f"  Lap: {car.sensor_information.rounds_finished + 1}",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30
            self.write_text(
                f"  Segment: #{car.current_segment.id if car.current_segment else 'None'}",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30
            distance_driven = car.sensor_information.get_distance_raced(
                self.track.length
            )
            self.write_text(
                f"  Distance driven: {distance_driven:.2f} m",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30
            self.write_text(
                f"  Position: {car.position.x:.2f}, {car.position.y:.2f}",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30
            self.write_text(
                f"  Heading: {(car.heading % (2 * math.pi)):.2f} rad",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30
            self.write_text(
                f"  Speed: {car.sensor_information.absolute_velocity:.0f} m/s = {car.sensor_information.absolute_velocity * 3.6:.0f} km/h",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30
            self.write_text(
                f"  Average Speed: {(distance_driven / self.race.time_now):.2f} m/s = {(distance_driven / self.race.time_now * 3.6):.2f} km/h",
                text_x + 10,
                text_y,
                24,
            )
            text_y += 30

            if not draw_sensors:
                # Draw throttle and steering as arrow.
                center_x = text_x + 200
                center_y = text_y + 250

                arrow_start = Vector2(center_x, center_y)
                arrow_direction = Vector2(car.steer_angle, -car.throttle)
                length_multiplier = 200
                arrow_end = arrow_start + arrow_direction * length_multiplier

                # Draw throttle and steering as arrows
                # Throttle arrow (vertical, up for positive throttle)
                throttle = car.throttle  # -1..1
                arrow_color = GREEN if throttle >= 0 else RED
                pygame.draw.line(
                    self._display_surf,
                    arrow_color,
                    (center_x, center_y),
                    (arrow_end.x, arrow_end.y),
                    6,
                )

                pygame.draw.circle(
                    self._display_surf, BLACK, (center_x, center_y), 200, 1
                )

                return

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
            center_y = text_y + 250

            pygame.draw.circle(self._display_surf, BLACK, (center_x, center_y), 3)
            pygame.draw.circle(self._display_surf, BLACK, (center_x, center_y), 202, 1)

            for i, direction in enumerate(directions):
                start_x = center_x + direction.x * 5
                start_y = center_y + direction.y * 5
                end_x = center_x + direction.x * 200
                end_y = center_y + direction.y * 200
                pygame.draw.line(
                    self._display_surf, BLACK, (start_x, start_y), (end_x, end_y), 1
                )
                # Draw the length
                length = lengths[i]
                length_x = center_x + direction.x * length * 200
                length_y = center_y + direction.y * length * 200
                pygame.draw.circle(self._display_surf, RED, (length_x, length_y), 3)

        write_car_info(self.race.cars[0], text_x, text_y, self.draw_whole_track)

        self.write_text(
            f"Simulation Speed: A <<   {self.speed_modifier:.3f}x   >> D",
            20,
            self.height - 75,
            24,
            False,
        )
        self.write_text(
            "Switch Mode: M",
            20,
            self.height - 50,
            24,
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
                # print(f"dt: {dt:.4f} s, FPS: {self.clock.get_fps():.2f}")
                self.race.update(dt * self.speed_modifier)

                self.on_loop()
                self.on_render()

            # if r.race_finished:
            #     break
        self.on_cleanup()


class Game:
    def __init__(
        self, track_name: str, controller: CarController, noise: bool, time_max_sec: int
    ):
        self.track_name = track_name
        self.controller = controller
        self.time_max_sec = time_max_sec
        self.noise = noise

    def run(self):
        track = import_from_torcs(self.track_name, 1)
        race = Race(track, self.noise, self.time_max_sec)
        race.create_car(self.controller)

        gui = GameGUI(race)
        gui.on_execute()


if __name__ == "__main__":
    game: Game = Game("Brondehach", TestController(15), False, 300)
    game.run()
