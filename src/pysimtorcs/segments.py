import math
from enum import Enum
from math import atan2

from pysimtorcs.geometry import LineSegment, create_vector2_from_rad
from pygame.math import Vector2

class TurnDirection(Enum):
    Left = 1
    Right = 2
    Straight = 3


class Turn:
    MAX_TURN_SEGMENT_LENGTH_SQR: float = 2500

    def __init__(self, id: int, first_segment: "CoordinateSegment"):
        self.id = id

        self.length: float = 0
        self.angle: float = 0
        self.speed_max: float = 100

        self.direction: TurnDirection = first_segment.turn_direction

        self.segments: list[Segment] = []
        self.add_segment(first_segment)

    def add_segment(self, segment: "Segment"):
        segment.in_turn = self
        self.segments.append(segment)

    def finalize(self):
        self.length = 0
        self.angle = 0

        for segment in self.segments:
            if isinstance(segment, CoordinateSegment):
                self.length += math.sqrt(segment.length_measured)
                self.angle += abs(segment.segment_angle)


class Segment:
    """An abstract base class for all track segments."""

    def __init__(self, previous: "Segment"):
        """Initializes a new segment.

        Args:
            previous (Segment): The previous segment in the track.
            track (Track): The track this segment belongs to.
        """
        self.previous: Segment = previous

        self.id: int = -1
        """The unique identifier for this segment.
        """

        # p3    p4
        # |-----|
        # | end |
        # |     |
        # | str |
        # |-----|
        # p1   p2

        self.p1: Vector2 = Vector2()
        """Position: Start, left side.
        """
        self.p2: Vector2 = Vector2()
        """Position: Start, right side.
        """
        self.p3: Vector2 = Vector2()
        """Position: End, left side.
        """
        self.p4: Vector2 = Vector2()
        """Position: End, right side.
        """

        self.center_start: Vector2 = Vector2()
        self.center_end: Vector2 = Vector2()

        self.segment_lines: list[LineSegment] = []

        self.next_turn: Turn = None
        self.in_turn: Turn = None

        self.axis: LineSegment = None

        # in rad
        self.segment_angle: float = 0
        self.segment_direction: Vector2 = Vector2()

        self.length_measured: float = 0
        self.length_track_total: float = 0
        self.width_start: float = 0
        self.width_end: float = 0

        self.x_min: float = float("inf")
        self.x_max: float = float("-inf")
        self.y_min: float = float("inf")
        self.y_max: float = float("-inf")

    def update_segment_lines(self):
        """Creates / Updates the segment lines for this segment."""
        self.segment_lines.clear()
        self.segment_lines.append(LineSegment(self.p1, self.p3))
        self.segment_lines.append(LineSegment(self.p2, self.p4))

    def update_centers(self):
        """Updates the center points for this segment. Also updates the axis line, segment direction, angle, length and widths."""
        self.center_start = (self.p1 + self.p2) / 2
        self.center_end = (self.p3 + self.p4) / 2

        self.axis = LineSegment(self.center_start, self.center_end)

        self.segment_direction = (self.center_end - self.center_start).normalize()
        self.segment_angle = atan2(self.segment_direction.y, self.segment_direction.x)

        self.length_measured = self.center_end.distance_to(self.center_start)
        self.width_start = self.p2.distance_to(self.p1)
        self.width_end = self.p4.distance_to(self.p3)

    def determine_min_max(self):
        self.x_min = min(self.x_min, self.p1.x, self.p2.x, self.p3.x, self.p4.x)
        self.x_max = max(self.x_max, self.p1.x, self.p2.x, self.p3.x, self.p4.x)
        self.y_min = min(self.y_min, self.p1.y, self.p2.y, self.p3.y, self.p4.y)
        self.y_max = max(self.y_max, self.p1.y, self.p2.y, self.p3.y, self.p4.y)

    def get_normalized_line_segments(
        self, x_min: float, x_max: float, y_min: float, y_max: float
    ) -> list[LineSegment]:
        result: list[LineSegment] = []

        for line_segment in self.segment_lines:
            from_new = Vector2(
                x=(line_segment.from_point.x - x_min) / (x_max - x_min)
                if (x_max - x_min) != 0
                else 0,
                y=(line_segment.from_point.y - y_min) / (y_max - y_min)
                if (y_max - y_min) != 0
                else 0,
            )
            to_new = Vector2(
                x=(line_segment.to_point.x - x_min) / (x_max - x_min)
                if (x_max - x_min) != 0
                else 0,
                y=(line_segment.to_point.y - y_min) / (y_max - y_min)
                if (y_max - y_min) != 0
                else 0,
            )
            result.append(LineSegment(from_new, to_new))

        return result

    def to_polygon(self) -> list[Vector2]:
        return [self.p3, self.p1, self.p2, self.p4]  # , self.p3]

    def get_bbox(self) -> tuple[float, float, float, float]:
        """Returns the bounding box of the segment.

        Returns:
            tuple[float, float, float, float]: (min_x, min_y, max_x, max_y)
        """
        min_x = min(self.p1.x, self.p2.x, self.p3.x, self.p4.x)
        max_x = max(self.p1.x, self.p2.x, self.p3.x, self.p4.x)
        min_y = min(self.p1.y, self.p2.y, self.p3.y, self.p4.y)
        max_y = max(self.p1.y, self.p2.y, self.p3.y, self.p4.y)

        return (min_x, min_y, max_x, max_y)

    def __eq__(self, value):
        if isinstance(value, Segment):
            return self.id == value.id
        return False

    def __hash__(self):
        return hash(self.id)


# Last segment of a track, connecting its end and starting segments.
class ConnectingSegment(Segment):
    def __init__(self, from_segment: Segment, to_segment: Segment):
        super().__init__(from_segment)

        self.p1 = from_segment.p3
        self.p2 = from_segment.p4
        self.p3 = to_segment.p1
        self.p4 = to_segment.p2

        self.update_segment_lines()
        self.update_centers()


# Connects two coordinate segments with deviating directions.
# The segment has only three points. The third point depends on the turn direction and is one of the starting points of the _to_ segment.
# Furthermore, it only contains a single (the "outside") segment line.

# p3   (p4)
# |\
# | \
# |  \
# |   \
# |----\
# p1   p2

#  OR:


# (p3)  p4
#     /|
#    / |
#   /  |
#  /   |
# /----|
# p1   p2
class EdgeSegment(Segment):
    def __init__(
        self, from_segment: Segment, to_segment: Segment, turn_direction: TurnDirection
    ):
        super().__init__(from_segment)

        self.to_segment: Segment = to_segment
        self.turn_direction: TurnDirection = turn_direction

        # DO NOT COPY POINTS HERE. USE REFERENCES.
        self.p1 = from_segment.p3#.copy()
        self.p2 = from_segment.p4#.copy()

        if turn_direction == TurnDirection.Right:
            self.p3 = self.p1
            self.p4 = to_segment.p2

            self.third_point = self.p4

            self.segment_lines.append(LineSegment(self.p2, self.p4))
        elif turn_direction == TurnDirection.Left:
            self.p3 = to_segment.p1
            self.p4 = self.p2

            self.third_point = self.p3

            self.segment_lines.append(LineSegment(self.p1, self.p3))
        else:
            raise ValueError("EdgeSegment must have a turn direction of Left or Right")

        self.update_centers()

    def to_polygon(self) -> list[Vector2]:
        return [self.third_point, self.p1, self.p2]  # , self.third_point]

    def get_bbox(self) -> tuple[float, float, float, float]:
        min_x = min(self.third_point.x, self.p1.x, self.p2.x)
        max_x = max(self.third_point.x, self.p1.x, self.p2.x)
        min_y = min(self.third_point.y, self.p1.y, self.p2.y)
        max_y = max(self.third_point.y, self.p1.y, self.p2.y)

        return (min_x, min_y, max_x, max_y)


class CoordinateSegment(Segment):
    def __init__(
        self, length: float, width_end: float, turn_angle: float, previous: Segment
    ):
        super().__init__(previous)

        self.length: float = length
        self.width_end: float = width_end
        self.turn_angle: float = turn_angle

        self.turn_direction: TurnDirection = TurnDirection.Straight
        if turn_angle < 0:
            self.turn_direction = TurnDirection.Left
        elif turn_angle > 0:
            self.turn_direction = TurnDirection.Right

    def initialize(self, width_start: float) -> EdgeSegment | None:
        self.p1 = (
            self.previous.p3.copy()
            if self.previous is not None
            else Vector2(0, width_start / 2)
        )
        self.p2 = (
            self.previous.p4.copy()
            if self.previous is not None
            else Vector2(0, -width_start / 2)
        )

        if self.turn_direction != TurnDirection.Straight:
            _from = self.p1 if self.turn_direction == TurnDirection.Right else self.p2
            _to = self.p2 if self.turn_direction == TurnDirection.Right else self.p1
            vec = _to - _from
            vec_rot = _from + vec.rotate_rad(self.turn_angle)# * 180.0 / math.pi)

            if self.turn_direction == TurnDirection.Right:
                self.p2 = vec_rot
            else:
                self.p1 = vec_rot

        direction_in_rad = self.turn_angle + (
            self.previous.segment_angle if self.previous is not None else 0.0
        )
        direction_with_length = create_vector2_from_rad(direction_in_rad) * self.length
        # direction_with_length = create_vector2_from_rad(direction_in_rad).scale(self.length)

        self.p3 = self.p1 + direction_with_length
        self.p4 = self.p2 + direction_with_length

        # Move the end points towards each other if the segment gets more narrow towards its end and vice versa
        if not math.isclose(width_start, self.width_end):
            # if width_start != self.width_end:
            p3_to_p4 = self.p4 - self.p3
            distance_p3_to_p4 = p3_to_p4.magnitude()
            inset_amount = (distance_p3_to_p4 - self.width_end) * 0.5
            inset_direction = p3_to_p4.normalize()
            self.p3 += inset_direction * inset_amount
            self.p4 -= inset_direction * inset_amount

        self.update_segment_lines()
        self.update_centers()

        if self.previous is not None and self.turn_direction != TurnDirection.Straight:
            return EdgeSegment(self.previous, self, self.turn_direction)

        return None
