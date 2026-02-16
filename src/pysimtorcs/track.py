import math
import os
import xml.etree.ElementTree as ET

from pygame import Vector2

from pysimtorcs.grid import Grid
from pysimtorcs.segments import (
    ConnectingSegment,
    CoordinateSegment,
    EdgeSegment,
    Segment,
    Turn,
    TurnDirection,
)


class Track:
    def __init__(self, name: str = "", level_of_detail: int = 1):
        self.name = name
        self.level_of_detail = level_of_detail
        self.start_width = 13.0

        self.min_turn_rad = 0.1 / level_of_detail

        self.segments: list[Segment] = []
        self.prev_segment: CoordinateSegment = None
        self.next_segment_id: int = 0

        self.starting_point: Vector2 = Vector2(0, self.start_width / 2)
        self.starting_angle: float = 0

        self.turns: list[Turn] = []

        self.length: float = 0
        self.length_straight: float = 0

        self.x_min: float = float("inf")
        self.x_max: float = float("-inf")
        self.y_min: float = float("inf")
        self.y_max: float = float("-inf")

        self.grid: Grid = None

    def __next_segment_id(self) -> int:
        id: int = self.next_segment_id
        self.next_segment_id += 1
        return id

    def add_segment(self, length: float, width: float, angle: float):
        new_segment: CoordinateSegment = CoordinateSegment(
            length, width, angle, self.prev_segment
        )
        edge_segment: EdgeSegment | None = new_segment.initialize(
            self.prev_segment.width_end
            if self.prev_segment is not None
            else self.start_width
        )

        if edge_segment is not None:
            edge_segment.id = self.__next_segment_id()
            self.determine_min_max(edge_segment)
            self.segments.append(edge_segment)

        new_segment.id = self.__next_segment_id()
        self.determine_min_max(new_segment)
        self.segments.append(new_segment)

        self.prev_segment = new_segment

    def add_turn(self, angle_in_rad: float, radius_start: float, radius_end: float):
        angle_half = angle_in_rad / 2
        segment1_length: float = radius_start * abs(angle_half)
        segment2_length: float = radius_end * abs(angle_half)

        # First half.
        for _ in range(self.level_of_detail):
            self.add_segment(
                segment1_length / self.level_of_detail,
                self.start_width,
                angle_half / self.level_of_detail,
            )

        # Second half.
        for _ in range(self.level_of_detail):
            self.add_segment(
                segment2_length / self.level_of_detail,
                self.start_width,
                angle_half / self.level_of_detail,
            )

    def finalize(self):
        first_segment: Segment = self.segments[0]
        last_segment: Segment = self.segments[-1]

        connecting_segment: ConnectingSegment = ConnectingSegment(
            last_segment, first_segment
        )
        connecting_segment.id = self.__next_segment_id()
        self.segments.append(connecting_segment)

        self.starting_point = (
            first_segment.center_start
            + first_segment.segment_direction * (0.1 * first_segment.length_measured)
        )
        self.starting_angle = first_segment.segment_angle

        self.__find_turn_segments()
        # TODO findMaxTurnSpeeds()

        self.grid: Grid = Grid(self.x_max - self.x_min, self.y_max - self.y_min, 10)

        for segment in self.segments:
            self.grid.add_segment(segment)
            segment.length_track_total = self.length
            self.length += segment.length_measured

            if segment.in_turn is None:
                self.length_straight += segment.length_measured

    def __find_turn_segments(self):
        current_turn: Turn | None = None
        next_turn_id: int = 0

        for segment in self.segments:
            if isinstance(segment, CoordinateSegment):
                # A segment is only considered as a turn, if its arc exceeds a certain threshold (depends on level of detail) and its length is shorter than MAX_TURN_SEGMENT_LENGTH_SQR.
                if (
                    segment.turn_direction != TurnDirection.Straight
                    and abs(segment.turn_angle) >= self.min_turn_rad
                    and segment.length_measured <= 50
                ):  # <-- length measured is not sqr!
                    #    segment.length_measured <= Turn.MAX_TURN_SEGMENT_LENGTH_SQR:
                    if current_turn is None:
                        # Start a new turn.
                        current_turn = Turn(next_turn_id, segment)
                        next_turn_id += 1
                    elif current_turn.direction != segment.turn_direction:
                        # End the current turn. Start a new one.
                        current_turn.finalize()
                        self.turns.append(current_turn)
                        current_turn = Turn(next_turn_id, segment)
                        next_turn_id += 1
                    else:
                        # Add the segment to the current turn.
                        current_turn.add_segment(segment)
                elif current_turn is not None:
                    # End the current turn.
                    current_turn.finalize()
                    self.turns.append(current_turn)
                    current_turn = None
            elif isinstance(segment, EdgeSegment) and current_turn is not None:
                # Add a connecting edge segment to the current turn.
                current_turn.add_segment(segment)

        # Finalize the current turn if it exists.
        if current_turn is not None:
            current_turn.finalize()
            self.turns.append(current_turn)

        # Assign a reference to the next turn to each segment if the track.
        for i, segment in enumerate(self.segments):
            if segment.in_turn is not None:
                continue

            for j_ in range(i + 1, len(self.segments) + i + 1):
                j_mod = j_ % len(self.segments)
                if self.segments[j_mod].in_turn is not None:
                    segment.next_turn = self.segments[j_mod].in_turn
                    break

    def determine_min_max(self, segment: Segment):
        bbox = segment.get_bbox()
        # segment.determine_min_max()

        self.x_min = min(self.x_min, bbox[0])
        self.x_max = max(self.x_max, bbox[2])
        self.y_min = min(self.y_min, bbox[1])
        self.y_max = max(self.y_max, bbox[3])

    def get_segments_normalized(self):
        if not self.segments or self.length == 0:
            return []
        return [segment.length_measured / self.length for segment in self.segments]


def import_from_torcs(name : str, level_of_detail: int = 1) -> Track:
    invert: bool = name.startswith("!")
    if invert:
        name = name[1:]

    track: Track = Track(name, level_of_detail)

    file_path = os.path.join("input", "tracks", f"{name}.xml")
    tree = ET.parse(file_path)
    root = tree.getroot()

    x_main_track = None
    x_track_segments = None

    # Find the main section
    for section in root.findall("section"):
        if section.attrib.get("name") == "Main Track":
            x_main_track = section
            break

    track_width = 10.0
    # Determine the track width
    for attnum in x_main_track.findall("attnum"):
        if attnum.attrib.get("name") == "width":
            track_width = float(attnum.attrib.get("val"))
            break

    track.start_width = track_width

    # Find the track segment section
    for section in x_main_track.findall("section"):
        if section.attrib.get("name") == "Track Segments":
            x_track_segments = section
            break

    # Iterate over all segments
    for x_track_segment in x_track_segments.findall("section"):
        seg_type = None
        map_vals = {}

        # Get segment type
        for attstr in x_track_segment.findall("attstr"):
            if attstr.attrib.get("name") == "type":
                seg_type = attstr.attrib.get("val")
                map_vals["type"] = seg_type
                break

        # Get numeric properties
        for attnum in x_track_segment.findall("attnum"):
            name = attnum.attrib.get("name")
            val = attnum.attrib.get("val")
            if name in ["lg", "arc", "radius", "end radius"]:
                map_vals[name] = val

        if map_vals["type"] == "str":
            length = float(map_vals["lg"])
            track.add_segment(length, track_width, 0.0)
        else:
            arc = float(map_vals["arc"])
            left_turn = map_vals["type"] == "lft"
            radius_start = float(map_vals["radius"])
            radius_end = float(map_vals.get("end radius", radius_start))
            angle_in_rad = (
                arc
                * (math.pi / 180.0)
                * (-1.0 if left_turn else 1.0)
                * (-1.0 if invert else 1.0)
            )
            track.add_turn(angle_in_rad, radius_start, radius_end)

    track.finalize()
    return track
