from dataclasses import dataclass
import math


@dataclass(slots=True)
class Vector2:
    # def __init__(self, x: float = 0.0, y: float = 0.0):
    #     self.x = x
    #     self.y = y

    x: float = 0.0
    y: float = 0.0

    def __add__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x, self.y + other.y)

        raise TypeError(
            f"Unsupported operand type(s) for +: 'Vector2' and '{type(other).__name__}'"
        )

    def __sub__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x - other.x, self.y - other.y)
        raise TypeError(
            f"Unsupported operand type(s) for -: 'Vector2' and '{type(other).__name__}'"
        )

    def __mul__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x * other.x, self.y * other.y)
        elif isinstance(other, (int, float)):
            return Vector2(self.x * other, self.y * other)
        raise TypeError(
            f"Unsupported operand type(s) for *: 'Vector2' and '{type(other).__name__}'"
        )

    def __truediv__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x / other.x, self.y / other.y)
        elif isinstance(other, (int, float)):
            return Vector2(self.x / other, self.y / other)
        raise TypeError(
            f"Unsupported operand type(s) for /: 'Vector2' and '{type(other).__name__}'"
        )

    def cross(self, other: "Vector2") -> float:
        return self.x * other.y - self.y * other.x

    def scale(self, scalar: float) -> "Vector2":
        return Vector2(self.x * scalar, self.y * scalar)

    def scale_self(self, scalar: float) -> None:
        self.x *= scalar
        self.y *= scalar

    def rotate(self, angle_in_rad: float) -> "Vector2":
        cos_angle = math.cos(angle_in_rad)
        sin_angle = math.sin(angle_in_rad)
        return Vector2(
            x=self.x * cos_angle - self.y * sin_angle,
            y=self.x * sin_angle + self.y * cos_angle,
        )

    def sqr_length(self) -> float:
        return self.x**2 + self.y**2

    def length(self) -> float:
        return math.sqrt(self.sqr_length())

    def normalize(self) -> "Vector2":
        mag = self.length()
        if mag > 0:
            return Vector2(self.x / mag, self.y / mag)
        return Vector2()

    def sqr_distance(self, other: "Vector2") -> float:
        return (self.x - other.x) ** 2 + (self.y - other.y) ** 2

    def distance(self, other: "Vector2") -> float:
        return math.sqrt(self.sqr_distance(other))

    def copy(self) -> "Vector2":
        return Vector2(self.x, self.y)

    def __repr__(self):
        return f"Vector2({self.x}, {self.y})"


class LineSegment:
    def __init__(self, from_point: Vector2, to_point: Vector2):
        self.from_point: Vector2 = from_point
        self.to_point: Vector2 = to_point

    def intersects(self, other: "LineSegment") -> Vector2:
        """
        Findet den Schnittpunkt zweier Liniensegmente.

        Args:
            p1, q1 (Vector2): Start- und Endpunkt des ersten Liniensegments.
            p2, q2 (Vector2): Start- und Endpunkt des zweiten Liniensegments.

        Returns:
            Vector2: Der Schnittpunkt, falls vorhanden.
            None: Falls sich die Liniensegmente nicht schneiden.
        """
        p1 = self.from_point
        q1 = self.to_point
        p2 = other.from_point
        q2 = other.to_point

        def orientation(p: Vector2, q: Vector2, r: Vector2):
            """
            Bestimmt die Ausrichtung (Orientierung) von drei Punkten.
            Liefert 0, 1 oder 2 für kollinear, im Uhrzeigersinn oder gegen den Uhrzeigersinn.
            """
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
            if val == 0:
                return 0  # kollinear
            return 1 if val > 0 else 2  # im Uhrzeigersinn oder gegen den Uhrzeigersinn

        def on_segment(p: Vector2, q: Vector2, r: Vector2):
            """Prüft, ob Punkt q auf dem Liniensegment pr liegt."""
            return (
                q.x <= max(p.x, r.x)
                and q.x >= min(p.x, r.x)
                and q.y <= max(p.y, r.y)
                and q.y >= min(p.y, r.y)
            )

        # Orientierungen berechnen
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        # Allgemeiner Fall: Schnittpunkt, wenn sich die Orientierungen ändern
        if o1 != o2 and o3 != o4:
            # Die Vektoren für die parametrische Gleichung
            r = q1 - p1
            s = q2 - p2

            # Denominator für die Lösung des linearen Gleichungssystems
            denominator = r.x * s.y - r.y * s.x

            if denominator == 0:
                # Parallel, aber nicht kollinear. Kein Schnittpunkt.
                return None

            t = ((p2.x - p1.x) * s.y - (p2.y - p1.y) * s.x) / denominator
            u = ((p2.x - p1.x) * r.y - (p2.y - p1.y) * r.x) / denominator

            # Prüfen, ob der Schnittpunkt innerhalb der Segmente liegt
            if 0 <= t <= 1 and 0 <= u <= 1:
                # Berechnen des Schnittpunkts und Rückgabe
                intersection_point = p1 + r * t
                return intersection_point

        # Spezialfälle für kollineare Segmente
        if o1 == 0 and on_segment(p1, p2, q1):
            return p2
        if o2 == 0 and on_segment(p1, q2, q1):
            return q2
        if o3 == 0 and on_segment(p2, p1, q2):
            return p1
        if o4 == 0 and on_segment(p2, q1, q2):
            return q1

        return None


def point_within_polygon(point: Vector2, polygon: list[Vector2]):
    """
    Prüft effizient, ob ein Punkt innerhalb eines Polygons liegt,
    mithilfe des Ray-Casting-Algorithmus.

    Args:
        point (Vector2): Der zu testende Punkt.
        polygon (list[Vector2]): Eine Liste von Vector2-Objekten, die die Vertices
                                  des Polygons in der richtigen Reihenfolge darstellen.
                                  Das Polygon muss geschlossen sein (der letzte Vertex muss
                                  nicht identisch mit dem ersten sein).

    Returns:
        bool: True, falls der Punkt im Polygon ist, sonst False.
    """
    n = len(polygon)
    if n < 3:
        return False  # Ein Polygon muss mindestens 3 Vertices haben

    intersections = 0
    p = point

    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[
            (i + 1) % n
        ]  # Die nächste Kante (einschließlich der letzten zu ersten)

        # Prüfen, ob der Punkt auf einer horizontalen Kante liegt (Spezialfall)
        if p1.y == p2.y == p.y:
            if p.x >= min(p1.x, p2.x) and p.x <= max(p1.x, p2.x):
                return True

        # Prüfen, ob der Punkt auf einem Vertex liegt
        if p.x == p1.x and p.y == p1.y:
            return True

        # Ray-Casting-Logik
        # Nur Kanten betrachten, die den horizontalen Ray schneiden könnten.
        if (p1.y <= p.y < p2.y) or (p2.y <= p.y < p1.y):
            # Formel zur Berechnung des x-Schnittpunkts.
            # Ein horizontaler Ray von `point` nach rechts.
            # Hier nutzen wir die schnelle Vektorsubtraktion
            if p2.x - p1.x == 0:
                # Vertikale Kante. Schneidet nur, wenn x der Kante <= x des Punktes
                x_intersection = p2.x
            else:
                x_intersection = (p2.x - p1.x) * (p.y - p1.y) / (p2.y - p1.y) + p1.x

            # Überprüfen, ob der Schnittpunkt auf der rechten Seite des Punktes liegt
            if x_intersection > p.x:
                intersections += 1

    return intersections % 2 == 1

    # def intersects(self, other: "LineSegment"):
    #     line1 = LineString([self.from_point.to_point(), self.to_point.to_point()])
    #     line2 = LineString([other.from_point.to_point(), other.to_point.to_point()])
    #     intersection = line1.intersection(line2)

    #     if intersection.is_empty:
    #         return None

    #     if intersection.geom_type == "Point":
    #         return Vector2(intersection.x, intersection.y)

    #     # If the intersection is a LineString (overlapping segments), return the midpoint
    #     if intersection.geom_type == "LineString":
    #         coords = list(intersection.coords)
    #         mid_idx = len(coords) // 2
    #         mid = coords[mid_idx]
    #         return Vector2(mid[0], mid[1])

    #     return None


def create_vector2_from_rad(angle_in_rad: float) -> Vector2:
    return Vector2(x=math.cos(angle_in_rad), y=math.sin(angle_in_rad))


def adjacent_point_on_segment(p: Vector2, line: LineSegment) -> Vector2:
    a = p.x - line.from_point.x
    b = p.y - line.from_point.y
    c = line.to_point.x - line.from_point.x
    d = line.to_point.y - line.from_point.y

    dot = a * c + b * d
    length_squared = c * c + d * d

    param = dot / length_squared if length_squared > 0.0 else -1.0

    if param < 0.0:
        return Vector2(line.from_point.x, line.from_point.y)
    elif param > 1.0:
        return Vector2(line.to_point.x, line.to_point.y)
    else:
        return Vector2(line.from_point.x + param * c, line.from_point.y + param * d)


# # Shapely provides the 'interpolate' method on LineString to find a point at a given distance along the line.
# # To find the closest point on a line to a given point, use 'LineString.interpolate(LineString.project(Point))'.
# def adjacent_point_on_segment(p: Vector2, line: LineSegment) -> Vector2:
#     line_string = LineString([line.from_point.to_point(), line.to_point.to_point()])
#     point = p.to_point()
#     projected_dist = line_string.project(point)
#     closest_point = line_string.interpolate(projected_dist)
#     return Vector2(closest_point.x, closest_point.y)


# def delta(q: Vector2, r: Vector2, s: Vector2) -> float:
#     return (r.x - q.x) * (s.y - q.y) - (r.y - q.y) * (s.x - q.x)


# def sign(val: float) -> int:
#     if val > 0:
#         return 1
#     elif val < 0:
#         return -1
#     return 0


# def right_cross(q: Vector2, r_: Vector2, s_: Vector2) -> int:
#     r = s_ if r_.y > s_.y else r_
#     s = r_ if r_.y > s_.y else s_

#     if q.y <= r.y or q.y > s.y:
#         return 1

#     d = delta(q, r, s)
#     return sign(d)


# def point_within_polygon(point: Vector2, polygon: list[Vector2]) -> bool:
#     sign_val = -1

#     for i in range(len(polygon) - 1):
#         sign_val *= right_cross(point, polygon[i], polygon[i + 1])
#         if sign_val == 0:
#             break

#     return sign_val >= 0
