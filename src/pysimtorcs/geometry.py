import math
from pygame.math import Vector2
import numpy as np

class LineSegment:
    def __init__(self, from_point: Vector2, to_point: Vector2):
        self.from_point: Vector2 = from_point
        self.to_point: Vector2 = to_point

    def intersects_vector(self, other: "LineSegment") -> Vector2 | None:
        """
        Efficient intersection of two line segments (2D) using pygame.Vector2.
        Returns the intersection point as Vector2 if it exists, otherwise None.
        """
        p = self.from_point
        r = self.to_point - self.from_point
        q = other.from_point
        s = other.to_point - other.from_point

        r_cross_s = r.cross(s)
        q_minus_p = q - p

        if r_cross_s == 0:
            # Lines are parallel or collinear
            return None

        t = q_minus_p.cross(s) / r_cross_s
        u = q_minus_p.cross(r) / r_cross_s

        if 0 <= t <= 1 and 0 <= u <= 1:
            intersection = p + t * r
            return intersection

        return None
    
    # def intersects(self, other_from: Vector2, other_to:np.ndarray) -> Vector2 | None:
    def intersects(self, other_from: Vector2, other_to:Vector2) -> Vector2 | None:
        """
        Effizienter Schnittpunkt zweier Liniensegmente (2D).
        Gibt den Schnittpunkt als Vector2 zurück, falls vorhanden, sonst None.
        Arbeitet direkt auf den x- und y-Werten.
        """
        x1, y1 = self.from_point.x, self.from_point.y
        x2, y2 = self.to_point.x, self.to_point.y
        x3, y3 = other_from.x, other_from.y
        # x4, y4 = other_to[0], other_to[1]
        x4, y4 = other_to.x, other_to.y

        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x4 - x3
        dy2 = y4 - y3

        denom = dx1 * dy2 - dy1 * dx2
        if denom == 0:
            # Parallel oder kollinear
            return None

        dx3 = x3 - x1
        dy3 = y3 - y1

        t = (dx3 * dy2 - dy3 * dx2) / denom
        u = (dx3 * dy1 - dy3 * dx1) / denom

        if 0 <= t <= 1 and 0 <= u <= 1:
            ix = x1 + t * dx1
            iy = y1 + t * dy1
            return Vector2(ix, iy)

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
