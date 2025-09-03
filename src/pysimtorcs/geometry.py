import math


class Vector2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x: float = x
        self.y: float = y

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

    def sqr_magnitude(self) -> float:
        return self.x**2 + self.y**2

    def magnitude(self) -> float:
        return math.sqrt(self.sqr_magnitude())

    def normalize(self) -> "Vector2":
        mag = self.magnitude()
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

    # def intersects(self, other: 'LineSegment') -> bool:
    #     line1 = self.from_point.to_point(), self.to_point.to_point()
    #     line2 = other.from_point.to_point(), other.to_point.to_point()
    #     return LineString(line1).intersects(LineString(line2))

    def intersects(self, other: "LineSegment") -> Vector2:
        qp = other.from_point - self.from_point
        qpr = qp.cross(self.to_point - self.from_point)
        rs = (self.to_point - self.from_point).cross(other.to_point - other.from_point)

        if rs == 0.0 and qpr == 0.0:
            # Collinear case
            return None

        if rs == 0.0 and qpr != 0.0:
            # Parallel case
            return None

        if rs != 0.0:
            # Intersecting case
            u = qpr / rs

            if u < 0.0 or u > 1.0:
                return None

            qps = qp.cross(other.to_point - other.from_point)
            t = qps / rs

            if t < 0.0 or t > 1.0:
                return None

            return self.from_point + (self.to_point - self.from_point) * t

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


def delta(q: Vector2, r: Vector2, s: Vector2) -> float:
    return (r.x - q.x) * (s.y - q.y) - (r.y - q.y) * (s.x - q.x)


def sign(val: float) -> int:
    if val > 0:
        return 1
    elif val < 0:
        return -1
    return 0


def right_cross(q: Vector2, r_: Vector2, s_: Vector2) -> int:
    r = s_ if r_.y > s_.y else r_
    s = r_ if r_.y > s_.y else s_

    if q.y <= r.y or q.y > s.y:
        return 1

    d = delta(q, r, s)
    return sign(d)


def point_within_polygon(point: Vector2, polygon: list[Vector2]) -> bool:
    sign_val = -1

    for i in range(len(polygon) - 1):
        sign_val *= right_cross(point, polygon[i], polygon[i + 1])
        if sign_val == 0:
            break

    return sign_val >= 0
