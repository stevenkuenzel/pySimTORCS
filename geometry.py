from shapely.geometry import Point, Polygon

# pt : Point = Point(1.1, 0.1) 

# print(pt)

# poly : Polygon = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])

# print(pt.within(poly))



import math
from shapely.geometry import LineString



class Vector2:
    def __init__(self, x : float = 0.0, y : float = 0.0):
        self.x : float = x
        self.y : float = y

    def __add__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x, self.y + other.y)

        raise TypeError(f"Unsupported operand type(s) for +: 'Vector2' and '{type(other).__name__}'")

    def __sub__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x - other.x, self.y - other.y)
        raise TypeError(f"Unsupported operand type(s) for -: 'Vector2' and '{type(other).__name__}'")
    
    def __mul__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x * other.x, self.y * other.y)
        elif isinstance(other, (int, float)):
            return Vector2(self.x * other, self.y * other)
        raise TypeError(f"Unsupported operand type(s) for *: 'Vector2' and '{type(other).__name__}'")

    def __truediv__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x / other.x, self.y / other.y)
        elif isinstance(other, (int, float)):
            return Vector2(self.x / other, self.y / other)
        raise TypeError(f"Unsupported operand type(s) for /: 'Vector2' and '{type(other).__name__}'")

    def cross(self, other : "Vector2") -> float:
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
            y=self.x * sin_angle + self.y * cos_angle
        )

    def sqr_magnitude(self) -> float:
        return self.x ** 2 + self.y ** 2

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

    def to_point(self) -> Point:
        return Point(self.x, self.y)

class LineSegment:
    def __init__(self, from_point : Vector2, to_point : Vector2):
        self.from_point : Vector2 = from_point
        self.to_point : Vector2 = to_point

    # def intersects(self, other: 'LineSegment') -> bool:
    #     line1 = self.from_point.to_point(), self.to_point.to_point()
    #     line2 = other.from_point.to_point(), other.to_point.to_point()
    #     return LineString(line1).intersects(LineString(line2))
    
    def intersects(self, other: 'LineSegment'):
        line1 = LineString([self.from_point.to_point(), self.to_point.to_point()])
        line2 = LineString([other.from_point.to_point(), other.to_point.to_point()])
        intersection = line1.intersection(line2)

        if intersection.is_empty:
            return None
        
        if intersection.geom_type == 'Point':
            return Vector2(intersection.x, intersection.y)
        
        # If the intersection is a LineString (overlapping segments), return the midpoint
        if intersection.geom_type == 'LineString':
            coords = list(intersection.coords)
            mid_idx = len(coords) // 2
            mid = coords[mid_idx]
            return Vector2(mid[0], mid[1])
        
        return None

def create_vector2_from_rad(angle_in_rad : float) -> Vector2:
    return Vector2(x = math.cos(angle_in_rad), y = math.sin(angle_in_rad))

def point_within_polygon(point: Point, polygon: Polygon) -> bool:
    return point.within(polygon)

# def adjacent_point_on_segment(p: Vector2, line: LineSegment) -> Vector2:
#     a = p.x - line.from_point.x
#     b = p.y - line.from_point.y
#     c = line.to_point.x - line.from_point.x
#     d = line.to_point.y - line.from_point.y

#     dot = a * c + b * d
#     length_squared = c * c + d * d

#     param = dot / length_squared if length_squared > 0.0 else -1.0

#     if param < 0.0:
#         return Vector2(line.from_point.x, line.from_point.y)
#     elif param > 1.0:
#         return Vector2(line.to_point.x, line.to_point.y)
#     else:
#         return Vector2(line.from_point.x + param * c, line.from_point.y + param * d)
    

# Shapely provides the 'interpolate' method on LineString to find a point at a given distance along the line.
# To find the closest point on a line to a given point, use 'LineString.interpolate(LineString.project(Point))'.
def adjacent_point_on_segment(p: Vector2, line: LineSegment) -> Vector2:
    line_string = LineString([line.from_point.to_point(), line.to_point.to_point()])
    point = p.to_point()
    projected_dist = line_string.project(point)
    closest_point = line_string.interpolate(projected_dist)
    return Vector2(closest_point.x, closest_point.y)


a = Vector2(1, 2)
b = Vector2(3, 4)
c = a + b
print(c)