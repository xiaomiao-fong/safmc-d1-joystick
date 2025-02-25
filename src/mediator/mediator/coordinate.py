import math
from dataclasses import dataclass

from geometry_msgs.msg import Point

class staticproperty(property):
    def __get__(self, owner_self, owner_cls):
        return self.fget()


@dataclass
class Coordinate:

    x: float  # n
    y: float  # e
    z: float  # d

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self) -> str:
        return f"Coord(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

    def __neg__(self) -> 'Coordinate':
        return Coordinate(-self.x, -self.y, -self.z)

    def __add__(self, other: 'Coordinate') -> 'Coordinate':
        if not isinstance(other, Coordinate):
            raise TypeError(
                f"Unsupported operand type(s) for +: 'NEDCoordinate' and '{type(other).__name__}'")
        return Coordinate(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Coordinate') -> 'Coordinate':
        if not isinstance(other, Coordinate):
            raise TypeError(
                f"Unsupported operand type(s) for +: 'NEDCoordinate' and '{type(other).__name__}'")
        return Coordinate(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'Coordinate':
        if not isinstance(scalar, (int, float)):
            raise TypeError(
                f"Unsupported operand type(s) for *: 'NEDCoordinate' and '{type(scalar).__name__}'")
        return Coordinate(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> 'Coordinate':
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> 'Coordinate':
        if not isinstance(scalar, (int, float)):
            raise TypeError(
                f"Unsupported operand type(s) for *: 'NEDCoordinate' and '{type(scalar).__name__}'")
        if scalar == 0:
            raise ValueError("Division by zero is not allowed.")
        return Coordinate(self.x / scalar, self.y / scalar, self.z / scalar)

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    @property
    def magnitude_2d(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    @property
    def normalized(self) -> 'Coordinate':
        mag = self.magnitude
        if mag == 0:
            raise ValueError("Cannot normalize a zero vector.")
        return self / mag

    @property
    def normalized_2d(self) -> 'Coordinate':
        mag = self.magnitude_2d
        if mag == 0:
            raise ValueError("Cannot normalize a zero vector.")
        return Coordinate(
            x=self.x/mag,
            y=self.y/mag,
            z=self.z,
        )

    def to_point(self) -> Point:
        return Point(
            x=self.x, y=self.y, z=self.z
        )

    @staticmethod
    def from_point(point: Point) -> 'Coordinate':
        return Coordinate(
            x=point.x,
            y=point.y,
            z=point.z,
        )

    @staticmethod
    def distance(coord1: 'Coordinate', coord2: 'Coordinate') -> float:
        return math.sqrt((coord1.x - coord2.x) ** 2 + (coord1.y - coord2.y) ** 2 + (coord1.z - coord2.z) ** 2)

    @staticmethod
    def clamp_magnitude(coord: 'Coordinate', max_magnitude: float) -> 'Coordinate':
        return coord if coord.magnitude <= max_magnitude else max_magnitude * coord.normalized

    @staticmethod
    def clamp_magnitude_2d(coord: 'Coordinate', max_magnitude: float) -> 'Coordinate':
        return coord if coord.magnitude_2d <= max_magnitude else max_magnitude * coord.normalized_2d

    @staticmethod
    def ned_to_enu(ned_coord: 'Coordinate') -> 'Coordinate':
        return Coordinate(
            x=ned_coord.y,
            y=ned_coord.x,
            z=-ned_coord.z
        )

    @staticmethod
    def enu_to_ned(enu_coord: 'Coordinate') -> 'Coordinate':
        return Coordinate(
            x=enu_coord.y,
            y=enu_coord.x,
            z=-enu_coord.z
        )

    @staticproperty
    def right() -> 'Coordinate':
        return Coordinate(1, 0, 0)

    @staticproperty
    def front() -> 'Coordinate':
        return Coordinate(0, 1, 0)

    @staticproperty
    def down() -> 'Coordinate':
        return Coordinate(0, 0, 1)
