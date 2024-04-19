from ctypes import pointer
from dataclasses import dataclass
import math
from nav_msgs.msg import OccupancyGrid
from typing import Generic, Tuple, TypeVar, Union
from math import atan2, cos, degrees, pi, sin

_T = TypeVar("_T", bound=Union[int, float])

class Coord(Generic[_T]):
    __slots__ = ["x", "y"]
    def __init__(self, x: _T = 0, y: _T = 0) -> None:
        self.x = x
        self.y = y
    def __repr__(self) -> str:
        return f"Coord: x:{self.x:.3f}; y:{self.y:.3f}"
    def dist_to(self, other: "Coord[_T]") -> _T:
        return (other - self).norm()
    def angle_from_x_axis_to(self, other: "Coord[_T]") -> _T:
        dx = other.x - self.x
        dy = other.y - self.y
        return atan2(dy, dx)
    def angle_from_x_axis(self) -> float:
        return atan2(self.y, self.x)
    def rotated_by(self, theta: _T) -> "Coord[_T]":
        rotor = cos(theta) + 1j*sin(theta)
        complex_result = rotor * complex(self.x, self.y)
        return Coord(complex_result.real , complex_result.imag)
    def norm(self) -> _T:
        return (self.x * self.x + self.y * self.y) ** 0.5
    def normalized(self) -> "Coord[_T]":
        norm = self.norm()
        return Coord(self.x/norm, self.y/norm)
    def rounded(self) -> "Coord[_T]":
        return Coord(round(self.x), round(self.y))
    def __sub__(self, other: "Coord[_T]"):
        return Coord(self.x - other.x, self.y - other.y)
    def __add__(self, other: "Coord[_T]"):
        return Coord(self.x + other.x, self.y + other.y)
    def __mul__(self, other: _T):
        return Coord(self.x * other, self.y * other)
    def __rmul__(self, other: _T):
        return self * other
    def __truediv__(self, other: _T):
        return Coord(self.x / other, self.y / other)
       
class Costmap:
    MAX_COST = 100
    __slots__ = ["height", "width", "resolution", "data"]
    def __init__(self, height: int = 301, width: int = 151, resolution: float = 0.02, data: Tuple[int] = tuple([0]*(301*151))) -> None:
        self.height = height
        self.width = width
        self.resolution = resolution
        self.data = data
    @staticmethod
    def from_msg(msg: OccupancyGrid):
        return Costmap(
            data = msg.data,
            height = msg.info.height,
            width = msg.info.width,
            resolution = msg.info.resolution
        )
    def update_from_msg(self, msg: OccupancyGrid):
        self.data = msg.data
        self.height = msg.info.height
        self.width = msg.info.width
        self.resolution = msg.info.resolution

    def point_valid(self, x:int, y:int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def coord_valid(self, coord: Coord[int]) -> bool:
        return self.point_valid(coord.x, coord.y)

    def at(self, x: int, y: int) -> int:
        return self.data[int(self.width * y + x)] if self.point_valid(x, y) else self.MAX_COST
    
    def at_coord(self, coord: Coord[int]) -> int:
        return self.at(coord.x, coord.y)
        
    def at_meters(self, x: float, y: float) -> int:
        real_pos = self.meters_to_pos(x, y)
        return self.at_coord(real_pos)
    
    def at_coord_meters(self, coord: Coord[float]) -> int:
        return self.at_coord(self.meters_to_pos(coord.x, coord.y))
    
    def pos_to_meters(self, coord: Coord[int]) -> Coord[float]:
        return Coord(coord.x * self.resolution, coord.y * self.resolution)

    def meters_to_pos(self, x:float, y:float) -> Coord[int]:
        return Coord(int(round(x/self.resolution)), int(round(y/self.resolution)))


def normalized_theta(theta: float):
    normalized = theta % (math.pi * 2) 
    if normalized > math.pi:
        normalized -= math.pi * 2
    elif normalized < -math.pi:
        normalized += math.pi * 2
    return normalized

class Position(Coord[_T]):
    __slots__ = ["theta"]
    def __init__(self, x: _T, y: _T, theta: float = 0) -> None:
        super().__init__(x, y)
        self.theta = theta
    def __repr__(self) -> str:
        return f"Pos: x:{self.x:.3f}; y:{self.y:.3f}; th:{self.theta:.4f}"
    @property
    def theta_norm(self):
        return normalized_theta(self.theta)

def test():
    a = Coord(5, 1)
    print(f"Start = {degrees(a.angle_from_x_axis())}")
    print("+90: ", degrees(a.rotated_by(pi/2).angle_from_x_axis()))
    print("+180: ", degrees(a.rotated_by(pi).angle_from_x_axis()))
    print("-90", degrees(a.rotated_by(-pi/2).angle_from_x_axis()))
    print(normalized_theta(42))
    print(normalized_theta(-42))

if __name__ == "__main__":
    test()