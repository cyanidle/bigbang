from dataclasses import dataclass, field
from math import radians
from turtle import back
from typing import List

from py_lib.rosparams_dataclass import RosparamsDataclass
from bigbang_eurobot.msg import MotorParams as MotorParamsMsg


@dataclass
class ServosParams(RosparamsDataclass):
    nums: List[int] = field(default_factory=list)
    channels: List[int] = field(default_factory=list)
    speeds: List[int] = field(default_factory=list)
    minVals: List[int] = field(default_factory=list)
    maxVals: List[int] = field(default_factory=list)
    startPercents: List[int] = field(default_factory=list)


@dataclass
class StartPosition(RosparamsDataclass):
    x: float = 0
    y: float = 0
    z: float = 0


@dataclass
class ImuCovariance(RosparamsDataclass):
    orientaion: List[float] = field(default_factory=lambda: [0.]*9)
    rotation: List[float] = field(default_factory=lambda: [0.]*9)
    acceleration: List[float] = field(default_factory=lambda: [0.]*9)


@dataclass
class OdomCovariance(RosparamsDataclass):
    odometry_pos: List[float] = field(default_factory=lambda: [0.]*36)
    odometry_twist: List[float] = field(default_factory=lambda: [0.]*36)

@dataclass
class MoveCovariance(RosparamsDataclass):
    matrix: List[float] = field(default_factory=lambda: [0.2, 0, 0,
                                                             0, 0.2, 0,
                                                             0, 0, 0.2])


@dataclass
class PID(RosparamsDataclass):
    p: float = 5
    i: float = 1
    d: float = 0

@dataclass
class MotorParams(RosparamsDataclass):
    angle: int = 0
    num: int = 0
    max_speed: float = 0.5
    max_turn_speed: float = 0.2
    radius: float = 0.15
    ticks_per_rotation: int = 360
    coeff: float = 1
    pid: PID = field(default_factory=PID)
    radians: float = field(init=False)

    def __post_init__(self):
        self.radians = radians(self.angle)
        super().__post_init__()

    @property
    def as_params_msg(self) -> MotorParamsMsg:
        return MotorParamsMsg(
            num=self.num,
            radius=self.radius,
            angleDegrees=self.angle,
            interCoeff=self.pid.i,
            propCoeff=self.pid.p,
            diffCoeff=self.pid.d,
            coeff=self.coeff,
            turnMaxSpeed=self.max_turn_speed,
            maxSpeed=self.max_speed,
            ticksPerRotation=self.ticks_per_rotation
        )

