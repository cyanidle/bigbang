from copy import copy
from math import cos, sin #Питоне момент
from dataclasses import dataclass
from typing import Dict, List, Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from py_lib.arduino_params import OdomCovariance, MotorParams, StartPosition, MoveCovariance
from py_lib.rosparams_dataclass import RosparamsDataclass
from tf.transformations import quaternion_from_euler
import rospy
from bigbang_eurobot.msg import MotorInfo
from bigbang_eurobot.msg import Move2d

@dataclass
class Motor:
    params: MotorParams
    ddist: float = 0

@dataclass
class OdomSettings(RosparamsDataclass):
    theta_coeff: float = 1
    x_coeff: float = 1
    y_coeff: float = 1
    base_radius: float = 0.15
    max_delta_secs: float = 1
    covariance: OdomCovariance = OdomCovariance()
    move_covariance: MoveCovariance = MoveCovariance()
    move_source_id: int = 0

class OdomImpl:
    def __init__(self, frame_id: str, motors_params: Tuple[MotorParams, ...], settings:OdomSettings, start_pos:StartPosition):
        self._odom = Odometry()
        self._odom.header.frame_id = frame_id
        self._motors:Dict[int, Motor] = {}
        self.settings = settings
        self._odom.pose.covariance = self.settings.covariance.odometry_pos
        self._odom.twist.covariance = self.settings.covariance.odometry_twist
        for motor_params in motors_params:
            self._motors[motor_params.num] = Motor(motor_params)
        self.start_pos = start_pos
        self._last_x = self.start_pos.x
        self._last_y = self.start_pos.y
        self.pos_theta = self.start_pos.z
        self._last_update = rospy.Time.now()
        self._move = Move2d(
            0,
            0,
            0,
            self.settings.move_covariance.matrix,
            self.settings.move_source_id
        )

    def receive(self, info: MotorInfo) -> bool:
        motor = self._motors.get(info.num)
        if motor is None:
            rospy.logerr(f"Nonexistent motor received: {info.num}")
            return False
        motor.ddist = info.ddist
        if motor is None:
            rospy.logerr(f"Incorrect Motor index received: {info.num}")
        if info.num == len(self._motors) - 1:
            self._update()
            return True
        return False

    def _update_odom(self, delta_secs: float):
        self.reset_move()
        if delta_secs < self.settings.max_delta_secs:
            self._move.dtheta = 0
            for motor in self._motors.values():
                self._move.dtheta += motor.ddist/len(self._motors)/self.settings.base_radius*self.settings.theta_coeff
            self.pos_theta += self._move.dtheta
            self.twist.angular.z = self._move.dtheta / delta_secs

            self._move.dx = self._move.dy = 0
            for motor in self._motors.values():
                self.pose.x += motor.ddist * cos(self.pos_theta + motor.params.radians) / len(self._motors) * 2 * self.settings.x_coeff
                self._move.dx += motor.ddist * cos(motor.params.radians) / len(self._motors) * 2 * self.settings.x_coeff

                self.pose.y += motor.ddist * sin(self.pos_theta + motor.params.radians) / len(self._motors) * 2 * self.settings.y_coeff
                self._move.dy += motor.ddist * sin(motor.params.radians) / len(self._motors) * 2 * self.settings.y_coeff
            self.twist.linear.x = (self.pose.x - self._last_x) / delta_secs
            self.twist.linear.y = (self.pose.y - self._last_y) / delta_secs
            self._last_x = self.pose.x
            self._last_y = self.pose.y
            
        else:
            rospy.logwarn(f"Delta Seconds is too big while updating Odom: {delta_secs:.2f}!")

    def _update(self):
        current_time = rospy.Time.now()
        self._update_odom((current_time - self._last_update).to_sec())
        self._odom.header.stamp = rospy.Time.now()
        self._odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.pos_theta))
        self._last_update = current_time

    def correct(self, x:float, y:float, theta:float):
        self.reset_move()
        self.pose.x = self._last_x = x
        self.pose.y = self._last_y = y
        self.pos_theta = theta

    @property
    def pose(self):
        return self._odom.pose.pose.position
    @property
    def twist(self):
        return self._odom.twist.twist
    @property
    def as_msg(self) -> Odometry:
        return self._odom
    def reset_move(self):
        self._move.dtheta = self._move.dx = self._move.dy = 0 
    @property
    def as_move_msg(self):
        return self._move