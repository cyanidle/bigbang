#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import IntEnum, auto
from functools import wraps
import roslib
roslib.load_manifest("bigbang_eurobot")
import math
from py_lib.node_base import NodeBase, NodeBaseSettings
from py_lib.rosparams_dataclass import RosparamsDataclass
from py_lib.costmap import Coord, Costmap, Position, normalized_theta
from bigbang_eurobot.msg import Measure2d, PlanerStatus, MonteCarloState
from bigbang_eurobot.srv import (DirectMove, DirectMoveRequest, DirectMoveResponse,
                                DirectDrift, DirectDriftRequest, DirectDriftResponse,
                                ExecuteScript, ExecuteScriptRequest, ExecuteScriptResponse)
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Twist, Point
from dataclasses import dataclass, field
from typing import List, Optional
from threading import Thread
import rospy

@dataclass
class LocalPlanerTopics(RosparamsDataclass):
    frame_id: str = "costmap"
    child_frame_id: str = "local_plan"
    global_path_sub: str = "global_plan"
    monte_carlo_state_sub: str = "monte_carlo_state"
    costmap_sub: str = "costmap"
    position_sub: str = "filtered_pos"
    cmd_vel_pub: str = "cmd_vel"
    status_pub: str = "planer_status"
    direct_move: str = "direct_move"
    direct_unstable: str = "scripts/direct_unstable"
    direct_drift: str = "direct_drift"
    global_target: str = "global_planer/target"

@dataclass
class DirectDriveSettings(RosparamsDataclass):
    move_max_cost: float = 40
    drift_max_cost: float = 10
    drift_step_radians_per_meter_radius: float = 0.15
    wait_time: float = 2

@dataclass 
class DrivingSettings(RosparamsDataclass):
    min_speed_coeff: float = 0.4
    min_rotation_spd: float = 0.3
    full_rot_spd_per_radians: float = 2
    enable_mid_path_rotation: bool = False
    max_radians_per_meter: float = 1
    max_speed_for_meters: float = 0.5
    direct: DirectDriveSettings = field(default_factory=DirectDriveSettings)

@dataclass 
class LocalPlanerMargins(RosparamsDataclass):
    position: float = 0.03 #m
    theta: float = 0.04 #radians

@dataclass 
class LocalPlanerPathParams(RosparamsDataclass):
    half_slow_per_cost_of: float = 20
    approximation_step_points: int = 5
    approximation_max_cost: float = 30
    fallback_min_points_count: int = 3

@dataclass
class LocalPlanerSettings(RosparamsDataclass):
    topics: LocalPlanerTopics = LocalPlanerTopics()
    path: LocalPlanerPathParams = LocalPlanerPathParams()
    margins: LocalPlanerMargins = LocalPlanerMargins()
    drive: DrivingSettings = DrivingSettings()
    tick_rate: float = 12

class PlanerStatusWrapper(PlanerStatus):
    __slots__ = ("_tick_interval", )
    def __init__(self, tick_interval: float, *args, **kwds):
        self._tick_interval = tick_interval
        super().__init__(*args, **kwds)
    def is_moving(self):
        self.driving_for += self._tick_interval
        self.idle_for = 0.
        self.is_stuck = False
        self.reached = False
        self.rotated = False
    def stuck(self):
        self.driving_for = 0.
        self.idle_for += self._tick_interval
        self.is_stuck = True
        self.reached = False
        self.rotated = False
    def is_rotating(self):
        self.driving_for += self._tick_interval
        self.idle_for = 0.
        self.reached = True
        self.rotated = False
        self.is_stuck = False
    def is_done(self):
        self.driving_for = 0.
        self.idle_for += self._tick_interval
        self.reached = True
        self.rotated = True
        self.is_stuck = False
    def pause(self):
        if self.idle_for: self.idle_for += self._tick_interval
        else: self.driving_for += self._tick_interval
    

def direct_move_entry(responce: type):
    def _wrap_impl(func):
        @wraps(func)
        def wrapper(self: "LocalPlaner", msg):
            self.in_direct = True
            res = responce()
            try: res = func(self, msg)
            except Exception as e: rospy.logerr(f"While in direct drive: {e.__class__.__name__}: {e}")
            self.in_direct = False
            return res
        return wrapper
    return _wrap_impl

class LocalPlaner(NodeBase):
    def __init__(self, settings: NodeBaseSettings) -> None:
        super().__init__(settings)
        self.params = LocalPlanerSettings()
        self.costmap = Costmap()
        self.position = Position(0., 0.)
        self._target = Position(0., 0.)
        self.target_valid = False
        self.in_direct = False
        self.path: List[Position] = [] 
        self.monte_carlo_state = MonteCarloState()
        self.update_configs()
        self.start_tick()

    @property
    def topics(self):
        return self.params.topics
    @property
    def child_frame_id(self):
        return self.topics.child_frame_id
    def update_configs(self, param: Optional[str] = None):
        self.params.update(param)
        self.status = PlanerStatusWrapper(self.params.tick_rate)
        self.set_tick_rate(self.params.tick_rate)
        self.path_sub = rospy.Subscriber(self.topics.global_path_sub, Path, self._global_path_cb)
        self.costmap_sub = rospy.Subscriber(self.topics.costmap_sub, OccupancyGrid, self._costmap_cb)
        self.pos_sub = rospy.Subscriber(self.topics.position_sub, Measure2d, self._position_cb)
        self.cmd_vel_pub = rospy.Publisher(self.topics.cmd_vel_pub, Twist, queue_size=20)
        self.status_pub = rospy.Publisher(self.topics.status_pub, PlanerStatus, queue_size=20)
        self.monte_state_sub = rospy.Subscriber(self.topics.monte_carlo_state_sub, MonteCarloState, self._monte_state_cb)
        self.direct_mover = rospy.Service(self.topics.direct_move, DirectMove, self._direct_move)
        self.direct_drifter = rospy.Service(self.topics.direct_drift, DirectDrift, self._direct_drift)
        self.dicter_unstable_srv = rospy.Service(self.topics.direct_unstable, ExecuteScript, self._direct_unstable)
        def _new_target(msg: Point): 
            self.status.idle_for = 0
            self.status_pub.publish(self.status)
        self.global_target_sub = rospy.Subscriber(self.topics.global_target, Point, _new_target)

    @direct_move_entry(responce=ExecuteScriptResponse)
    def _direct_unstable(self, req: ExecuteScriptRequest):
        time = float(req.args[2])
        dir = Coord(float(req.args[0]), float(req.args[1])).normalized() * 0.4
        msg = Twist()
        msg.linear.x = dir.x
        msg.linear.y = dir.y
        passed = 0.
        while passed < time:
            self.cmd_vel_pub.publish(msg)
            passed += self.tick_interval
            rospy.sleep(self.tick_interval)
        return ExecuteScriptResponse(status=True)

    @direct_move_entry(responce=DirectDriftResponse)
    def _direct_drift(self, msg: DirectDriftRequest) -> DirectDriftResponse:
        def _cost_ok(target: Position):
            allowed = self.params.drive.direct.drift_max_cost
            cost = self._get_line_cost(self.position, target)
            if cost > allowed:
                rospy.logerr(f"[LOCAL] Direct drift obstructed! Cost: {cost}. Allowed: {allowed}")
                return False
            return True
        center_pos = Coord(msg.center_x, msg.center_y)
        start_pos = self.position
        base = self.position - center_pos
        rotation_coeff = 1 if msg.direction == msg.left else -1
        final_target = base.rotated_by(msg.radians) * rotation_coeff 
        relative_theta = self.position.angle_from_x_axis_to(center_pos)
        def _get_target(target_pos: Coord) -> Position:
            res = Position(target_pos.x, target_pos.y)
            if msg.dont_keep_relative_theta:
                res.theta = target_pos.angle_from_x_axis_to(center_pos) + relative_theta
            else:
                res.theta = start_pos.theta
            return res
        rospy.logwarn(f"NOT IMPLEMENTED! [LOCAL] Direct DRIFT starting to target --> {final_target}")
        step = self.params.drive.direct.drift_step_radians_per_meter_radius * base.norm()
        max_count = msg.radians / step + 1
        part = msg.radians / max_count
        current_count = 0
        waited = 0.
        while not self._reached_target(final_target):
            current_rot = part * current_count
            current_count += 1
            current_target = _get_target(base.rotated_by(current_rot) + center_pos)
            if waited >= self.params.drive.direct.wait_time:
               rospy.logerr(f"[LOCAL] Direct drift obstructed for too long! Allowed: {self.params.drive.direct.wait_time}")
               self.stop()
               return DirectDriftResponse(status=False)
            if not _cost_ok(current_target):
               self.status.stuck()
               self.stop()
               rospy.sleep(self.tick_interval)
               waited += self.tick_interval
               continue
            self._target = current_target
            self.status.is_moving()
            self._drive()
            rospy.sleep(self.tick_interval)
        rospy.logwarn(f"[LOCAL] Direct DRIFT done to target --> {final_target}")
        return DirectDriftResponse(status=True)

    @direct_move_entry(responce=DirectMoveResponse)
    def _direct_move(self, msg: DirectMoveRequest) -> DirectMoveResponse:
       target = self.position + Coord(msg.x_dir, msg.y_dir).rotated_by(self.position.theta).normalized() * msg.dist
       rospy.logwarn(f"[LOCAL] Received Direct move command! Self: {self.position!r} --> Target: {target!r}")
       def _cost_ok():
           cost = self._get_line_cost(self.position, target)
           if cost > self.params.drive.direct.move_max_cost:
               rospy.logerr(f"[LOCAL] Direct move obstructed! Cost: {cost}. Allowed: {self.params.drive.direct.move_max_cost}")
               return False
           return True
       self._target = Position(target.x, target.y, self.position.theta)
       waited = 0.
       while not self.reached_target():
           if waited >= self.params.drive.direct.wait_time:
               rospy.logerr(f"[LOCAL] Direct move obstructed for too long! Allowed: {self.params.drive.direct.wait_time}")
               self.stop()
               return DirectMoveResponse(status=False)
           if not _cost_ok(): 
               self.status.stuck()
               self.stop()
               rospy.sleep(self.tick_interval)
               waited += self.tick_interval
               continue
           self._drive()
           rospy.sleep(self.tick_interval)
           self.status.is_moving()
           rospy.logwarn_throttle(0.5, f"[LOCAL] Direct moving! Target: {self.target!r}")
           self.status_pub.publish(self.status)
       rospy.logwarn(f"[LOCAL] Direct move Done! Target: {self.target!r}")
       self.stop()
       return DirectMoveResponse(status=True)

    @property
    def target(self): return self._target
    @target.setter
    def target(self, val: Position):
        if self.in_direct: raise ValueError("Attempt to set while direct move")
        self._target = val

    def _monte_state_cb(self, msg: MonteCarloState):
        self.monte_carlo_state = msg

    def _position_cb(self, msg: Measure2d):
        self.position.x = msg.x
        self.position.y = msg.y
        self.position.theta = msg.theta

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap.update_from_msg(msg)

    def _global_path_cb(self, msg: Path):
        if self.in_direct: return
        new_path = []
        for pos in msg.poses:
            new_path.append(Position(pos.pose.position.x, 
                                      pos.pose.position.y, 
                                      self.theta_from_msg(pos.pose.orientation)))
        if not new_path: 
            self.stop()
        else: 
            self.target_valid = True
            self.path = new_path
            self.target = self.find_best_target()

    def find_best_target(self) -> Position:
        points_from_end = 1
        result = self.path[-points_from_end]
        approx_cost = self._get_line_cost(self.position, result)
        while approx_cost > self.params.path.approximation_max_cost:
            points_from_end += self.params.path.approximation_step_points
            if points_from_end >= len(self.path):
                return self.path[min(self.params.path.fallback_min_points_count, len(self.path) - 1)]
            result = self.path[-points_from_end]
            approx_cost = self._get_line_cost(self.position, result)
        return result

    def stop(self):
        self.path.clear()
        self.cmd_vel_pub.publish(Twist())

    def send_status(self):
        self.status_pub.publish(self.status)

    def tick(self):
        if self.in_direct: return
        if self.monte_carlo_state.is_bad:
            self.status.pause()
            self.stop()
        elif not self.path:
            if self.reached_target():
                self.status.is_done()
            else:
                self.status.stuck()
            self.stop()
        elif not self.reached_target():
            self.status.is_moving()
            self._drive()
        elif not self.rotated():
            self.status.is_rotating()
            self._rotate()
        else:
            self.status.is_done()
            self.stop()
        self.send_status()

    def _reached_target(self, target: Coord):
        return (self.position - target).norm() <= self.params.margins.position
    def reached_target(self):
        return self._reached_target(self.target)
    def rotated(self) -> bool:
        return abs(self.target.theta_norm - self.position.theta_norm) <= self.params.margins.theta

    def _rotate(self):
        diff = self.target.theta_norm - self.position.theta_norm
        diff = normalized_theta(diff)
        cmd = Twist()
        cmd.angular.z = diff / self.params.drive.full_rot_spd_per_radians
        sign = 1 if diff > 0 else -1
        if abs(cmd.angular.z) > 1: 
            cmd.angular.z = 1 * sign
        if abs(cmd.angular.z) < self.params.drive.min_rotation_spd: 
            cmd.angular.z = sign * self.params.drive.min_rotation_spd
        self.cmd_vel_pub.publish(cmd)

    def _drive(self, override_mid_path_rot: Optional[bool] = None):
        cmd = Twist()
        dist = (self.target - self.position).norm()
        dist_coeff = dist / self.params.drive.max_speed_for_meters
        if dist_coeff > 1: dist_coeff = 1
        if dist_coeff < self.params.drive.min_speed_coeff: dist_coeff = self.params.drive.min_speed_coeff
        actual_cmd = (self.target - self.position).rotated_by(-self.position.theta).normalized() * dist_coeff
        cmd.linear.x = actual_cmd.x
        cmd.linear.y = actual_cmd.y
        allow_mid_rotation = self.params.drive.enable_mid_path_rotation
        if override_mid_path_rot is not None: allow_mid_rotation = override_mid_path_rot
        if allow_mid_rotation:
            cmd.angular.z = normalized_theta(self.target.theta_norm - self.position.theta_norm) \
                * dist / self.params.drive.max_radians_per_meter
        self.cmd_vel_pub.publish(cmd)
        self.draw_line(self.position, self.target, 0.2)

    def _get_line_cost(self, start: Coord[float], target: Coord[float]) -> float:
        d_coord = self.costmap.meters_to_pos(target.x - start.x, target.y - start.y)
        start_coord = self.costmap.meters_to_pos(start.x, start.y)
        target_coord = self.costmap.meters_to_pos(target.x, target.y)
        if abs(d_coord.x) > abs(d_coord.y):
            bigger = abs(d_coord.x)
            xcoeff = -1 if d_coord.x < 0 else 1
            ycoeff = d_coord.y / bigger
        else:
            bigger = abs(d_coord.y)
            if not bigger:
                return 0
            xcoeff = d_coord.x / bigger
            ycoeff = -1 if d_coord.y < 0 else 1
        result: float = self.costmap.at_coord(start_coord)
        for step in range(bigger):
            result += self.costmap.at(start_coord.x + round(step * xcoeff), start_coord.y + round(step * ycoeff))
        result += self.costmap.at_coord(target_coord)
        return result

def main():
    node_name = "local_planer"
    rospy.init_node(node_name)
    planer = LocalPlaner(NodeBaseSettings(
        node_name=node_name
    ))
    rospy.on_shutdown(planer.stop)
    rospy.spin()

if __name__ == "__main__":
    main()