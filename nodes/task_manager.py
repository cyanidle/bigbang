#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from concurrent.futures import thread
from dataclasses import dataclass
import dataclasses
from enum import Enum, IntEnum
from pathlib import Path
from threading import Thread
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple, Union
from dataclasses import field

from pydantic import BaseModel
import yaml
from py_lib.costmap import Position
from py_lib.script_types import ScriptName
from py_lib.task_node import ServiceCall, TaskNode, TasksGraph, TasksSettings
from py_lib.tesk_manager_states import ChangeState, ManagerState, State
import roslib
roslib.load_manifest("bigbang_eurobot")
from py_lib.node_base import NodeBase, NodeBaseSettings
from py_lib.rosparams_dataclass import RosparamsDataclass
from bigbang_eurobot.msg import ArduinoStatus, Measure2d
from bigbang_eurobot.srv import ExecuteScriptRequest, ExecuteScript, ExecuteScriptResponse
from std_msgs.msg import Int16, Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import rospy

@dataclass
class ManagerTopics(RosparamsDataclass):
    frame_id: str = "costmap"
    child_frame_id: str = "task_manager"
    start_controls_sub: str = "task_manager"
    arduino_status_sub: str = "arduino/status"
    position_sub: str = "filtered_pos"

    balls_reset: str = "scripts/reset_balls"
    balls_sub: str = "arduino/balls"

    arduino_enable_pub: str = "arduino/enable"
    cancel_call: str = "scripts/cancel_current"
    score_pub: str = "lcd_show"
    reload_route_serv: str = "task_manager/reload_route"


@dataclass
class TimeSettings(RosparamsDataclass):
    return_home: float = 92
    round: float = 98

@dataclass
class TaskManagerSettings(RosparamsDataclass):
    tick_rate: float = 15
    topics: ManagerTopics = ManagerTopics()
    file: str = (Path.home()/"bigbang/route.yaml").absolute().as_posix()
    tasks: TasksSettings = TasksSettings()
    time: TimeSettings = TimeSettings()

class TaskManager(NodeBase):
    def __init__(self, settings: NodeBaseSettings) -> None:
        super().__init__(settings)
        self.state = ManagerState()
        self.state.register_callbacks({
                State.ACTIVE: self.handle_activate,
                State.PREPARING: self.handle_prepare,
                State.GO_HOME: self.handle_go_home,
                State.FULL_TIME_OUT: self.handle_full_timeout,
                State.OUT_OF_TASKS: self.handle_no_tasks
            })
        self.params = TaskManagerSettings()
        self._last_pin_was = True
        self._balls = 0
        self._triggered_for_time = 0
        self.round_time = 0
        self._score = 0
        self.round_active = False
        self.update_configs()
        self.start_tick()
    @property
    def score(self): return self._score
    @score.setter
    def score(self, new):
        if new != self._score:
            rospy.logwarn(f"New score: {new}")
            self._score = new
            self.score_pub.publish(self._score)

    def _arduino_status_cb(self, msg: ArduinoStatus):
        if self._last_pin_was != msg.pin_present:
            if self.state.current != State.JUST_STARTED or not self._last_pin_was:
                if msg.pin_present: self.state.insert_pin()
                else: self.state.remove_pin()
        self._last_pin_was = msg.pin_present

    def reload_route(self):
        with open(self.params.file, "r") as f:
            raw = yaml.load(f, yaml.Loader)
            self.graph = TasksGraph.parse_obj(raw["graph"])
        for t in self.graph.tasks.values():
            t.set_cancel_handler(self.call_canceler)
        self.graph.finish.set_cancel_handler(self.call_canceler)
        self.graph.start.set_cancel_handler(self.call_canceler)

    def handle_no_tasks(self):
        cancelled = self.graph.all_cancelled()
        cancelled = list(filter(lambda t: t.redoable, cancelled))
        rospy.logwarn(f"[TASKS] Tasks done")
        if not cancelled:
            rospy.logwarn(f"[TASKS] None cancelled and redoable. Going home")
            self.state.try_change(ChangeState.GO_HOME)
        else:
            rospy.logwarn(f"[TASKS] Will try to redo cancelled")
            for t in cancelled:
                t.reset()
        found = self.find_next_task()
        if found is None:
            self.state.try_change(ChangeState.GO_HOME)
            return
        self.task = found
        self.task.start()
        self.state.redo_cancelled()

    def handle_go_home(self):
        if not self.task.was_done and not self.task.cancelled:
            self.task.cancel()
        self.go_home = self.graph.finish
        self.go_home.start()

    def handle_activate(self):
        self.round_active = True
        found = self.find_next_task()
        assert found
        self.task = found
        self.task.start()

    def handle_prepare(self):
        self.round_time = 0
        self._triggered_for_time = 0
        self.score = 0
        self._balls = 0
        try: self.reset_balls()
        except Exception as e: rospy.logwarn(f"Error while resettings balls: {e}")
        self.enable_arduino(True)
        self.reload_route()
        self.task = self.graph.start
        self.task.start()
        self.task.wait_finished()
        self.score += self.task.calculate_reward()

    def handle_full_timeout(self):
        rospy.logwarn("Round finished!")
        self.call_canceler()

    def enable_arduino(self, state: bool):
        try: self.arduino_enable_pub.publish(state)
        except Exception as e: rospy.logwarn(f"Error while enabling/disabling arduino: {e}")
        
    def update_configs(self, param: Optional[str] = None):
        self.params.update(param)
        self.set_tick_rate(self.params.tick_rate)
        self.arduino_enable_pub = rospy.Publisher(self.params.topics.arduino_enable_pub, Bool, queue_size=20)
        self.score_pub = rospy.Publisher(self.params.topics.score_pub, Int16, queue_size=20)
        def _balls_cb(msg: Int16):
            if self._balls == msg.data: return
            self.score -= self._balls
            self._balls = msg.data
            self.score += self._balls
        self.balls_sub = rospy.Subscriber(self.params.topics.balls_sub, Int16, _balls_cb)
        def _controls_cb(msg: Int16): self.state.try_change(ChangeState(msg.data))
        self.start_sub = rospy.Subscriber(self.params.topics.start_controls_sub, Int16, _controls_cb)
        self.arduino_status_sub = rospy.Subscriber(self.params.topics.arduino_status_sub, ArduinoStatus, self._arduino_status_cb)
        self.call_canceler = rospy.ServiceProxy(self.params.topics.cancel_call, Empty, persistent=True)
        def _reload(req):
            rospy.logwarn(f"[TASKS] Reloading route!")
            self.reload_route()
            return EmptyResponse()
        self.route_reloader = rospy.Service(self.params.topics.reload_route_serv, Empty, _reload)
        self.position = Position(0, 0)
        self.reset_balls = rospy.ServiceProxy(self.params.topics.balls_reset, ExecuteScript)
        def _pos_cb(msg: Measure2d):
            self.position.x = msg.x # type: ignore
            self.position.y = msg.y # type: ignore
            self.position.theta = msg.theta
        self.pos_sub = rospy.Subscriber(self.params.topics.position_sub, Measure2d, _pos_cb)
        self.reload_route()
            
    @property
    def active(self): return self.state.current == State.ACTIVE
    @property
    def going_home(self): return self.state.current == State.GO_HOME
    @property
    def preparing(self): return self.state.current == State.PREPARING

    def find_next_task(self) -> Optional[TaskNode]:
        rospy.loginfo(f"Task done! Reward: {self.task.reward}. PrioNext: {self.task.priority_next}")
        def _try_get_prio():
            if self.task.priority_next is not None:
                prio = self.graph.tasks.get(self.task.priority_next)
                if prio is None: 
                    rospy.logerr(f"Priority next task({self.task.priority_next}) does not exist!")
                    return None
                if prio.was_done or prio.cancelled:
                    return None   
                return prio
        prio = _try_get_prio()
        def _check_reqs(task: TaskNode):
            if task.required is None: return True
            found = self.graph.tasks.get(task.required)
            if found is None:
                rospy.logwarn(f"[TASKS] Required task not found --> {task.required}")
                return True
            return not found.cancelled and found.was_done
        if prio is not None:
            return prio
        possible: List[TaskNode] = []
        possible_names: Dict[str, TaskNode] = {}
        def _get_name(t: TaskNode):
            for k, v in possible_names.items():
                if v is t: return k
            raise RuntimeError("Unreachable")
        for name, task in self.graph.tasks.items():
            if task is self.task: continue
            if not task.cancelled and not task.was_done and _check_reqs(task):
                possible.append(task)
                possible_names[name] = task
        if not possible:
            self.state.out_of_tasks()
            return None
        rospy.logwarn(f"[TASKS] Choosing task out of possible: {len(possible)}")
        best = min(possible, key=lambda x: x.get_cost_to(self.position, self.params.tasks))
        rospy.logwarn(f"[TASKS] Best task: {_get_name(best)}: cost: {best.get_cost_to(self.position, self.params.tasks)}")
        return best

    def tick(self):
        rospy.loginfo_throttle(2, f"[TASKS] Round time: {self.round_time:.2f}")
        if self.round_active:
            self.round_time += self.tick_interval
        self.handle_times()
        if self.active: self.tick_active()
        elif self.going_home: self.tick_go_home()
        
    def handle_times(self):
        for time, actions in self.graph.timed.items():
            if self.round_time > time and time > self._triggered_for_time:
                self._triggered_for_time = self.round_time
                for action in actions:
                    c = action.as_call()
                    c.start()
                    self.score += action.reward

        if self.round_time >= self.params.time.round:
            self.state.try_change(ChangeState.ROUND_OVER)
            self.enable_arduino(False)
            self.round_active = False
        elif self.round_time > self.params.time.return_home:
            self.state.try_change(ChangeState.GO_HOME)

    def tick_go_home(self):
        if not self.go_home.proceed():
            if self.go_home.cancelled:
                self.go_home.reset()
                self.go_home.start()
                return
            self.score += self.go_home.calculate_reward()
            rospy.logwarn("[TASKS] Go home done! Round over")
            self.state.try_change(ChangeState.ROUND_OVER)


    def tick_active(self):
        if not self.task.proceed():
            self.score += self.task.calculate_reward()
            new_task = self.find_next_task()
            if new_task is None:
                self.state.out_of_tasks()
                return
            self.task = new_task
            self.task.start()

def main():
    node_name = "task_manager"
    rospy.init_node(node_name)
    manager = TaskManager(NodeBaseSettings(
        node_name=node_name
    ))
    rospy.spin()

if __name__ == "__main__":
    main()