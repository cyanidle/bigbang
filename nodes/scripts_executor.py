#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from selectors import PollSelector
from threading import Thread
from bigbang_eurobot.srv import ExecuteScript, ExecuteScriptRequest, ExecuteScriptResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from dataclasses import field
from typing import Dict, Optional

import yaml
from py_lib.node_base import NodeBase, NodeBaseSettings
from py_lib.rosparams_dataclass import RosparamsDataclass
from py_lib.script_types import *
import rospy

@dataclass
class Topics(RosparamsDataclass):
    child_frame_id: str = "scripter_node"
    cancel_name: str = "cancel_current"
    plan_cancel: str = "global_planer/cancel"
    complete_topic: str = "debug/scripts/complete"
    correction_pub: str = "initialpose"

@dataclass
class ScripterSettings(RosparamsDataclass):
    file: str = (Path.home()/"bigbang/scripts.yaml").absolute().as_posix()
    topics: Topics = field(default_factory=Topics)
    scripts: ScriptsParams = field(default_factory=ScriptsParams)
    byte_buffer_amp: float = 2
    allow_debug_complete: bool = False

class ScriptsExecutor(NodeBase):
    def __init__(self, settings: NodeBaseSettings) -> None:
        super().__init__(settings)
        self.params = ScripterSettings()
        self._current_id = 0
        self._current_script = None
        self.last_status = False
        self._correction = PoseWithCovarianceStamped()
        self.update_configs()
        self.start_tick()

    def update_configs(self, param: Optional[str] = None):
        self.params.update(param)
        ScriptsStates.init(self.params.scripts)
        self.parse_file()
        self.plan_cancel = rospy.Publisher(self.params.topics.cancel_name, EmptyMsg, queue_size=20)
        if self.params.allow_debug_complete:
            self.complete_sub = rospy.Subscriber(self.params.topics.complete_topic, EmptyMsg, self.complete_current)
        self.correction_pub = rospy.Publisher(self.params.topics.correction_pub, PoseWithCovarianceStamped, queue_size=20)
        self.create_services()

    def script(self, name: ScriptName):
        return self.scripts.get(name)

    def _add_service(self, name: str, handler: ScriptStep):
        if name in self.services:
            raise RuntimeError(f"Script name '{name}' is already taken by {self.services[name]!r}")
        rospy.logwarn(f"Creating service: scripts/{name}")
        self.services[name] = rospy.Service(f"scripts/{name}",
                                                ExecuteScript,
                                                partial(self.handle_request, handler),
                                                buff_size=round(65536 * self.params.byte_buffer_amp))

    def create_services(self):
        self.services: Dict[ScriptName, rospy.Service] = {}
        self.services[self.params.topics.cancel_name] = rospy.Service(f"scripts/{self.params.topics.cancel_name}", Empty, self.cancel_current)
        self._add_service("move", InnateMove())
        self._add_service("direct_move", InnateDirectMove())
        self._add_service("wait", InnateWait())
        self._add_service("sleep", InnateWait())
        for name, script in self.scripts.items():
            self._add_service(name, script)

    def complete_current(self, msg: EmptyMsg):
        if not self._current_script: return
        if not is_move(self._current_script): return
        self._correction.header.stamp = rospy.Time.now()
        self._correction.header.seq += 1
        self._correction.pose.pose.position.x = self._current_script.last_target.x
        self._correction.pose.pose.position.y = self._current_script.last_target.y
        self._correction.pose.pose.orientation = self.msg_from_theta(self._current_script.last_target.theta)
        self.correction_pub.publish(self._correction)

    def cancel_current(self, msg: EmptyRequest):
        while self.should_cancel:
            rospy.sleep(0.01)
        self.should_cancel = True
        self.plan_cancel.publish()
        return EmptyResponse()

    def wait_executed(self, thr: Thread, is_move: bool):
        self.should_cancel = False
        thr.start()
        while thr.is_alive():
            if self.should_cancel:
                if is_move:
                    self.plan_cancel.publish()
                self.last_status = False
                break
            rospy.sleep(0.01)
        self.should_cancel = False

    def _exec_wrapper(self, script: ScriptStep, msg: ExecuteScriptRequest, id: int):
        status = script.execute(
            ScriptData(
                self.params.scripts,
                msg,
                self.scripts
                )
            )
        if id == self._current_id:
            self.last_status = status

    def handle_request(self, script: ScriptStep, msg: ExecuteScriptRequest):
        self._current_id += 1
        self._current_script = script
        thr = Thread(target=self._exec_wrapper, args=(script, msg, self._current_id), daemon=True)
        self.wait_executed(thr, script.is_move)
        self._current_script = None
        return ExecuteScriptResponse(self.last_status)

    def parse_file(self):
        with open(self.params.file, "r") as file:
            data: List[dict] = yaml.load(file, yaml.Loader)
            intermediate = ScriptsRepr.parse_obj(data)
            self.scripts = intermediate.to_steps()

    @property
    def frame_id(self):
        return self.params.topics.child_frame_id
    @property
    def child_frame_id(self):
        return self.params.topics.child_frame_id
    def tick(self):
        pass

def main():
    node_name = "scripts_executor"
    rospy.init_node(node_name)
    executor = ScriptsExecutor(NodeBaseSettings(
        node_name=node_name
    ))
    rospy.spin()

if __name__ == "__main__":
    main()