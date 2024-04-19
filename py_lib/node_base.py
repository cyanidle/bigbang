from abc import ABC, abstractmethod
import asyncio
from dataclasses import dataclass
from functools import partial
import math
import sys
from threading import Thread
import traceback
from typing import Callable, Coroutine, Optional, Tuple
from rospy import Duration, Rate, Time, get_param, init_node, get_name, Subscriber, is_shutdown, logerr, logwarn, logwarn_once, sleep, loginfo, Publisher
from bigbang_eurobot.msg import Reconfigure
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from py_lib.costmap import Coord
import debugpy

@dataclass
class NodeBaseSettings:
    node_name: str
    debug: bool = False
    allow_connect: bool = True
    base_frame: str = "costmap"

class NodeBase(ABC):
    """
    Params should be init before super().__init__() call!
    """
    def __str__(self) -> str:
        return f"Node '{self.node_name}'"
    def __repr__(self) -> str:
        return f"Node '{self.node_name}'"
    @property
    def child_frame_id(self):
        raise NotImplementedError
    @property
    def node_name(self):
        return self._node_settings.node_name
    @property
    def node_settings(self):
        return self._node_settings
    @property
    def is_debug(self):
        return self.node_settings.debug
    @property
    def tick_interval(self):
        return 1. / self.__tick_rate
    @staticmethod
    def theta_from_msg(msg: Quaternion) -> float:
        return euler_from_quaternion((msg.x, msg.y, msg.z, msg.w))[2]
    @staticmethod
    def msg_from_theta(theta: float) -> Quaternion:
        x, y, z ,w = quaternion_from_euler(0, 0, theta)
        return Quaternion(x, y, z, w)
    def set_tick_rate(self, new_val: float):
        self.__tick_rate = new_val
        self.__ticker = Rate(self.__tick_rate)
    def create_thread(self, target: Callable, *args) -> Thread:
        thr = Thread(target = target, daemon=True, args=args)
        return thr
    def create_ticker(self, target: Callable, rate: int = 20) -> Thread:
        sleeper = rospy.Rate(rate)
        def ticker():
            while True:
                try:
                    target()
                except Exception as e:
                    logerr(f"Error invoking tick() on {self.node_name}: Reason:")
                    logerr(f"Traceback: {traceback.format_exc()}")
                sleeper.sleep()
        thr = Thread(target = ticker, daemon=True)
        thr.start()
        return thr
    def tick(self):
        raise NotImplementedError
    def __tick(self):
        while True:
            try:
                self.tick()
            except Exception as e:
                logerr(f"Error invoking tick() on {self.node_name}: Reason: {e.__class__.__name__}:{e}")
                logerr(f"Traceback: {traceback.format_exc()}")
            self.__ticker.sleep()
    @abstractmethod
    def __init__(self, settings:NodeBaseSettings) -> None:
        self.set_tick_rate(20)
        self.__tick_thr = Thread(target = self.__tick, daemon=True)
        self._last_marker_id = 0
        self._node_settings = settings
        self._node_settings.allow_connect = get_param(f"~allow_debug_connect", self._node_settings.allow_connect)
        self._node_settings.debug = get_param(f"~debug", self._node_settings.debug) and self._node_settings.allow_connect
        if self._node_settings.allow_connect:
            debug_port = 5678
            while True:
                try:
                    debugpy.listen(debug_port)
                    break
                except:
                    debug_port += 1
                    continue
            logwarn(f"Node: {self.node_settings.node_name} --> Listening for debugpy client on port: {debug_port}")
        if self.is_debug:
            logwarn(f"Node: {self.node_settings.node_name} --> Wainting for client connection")
            count = 0
            while not debugpy.is_client_connected() and count < 10:
                sleep(1)
                count += 1
                if is_shutdown():
                    sys.exit(1)
        self._reconfigure_sub = Subscriber("/reconfigure", Reconfigure, callback=self._reconfigureCb)
        self._marker_pub = Publisher("/markers", Marker, queue_size=20)
        logwarn(f"Node ({get_name()}) is listening to '/reconfigure' topic for own name!")
        loginfo(f"Node ({get_name()}) started!")

    def start_tick(self):
        self.__tick_thr.start()

    def _reconfigureCb(self, msg:Reconfigure):
        if msg.target_node == get_name():
            logwarn(f"Node ({get_name()}) : Received 'Reconfigure' command!")
            self.update_configs(msg.param_name)

    def draw_line(self, start: Coord, end: Coord, duration: float = 0.5) -> None:
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.ARROW
        marker.header.frame_id = self.node_settings.base_frame
        marker.header.stamp = Time.now()
        marker.header.seq = self._last_marker_id
        marker.id = self._last_marker_id
        marker.ns = self.node_name
        self._last_marker_id += 1
        marker.pose.position.x = start.x
        marker.pose.position.y = start.y
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.scale.x = start.dist_to(end)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = Duration(duration)
        marker.pose.orientation = self.msg_from_theta(start.angle_from_x_axis_to(end))
        self._marker_pub.publish(marker)

    @abstractmethod
    def update_configs(self, param: Optional[str] = None):
        raise NotImplementedError