#/usr/bin/env python3
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Type, TypeVar, Union
from pydantic import BaseModel, Field
from py_lib.json_dict import JsonDict
from py_lib.node_base import NodeBase, NodeBaseSettings
from py_lib.rosparams_dataclass import RosparamsDataclass
from redis import ConnectionPool, Redis
from redis.exceptions import ConnectionError as RedisConnError
from redis.exceptions import TimeoutError as RedisTimeoutError
from std_msgs.msg import Int16
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from bigbang_eurobot.msg import PlanerStatus
from bigbang_eurobot.srv import ExecuteScript, ExecuteScriptResponse
import subprocess
import rospy

@dataclass
class RedisSettings(RosparamsDataclass):
    host: str = "localhost"
    port: int = 6379
    forward_balls_key: str = "biba:balls"

@dataclass
class TopicsSettings(RosparamsDataclass):
    lcd_show: str = "lcd_show"
    balls_topic: str = "arduino/balls"
    start_lift_pub: str = "basket:lift"
    start_lift_service: str = "scripts/basket_lift"

@dataclass
class Settings(RosparamsDataclass):
    child_frame_id: str = "interconnect"
    redis: RedisSettings = field(default_factory=RedisSettings)
    topics: TopicsSettings = field(default_factory=TopicsSettings)
    robot_name: str = "biba"
    friends: List[str] = field(default_factory=lambda: ["boba"])
    lift_time: int = 5500
    down_time: int = 3000

class PlanerStatusModel(BaseModel):
    reached: bool = False
    rotated: bool = False
    is_stuck: bool = False
    @classmethod
    def from_msg(cls, msg: PlanerStatus):
        return cls(**{
            "is_stuck": msg.is_stuck,
            "rotated": msg.rotated,
            "reached": msg.reached,
            "driving_for": msg.driving_for,
            "idle_for": msg.idle_for
        })

class RobotStateModel(BaseModel):
    score: int = 0
    balls: int = 0
    planer_status: PlanerStatusModel = Field(default_factory=PlanerStatusModel)
    def update(self, msg: Union[PlanerStatus, None]):
        if isinstance(msg, PlanerStatus):
            self.planer_status = self.planer_status.from_msg(msg)

class InterconnectNode(NodeBase):
    def __init__(self, settings: NodeBaseSettings) -> None:
        super().__init__(settings)
        self.params = Settings()
        self.state = RobotStateModel()
        self.redis_proc = None
        self.friends: Dict[str, RobotStateModel] = {}
        self.set_tick_rate(0.5)
        self.update_configs()
        self.start_tick()

    @property
    def child_frame_id(self):
        return self.params.child_frame_id
    @property
    def redis(self):
        return Redis(connection_pool=self.pool)
    def update_configs(self, param: str = ""):
        self.params.update(param)
        self.pool = ConnectionPool.from_url(f"redis://{self.params.redis.host}:{self.params.redis.port}", socket_connect_timeout=3)
        if self.params.redis.host == "localhost" and not self.redis_proc:
            self.redis_proc = subprocess.Popen(
                ["redis-server", 
                 "--port", str(self.params.redis.port), 
                 "--protected-mode", "no"
                ])
            rospy.sleep(2)
        def _lcd(msg: Int16): self.state.score = msg.data
        self.lcd_shower = rospy.Subscriber(self.params.topics.lcd_show, Int16, _lcd)
        def _balls(msg: Int16): self.state.balls = msg.data
        self.baller = rospy.Subscriber(self.params.topics.balls_topic, Int16, _balls)
        def _lift(req): 
            self.redis.publish(self.params.topics.start_lift_pub, f"{self.params.lift_time}-{self.params.down_time}")
            return ExecuteScriptResponse(status=True)
        self.lifter = rospy.Service(self.params.topics.start_lift_service, ExecuteScript, _lift)

    def read_state(self, robot_name: str) -> RobotStateModel:
        if rospy.is_shutdown(): return RobotStateModel()
        raw: dict = self.redis.hgetall(robot_name)
        return RobotStateModel.parse_obj(JsonDict(raw).top)

    def update_friends(self):
        for friend in self.params.friends:
            self.friends[friend] = self.read_state(friend)
        # TODO: send states to some topic

    def send_state(self):
        as_json = JsonDict(self.state.dict()).flattened()
        to_send = {}
        for k, v in as_json.items():
            to_send[k] = str(v)
        try:
            self.redis.hset(self.params.robot_name, mapping=to_send)
            if self.params.redis.forward_balls_key:
                self.redis.set(self.params.redis.forward_balls_key, self.state.balls)
        except:
            if rospy.is_shutdown(): pass
            else: raise

    def cleanup(self):
        self.state = RobotStateModel()
        self.send_state()
        if self.redis_proc:
            self.redis_proc.terminate()

    def tick(self):
        try:
            self.send_state()
            self.update_friends()
        except (RedisConnError, RedisTimeoutError): 
            rospy.logwarn_throttle(3, "[Redis] No connection")
        except: raise

def main():
    node_name = "interconnect_node"
    rospy.init_node(node_name)
    interconnect = InterconnectNode(NodeBaseSettings(
        node_name=node_name
    ))
    rospy.on_shutdown(interconnect.cleanup)
    rospy.spin()

if __name__ == "__main__":
    main()