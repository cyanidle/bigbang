from dataclasses import dataclass, field
import functools
from typing import Any, Dict, List, Literal, MutableSequence, Optional, Sequence, Type, Union, cast
from pydantic import BaseModel, Extra, Field
from bigbang_eurobot.msg import PlanerStatus, ServoCommand
from bigbang_eurobot.srv import ExecuteScriptRequest, DirectMoveResponse, DirectMove, DirectMoveRequest
from py_lib.costmap import Position
from py_lib.rosparams_dataclass import RosparamsDataclass
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, Bool

import rospy

ScriptName = str

def pop_to_named(args: MutableSequence[str], names: Sequence[str]) -> Dict[str, str]:
    if len(args) < len(names):
        raise RuntimeError(f"Cannot have args list shorter than names list: {args} < {names}")
    result = {}
    for name in names:
        result[name] = args.pop(0)
    return result

def parse_model(model: Type[BaseModel], args: MutableSequence[str]):
    return model.parse_obj(pop_to_named(args, tuple(model.__fields__)))

@dataclass
class MoveParams(RosparamsDataclass):
    try_reach_for: float = 1
    min_wait: float = 0.3

    pub_into: str = "global_planer/target"
    planer_state: str = "planer_status"
    direct_move_srv: str = "direct_move"

@dataclass
class ScriptsParams(RosparamsDataclass):
    move: MoveParams = field(default_factory=MoveParams)
    sleep_interval: float = 0.01

_available_topics = {
    'servo_command': ServoCommand,
    'lcd_show': Int16,
    'arduino/pump': Bool,
    'arduino/valve': Bool,
}

@dataclass
class ScriptData:
    params: ScriptsParams
    msg: ExecuteScriptRequest
    all: Dict[str, 'ScriptStep']

class ScriptsStates:
    @classmethod
    def wait_target_reached(cls, params: ScriptsParams):
        rospy.sleep(params.move.min_wait)
        def should_drive():
            return (cls.planer_status.idle_for < params.move.try_reach_for 
                    and not (cls.planer_status.reached and cls.planer_status.rotated))
        while should_drive():
            rospy.sleep(params.sleep_interval)
        return cls.planer_status.reached and cls.planer_status.rotated and not cls.planer_status.is_stuck
    @classmethod
    def init(cls, params: ScriptsParams):
        cls.planer_status = PlanerStatus()
        def cb(msg: PlanerStatus):
            cls.planer_status = msg
        cls.move = rospy.Publisher(params.move.pub_into, Point, queue_size=20)
        cls.dyn_topics: Dict[str, Type] = {} 
        for name, t in _available_topics.items():
            cls.dyn_topics[name] = rospy.Publisher(name, t, queue_size=20)
        cls._planer_stater = rospy.Subscriber(params.move.planer_state, PlanerStatus, callback=cb)
        cls.direct = rospy.ServiceProxy(params.move.direct_move_srv, DirectMove, True)

class ScriptStep(BaseModel, extra=Extra.allow):
    min_time: float = 0
    @property
    def is_move(self):
        return False
    def execute(self, data: ScriptData) -> bool:
        raise NotImplementedError
    @staticmethod
    def check_time(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            inst = args[0]
            assert isinstance(inst, ScriptStep)
            start = rospy.Time.now()
            result = func(*args, **kwargs)
            passed = (rospy.Time.now() - start).to_sec()
            rospy.sleep(inst.min_time - passed)
            return result
        return wrapper

def is_move(step: ScriptStep) -> bool:
    return step.is_move

class MoveTarget(BaseModel):
    x: float = 0
    y: float = 0
    theta: float = 0
    @property
    def as_pos(self):
        return Position(self.x, self.y, self.theta)
    @property
    def as_point(self):
        return Point(self.x, self.y, self.theta)
    @property   
    def as_args(self):
        return [str(self.x), str(self.y), str(self.theta)]

class MoveBase(ScriptStep):
    last_target: MoveTarget = Field(default_factory=MoveTarget, repr=False)
    @property
    def is_move(self):
        return True

class Move(MoveBase):
    target: MoveTarget
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        target = self.target
        self.last_target = target
        ScriptsStates.move.publish(target.as_point)
        return ScriptsStates.wait_target_reached(data.params)

class InnateMove(MoveBase):
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        target = MoveTarget.parse_obj(pop_to_named(data.msg.args, ("x", "y", "theta")))
        self.last_target = target
        ScriptsStates.move.publish(target.as_point)
        return ScriptsStates.wait_target_reached(data.params)

class CallService(ScriptStep):
    service: str
    args: List[str]

class PostMsgs(ScriptStep):
    topic: str
    msg: Optional[Dict[str, Any]] = None
    msgs: List[Dict[str, Any]] = Field(default_factory=list)
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        msg_type = _available_topics[self.topic]
        pub = ScriptsStates.dyn_topics[self.topic]
        try:
            if self.msg is not None:
                cmd = msg_type(**self.msg)
                pub.publish(cmd)
            for submsg in self.msgs:
                cmd = msg_type(**submsg)
                pub.publish(cmd)
            return True
        except:
            return False
        
class Wait(ScriptStep):
    amount: float
    def execute(self, data: ScriptData) -> bool:
        rospy.sleep(self.amount)
        return True

class InnateWait(ScriptStep):
    def execute(self, data: ScriptData) -> bool:
        rospy.sleep(float(data.msg.args[0]))
        return True

class CallExisting(ScriptStep):
    steps: List[str]
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        for name in self.steps:
            if not data.all[name].execute(data):
                return False
        return True

class RobotScript(ScriptStep):
    steps: List[ScriptStep]
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        for step in self.steps:
            if not step.execute(data):
                return False
        return True

class DirectMoveSrv(ScriptStep):
    y_dir: float
    x_dir: float
    dist: float
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        req = DirectMoveRequest(y_dir=self.y_dir, x_dir=self.x_dir, dist=self.dist)
        status: DirectMoveResponse = ScriptsStates.direct.call(req)
        return status.status


class InnateDirectMove(ScriptStep):
    @ScriptStep.check_time
    def execute(self, data: ScriptData) -> bool:
        parsed = pop_to_named(data.msg.args, ("x_dir", "y_dir", "dist"))
        req = DirectMoveRequest(y_dir=float(parsed["y_dir"]), x_dir=float(parsed["x_dir"]), dist=float(parsed["dist"]))
        status: DirectMoveResponse = ScriptsStates.direct.call(req)
        return status.status

TYPES_DICT: Dict[str, Type[ScriptStep]] = {
    "call_service": CallService,
    "post_msgs": PostMsgs,
    "post_msg": PostMsgs,
    "sleep": Wait,
    "move": Move, 
    "nested": RobotScript,
    "script": RobotScript,
    "existing": CallExisting,
    "direct_move": DirectMoveSrv
}

class ScriptStepRepr(BaseModel):
    type: str
    data: dict  
    def to_step(self) -> ScriptStep:
        cls = TYPES_DICT[self.type]
        result = cls.parse_obj(self.data)
        if isinstance(result, RobotScript):
            actual = []
            for step in result.steps:
                actual.append(ScriptStepRepr.parse_obj(step).to_step())
            result.steps = actual
        return result

class ScriptsRepr(BaseModel):
    scripts: Dict[str, ScriptStepRepr]
    def to_steps(self) -> Dict[str, ScriptStep]:
        result: Dict[str, ScriptStep] = {}
        for name, repr in self.scripts.items():
            result[name] = repr.to_step()
        return result
    

def test():
    args = ["1", "2", "3", "123"]
    lol = parse_model(MoveTarget, args)
    print(lol)

if __name__ == "__main__":
    test()