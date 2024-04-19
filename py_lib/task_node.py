import dataclasses
from threading import Thread
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple
from bigbang_eurobot.srv import ExecuteScriptRequest, ExecuteScript, ExecuteScriptResponse
from pydantic import BaseModel, Field, root_validator
from py_lib.costmap import normalized_theta

from py_lib.script_types import *

TaskName = str

class SingleAction(BaseModel):
    call: str
    args: List[str] = Field(default_factory=list)
    reward: int = 0
    def as_call(self):
        return ServiceCall(self.call, self.args)

@dataclass
class ServiceCall:
    name: str
    args: List[str] = field(default_factory=list)
    if TYPE_CHECKING:
        _: dataclasses.KW_ONLY
    thread: Thread = field(init=False, repr=False)
    proxy: rospy.ServiceProxy = field(init=False, repr=False)
    cancelled: bool = False
    def __post_init__(self):
        self.proxy = rospy.ServiceProxy(f"scripts/{self.name}", ExecuteScript, persistent=True)
        self.thread = Thread(target=self._exec, daemon=True)
    def _exec(self):
        try:
            res: ExecuteScriptResponse = self.proxy.call(ExecuteScriptRequest(self.args))
            self.cancelled = not res.status
        except Exception as e:
            rospy.logerr(f"Exception in call: {e.__class__.__name__}{e}. Call: {self!r}")
            self.cancelled = True
    def start(self):
        self.cancelled = False
        self.thread.start()
    def cancel(self):
        self.cancelled = True
    @property
    def is_busy(self):
        return self.thread.is_alive() and not self.cancelled

@dataclass
class Coeffs(RosparamsDataclass):
    dist_to_cost: float = 10
    theta_to_cost: float = 2
    time_to_cost: float = 2

@dataclass
class TasksSettings(RosparamsDataclass):
    coeffs: Coeffs = Coeffs()

class TaskNode(BaseModel, extra=Extra.allow):
    start_pos: Optional[MoveTarget] = None
    actions: List[SingleAction] = Field(default_factory=list)
    cleanup: List[SingleAction] = Field(default_factory=list)
    priority_next: Optional[TaskName] = None
    required: Optional[TaskName] = None
    redoable: bool = True
    reward: int = 0
    innate_cost: int = 0
    expected_time: float = 0
    # state
    cancelled: bool = False
    _done_count: int = -1
    _start_called: bool = False
    _canceler: Optional[Callable] = None
    _current_call: Optional[ServiceCall] = None
    _was_done = False
    @property
    def was_done(self):
        return self._was_done
    
    def set_cancel_handler(self, canceller = Callable):
        self._canceler = canceller

    def reset(self):
        self.cancelled = False
        self._done_count = -1
        self._start_called = False
        self._current_call = None
        self._was_done = False

    def start(self):
        self._start_called = True
        if self.start_pos:
            rospy.logwarn(f"Starting MOVE: {self.start_pos.as_args}")
            self._current_call = SingleAction(call="move", args=self.start_pos.as_args).as_call()
        elif self.actions:
            self._current_call = self.actions[0].as_call()
        else:
            self._was_done = True
            self._cleanup()
            return
        self._current_call.start()


    def wait_finished(self):
        while self.proceed():
            rospy.sleep(0.05)

    def calculate_reward(self):
        res = self.reward if self.was_done and not self.cancelled else 0.
        for i in range(self._done_count):
            res += self.actions[i].reward
        for a in self.cleanup:
            res += a.reward
        return res

    def proceed(self):
        if self._was_done:  return False
        if self.cancelled: raise RuntimeError("Proceed called after cancel!")
        if self._current_call is None: raise RuntimeError("Cannot proceed before calling start()!")
        if self._current_call.is_busy: 
            return True
        rospy.logwarn(f"[TASKS] Call finished")
        if self._current_call.cancelled:
            self.cancel()
            self._cleanup()
            return False
        self._done_count += 1
        self._current_call = None
        if (self._start_called and self._done_count >= len(self.actions)): 
            self._cleanup()
            self._was_done = True
            return False
        rospy.logwarn(f"[TASKS] Starting action: {self.actions[self._done_count]!r}")
        self._current_call = self.actions[self._done_count].as_call()
        self._current_call.start()
        return True

    def cancel(self):
        rospy.logwarn(f"[TASKS] Task cancelled!")
        if not self._start_called:
            raise RuntimeError(f"TaskNode.cancel() called before start()")
        if self._current_call:
            if self._canceler:
                try: self._canceler()
                except Exception as e: rospy.logerr(f"While canceling call: {e.__class__.__name__} --> {e}")
            self._current_call = None
        self.cancelled = True

    def _cleanup(self):
        for t in self.cleanup:
            c = t.as_call()
            rospy.logwarn(f"[TASKS] Cleanup: {t!r}")
            c.start()
            while c.is_busy and not c.cancelled:
                rospy.sleep(0.05)

    def get_cost_to(self, current_pos: Position, params: TasksSettings):
        if self.start_pos:
            dist_res = current_pos.dist_to(self.start_pos.as_pos) * params.coeffs.dist_to_cost if self.start_pos else 0
            theta_res = normalized_theta(current_pos.theta - self.start_pos.theta) * params.coeffs.theta_to_cost
        else:
            dist_res = theta_res = 0
        return self.innate_cost + dist_res + theta_res + (self.expected_time * params.coeffs.time_to_cost)



class TasksGraph(BaseModel):    
    tasks: Dict[TaskName, TaskNode] 
    timed: Dict[float, List[SingleAction]] = Field(default_factory=dict)
    finish: TaskNode
    start: TaskNode
    @root_validator
    def tasks_non_empty(cls, values):
        if not values["tasks"]: raise ValueError("Tasks cannot be empty!")
        return values
    def all_cancelled(self):
        return list(filter(lambda t: t.cancelled, self.tasks.values()))
    def reset(self):
        self.finish.reset()
        self.start.reset()
        for t in self.tasks.values():
            t.reset()