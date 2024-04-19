from enum import IntEnum
from dataclasses import dataclass, field
from typing import Callable, Dict

import rospy

class State(IntEnum):
    JUST_STARTED=0
    PREPARING=1
    ACTIVE=2
    OUT_OF_TASKS=3
    GO_HOME=4
    FULL_TIME_OUT=5

class ChangeState(IntEnum):
    INSERT_PIN = 0
    REMOVE_PIN = 1

    GO_HOME = 2
    ROUND_OVER = 3
    OUT_OF_TASKS = 4
    REDO_CANCELLED = 5

@dataclass
class ManagerState:
    _current: State = field(default=State.JUST_STARTED, init=False)
    _cbs: Dict[State, Callable] = field(default_factory=dict, init=False)
    @property
    def current(self): return self._current
    @current.setter
    def current(self, n):
        if self._current == n: return
        rospy.logwarn(f"[TASKS] State changed: {self._current!r} --> {n!r}")
        self._current = n
        if n in self._cbs: self._cbs[n]()
    def register_callbacks(self, cbs: Dict[State, Callable]): self._cbs = cbs
    def insert_pin(self): self.try_change(ChangeState.INSERT_PIN)
    def remove_pin(self): self.try_change(ChangeState.REMOVE_PIN)
    def redo_cancelled(self): self.try_change(ChangeState.REDO_CANCELLED)
    def out_of_tasks(self): self.try_change(ChangeState.OUT_OF_TASKS)
    def try_change(self, apply: ChangeState):

        def _accept_timeout_and_home():
            if apply == ChangeState.GO_HOME:
                self.current = State.GO_HOME
            if apply == ChangeState.ROUND_OVER:
                self.current = State.FULL_TIME_OUT
        def _accept_reset_with_pin():
            if apply == ChangeState.INSERT_PIN:
                self.current = State.PREPARING


        if self.current == State.JUST_STARTED:
            _accept_reset_with_pin()

        elif self.current == State.PREPARING:
            if apply == ChangeState.REMOVE_PIN: 
                self.current = State.ACTIVE

        elif self.current == State.GO_HOME:
            _accept_reset_with_pin()
            _accept_timeout_and_home()

        elif self.current == State.ACTIVE:
            _accept_reset_with_pin()
            _accept_timeout_and_home()
            if apply == ChangeState.OUT_OF_TASKS:
                self.current = State.OUT_OF_TASKS

        elif self.current == State.FULL_TIME_OUT:
            _accept_reset_with_pin()

        elif self.current == State.OUT_OF_TASKS:
            if apply == ChangeState.REDO_CANCELLED:
                self.current = State.ACTIVE
            _accept_reset_with_pin()
            _accept_timeout_and_home()

        else: raise ValueError("Invalid state")