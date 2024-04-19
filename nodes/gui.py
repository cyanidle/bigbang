#!/usr/bin/env python3
from dataclasses import dataclass, field
import sys
from threading import Thread
import tkinter as tk
from typing import Optional
from py_lib.node_base import NodeBase, NodeBaseSettings
from py_lib.rosparams_dataclass import RosparamsDataclass
import rospy
from std_srvs.srv import Empty, EmptyRequest
from bigbang_eurobot.srv import ExecuteScript, ExecuteScriptRequest
from bigbang_eurobot.msg import MonteCarloState, Measure2d

@dataclass
class GuiTopics(RosparamsDataclass):
    reset_particles_service: str = "particles/discard_belief"
    pump_on: str = "scripts/pump_on"
    pump_off: str = "scripts/pump_off"
    monte_state: str = "monte_carlo_state"
    position_sub: str = "filtered_pos"
    open_hatch: str = "scripts/open_hatch"
    reload_route: str = "task_manager/reload_route"
    close_hatch: str = "scripts/close_hatch"

@dataclass
class Canvas(RosparamsDataclass):
    width: int = 800
    height: int = 600

@dataclass
class GuiParams(RosparamsDataclass):
    topics: GuiTopics = field(default_factory=GuiTopics)
    canvas: Canvas = field(default_factory=Canvas)
    enable_exit: bool = False

class GuiNode(NodeBase):
    def __init__(self, settings: NodeBaseSettings) -> None:
        self.params = GuiParams()
        super().__init__(settings)
        self.update_configs()
        self.setup_tk()

    def setup_tk(self):
        self.gui = tk.Tk()
        self.gui.title("Bigbang Gui")
        self.reset_but = tk.Button(self.gui, text="Reset Particles", command=self.resetter.call, bg="red")
        self.pump_on_but = tk.Button(self.gui, text="Pump ON", command=self.pump_oner.call)
        self.pump_off_but = tk.Button(self.gui, text="Pump OFF", command=self.pump_offer.call)
        self.hatch_open_but = tk.Button(self.gui, text="Open Hatch", command=self.open_hatch.call)
        self.hatch_close_but = tk.Button(self.gui, text="Close Hatch", command=self.close_hatch.call)
        self.reload_route_but = tk.Button(self.gui, text="Reload Route", command=self.reload_route.call, bg="red")
        self._density_var = tk.StringVar(value="Density: Unknown")
        self._pos_var = tk.StringVar(value="Pos: x: None; y: None: Theta: None")
        self.set_pos = lambda x, y, th: self._pos_var.set(f"Pos: x: {x:.2f}; y: {y:.2f}; Theta: {th:.2f}")
        self.density_label = tk.Label(self.gui, textvariable=self._density_var)
        self.pos_label = tk.Label(self.gui, textvariable=self._pos_var)
        if self.params.enable_exit:
            self.exit_button = tk.Button(self.gui, text="Exit", fg="red", command=self.gui.destroy)
            self.exit_button.pack(anchor=tk.CENTER)
        self.reset_but.pack(anchor=tk.CENTER)
        self.pump_on_but.pack(anchor=tk.CENTER)
        self.pump_off_but.pack(anchor=tk.CENTER)
        self.hatch_open_but.pack(anchor=tk.CENTER)
        self.hatch_close_but.pack(anchor=tk.CENTER)
        self.reload_route_but.pack(anchor=tk.CENTER)
        self.density_label.pack(anchor=tk.CENTER)
        self.pos_label.pack(anchor=tk.CENTER)
        self.gui.attributes('-fullscreen', True)
        self.gui.resizable(False, False)
        self.gui.after(50, self.timer)

    def timer(self):
        if rospy.is_shutdown():
            self.gui.destroy()
        else:
            self.gui.after(50, self.timer)

    def set_density(self, d: float):
        if hasattr(self, "_density_var"): self._density_var.set(f"Density: {d:.2f}")

    def update_configs(self, param: Optional[str] = None):
        self.params.update(param)
        self.resetter = rospy.ServiceProxy(self.params.topics.reset_particles_service, Empty, persistent=True)
        self.pump_oner = rospy.ServiceProxy(self.params.topics.pump_on, ExecuteScript, persistent=True)
        self.pump_offer = rospy.ServiceProxy(self.params.topics.pump_off, ExecuteScript, persistent=True)
        self.close_hatch = rospy.ServiceProxy(self.params.topics.close_hatch, ExecuteScript, persistent=True)
        self.open_hatch = rospy.ServiceProxy(self.params.topics.open_hatch, ExecuteScript, persistent=True)
        self.reload_route = rospy.ServiceProxy(self.params.topics.reload_route, Empty, persistent=True)
        def _monte_cb(msg: MonteCarloState): 
            if hasattr(self, "set_density"): self.set_density(msg.density)
        self.monte_sub = rospy.Subscriber(self.params.topics.monte_state, MonteCarloState, _monte_cb)
        def _pos_cb(msg: Measure2d): 
            if hasattr(self, "set_pos"): self.set_pos(msg.x, msg.y, msg.theta)
        self.pos_sub = rospy.Subscriber(self.params.topics.position_sub, Measure2d, _pos_cb)
        self.should_restart = True

def main():
    node_name = "gui_node"
    rospy.init_node(node_name)
    gui = GuiNode(NodeBaseSettings(
        node_name=node_name
    ))
    ros_thr = Thread(target=rospy.spin, daemon=True)
    ros_thr.start()
    gui.gui.mainloop()

if __name__ == "__main__":
    main()