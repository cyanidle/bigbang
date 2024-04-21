#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('bigbang_eurobot')
from dataclasses import dataclass, fields
from typing import List, Tuple, Callable
from functools import cache, partial
import debugpy
from geometry_msgs.msg import Point32
from py_lib.imuimpl import ImuImpl, ImuSettings
from py_lib.odomimpl import OdomSettings, OdomImpl
from py_lib.arduino_topics import ArduinoTopicsParams
from py_lib.arduino_params import MotorParams, ServosParams, StartPosition
from py_lib.node_base import NodeBase, NodeBaseSettings
from py_lib.rosparams_dataclass import RosparamsDataclass
from std_srvs.srv import EmptyRequest, Empty, EmptyResponse
from copy import copy
from math import cos, sin
from threading import Thread
import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from bigbang_eurobot.srv import ExecuteScript, ExecuteScriptResponse, ExecuteScriptRequest
from bigbang_eurobot.msg import (ArduinoCommand, PinReader,
                                MotorInfo, ServoCreateUpdate,
                                PinReaderResponce, RawImu, ServoCommand, 
                                Move2d, Measure2d, ArduinoStatus, ArduinoLed)
from bigbang_eurobot.msg import MotorParams as MotorParamsMsg
from geometry_msgs.msg import Point

class SLIP:
    END = 0xC0
    ESC = 0xDB
    EscapedEND = 0xdc
    EscapedESC = 0xdd

@dataclass
class ServiceCodes(RosparamsDataclass):
    ok: int = 11
    pin_present: int = 12
    pin_pulled: int = 13
    ball: int = 14
    @property
    def all_codes(self):
        return (self.ok, self.pin_present, self.pin_pulled, self.ball)

    def has(self, num: int) -> bool:
        return num in self.all_codes

@dataclass
class ArduinoInterfaceParams(RosparamsDataclass):
    motor0: MotorParams = MotorParams(num = 0, angle=0)
    motor1: MotorParams = MotorParams(num = 1, angle=120)
    motor2: MotorParams = MotorParams(num = 2, angle=240)
    start_position: StartPosition = StartPosition()
    odom_coeffs: OdomSettings = OdomSettings()
    imu_coeffs: ImuSettings = ImuSettings()
    ok_timeout: float = 5.
    bootup_time: float = 3.
    servos: ServosParams = ServosParams()
    service_codes: ServiceCodes = ServiceCodes()
    arduino_topics: ArduinoTopicsParams = ArduinoTopicsParams() 
    frame_id: str = "costmap"
    child_frame_id: str = "odom"
    tick_rate: float = 20
    pause_between_motors_create: float = 0.7
    delay_between_msgs: float = 0.004 # 5 ms
    correction_interval_sec: float = 1
    @property
    def all_motors(self):
        return (self.motor0, self.motor1, self.motor2)

class ArduinoInterface(NodeBase):
    @property
    def topics(self) -> ArduinoTopicsParams:
        return self.params.arduino_topics
    
    def __init__(self, settings: NodeBaseSettings) -> None:
        super().__init__(settings)
        self.params = ArduinoInterfaceParams()
        self.pending: List[Callable] = []
        self.odom = OdomImpl(self.params.frame_id,
                        self.params.all_motors,
                        self.params.odom_coeffs,
                        self.params.start_position)
        self.imu = ImuImpl(self.params.imu_coeffs,
                        self.params.start_position)
        self.ok_timeout = self.params.ok_timeout
        self.status = ArduinoStatus()
        self._last_correction = rospy.Time.now()
        self.update_configs()
        self.start_tick()

    @property
    def child_frame_id(self):
        return self.params.child_frame_id
    def _create_subs_pubs(self):
        self.motors_updater = rospy.Publisher(self.topics.pub_update_motor, MotorParamsMsg, queue_size=20)
        self.general_command = rospy.Publisher(self.topics.pub_command, ArduinoCommand, queue_size=20)
        self.topic_pin_command = rospy.Publisher(self.topics.pub_pin_command, PinReader, queue_size=20)
        self.topic_create_servo = rospy.Publisher(self.topics.pub_create_servo, ServoCreateUpdate, queue_size=20)
        self.topic_odom_pub = rospy.Publisher(self.topics.pub_odom, Odometry, queue_size=20)
        self.topic_imu_pub = rospy.Publisher(self.topics.pub_imu, Imu, queue_size=20)
        self.move_updates_pub = rospy.Publisher(self.topics.pub_move_updates, Move2d, queue_size=20)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.status_pub = rospy.Publisher(self.topics.pub_status, ArduinoStatus, queue_size=20)
        self.baller = rospy.Publisher(self.topics.pub_balls, Int16, queue_size=20)
        self.balls = 0
        def _reset_balls(req): 
            self.balls = 0
            return ExecuteScriptResponse(status=True)
        self.balls_resetter = rospy.Service(self.topics.reset_balls, ExecuteScript, _reset_balls)
        self.arduino_resetter = rospy.ServiceProxy(self.topics.reset_arduino_service, Empty)
        self.sub_valve = rospy.Subscriber(self.topics.sub_valve, Bool, self._valve_cb)
        self.sub_pump = rospy.Subscriber(self.topics.sub_pump, Bool, self._pump_cb)
        self.sub_motors_info = rospy.Subscriber(self.topics.sub_motors_info, MotorInfo, self._motor_info_cb)
        self.sub_pin_reply = rospy.Subscriber(self.topics.sub_pin_reply, PinReaderResponce, self._pin_reply_cb)
        self.sub_raw_imu = rospy.Subscriber(self.topics.sub_raw_imu, RawImu, self._raw_imu_cb)
        self.sub_cmd_vel = rospy.Subscriber(self.topics.sub_cmd_vel, Twist, self._cmd_vel_cb)
        self.sub_servo_command = rospy.Subscriber(self.topics.sub_servo_command, ServoCommand, self._servo_command_cb)
        self.sub_lcd_show = rospy.Subscriber(self.topics.sub_lcd_show, Int16, self._lcd_show_cb)
        self.sub_correction = rospy.Subscriber(self.topics.sub_correction, Measure2d, self._correct_cb)
        self.sub_led = rospy.Subscriber(self.topics.sub_led, ArduinoLed, self._led_cb)
        def _script_led(req: ExecuteScriptRequest):
            on, r, g, b = tuple(req.args)
            self._led_cb(ArduinoLed(on=bool(int(on)), r=int(r), g=int(g), b=int(b)))
            return ExecuteScriptResponse(status=True)
        def _script_led_red(req: ExecuteScriptRequest):
            self._led_cb(ArduinoLed(on=True, r=255, g=0, b=0))
            return ExecuteScriptResponse(status=True)
        def _script_led_off(req):
            self._led_cb(ArduinoLed(on=False))
            return ExecuteScriptResponse(status=True)
        self.sub_led_scr = rospy.Service(self.topics.srv_script_led, ExecuteScript, _script_led)
        self.sub_led_scr_off = rospy.Service(self.topics.srv_script_led_off, ExecuteScript, _script_led_off)
        self.sub_led_scr = rospy.Service(self.topics.srv_script_led_red, ExecuteScript, _script_led_red)
        self.sub_enable = rospy.Subscriber(self.topics.sub_enable_arduino, Bool, self._enable_arduino)

    def _led_cb(self, msg: ArduinoLed):
        self.general_command.publish(ArduinoCommand(
            function=7,
            aux=msg.on,
            x=msg.r,
            y=msg.g,
            z=msg.b
        ))

    def _valve_cb(self, msg: Bool):
        self.general_command.publish(ArduinoCommand(
            function=6,
            aux=msg.data
        ))

    def _pump_cb(self, msg: Bool):
        self.general_command.publish(ArduinoCommand(
            function=5,
            aux=msg.data
        ))

    def _enable_arduino(self, msg: Bool):
        rospy.logwarn_throttle(0.5, f"Arduino is enabled: {msg.data}")
        if msg.data:
            self.general_command.publish(ArduinoCommand(
                function=3
            ))
        else:
            self.general_command.publish(ArduinoCommand(
                function=4
            ))

    def _correct_cb(self, msg: Measure2d):
        now = rospy.Time.now()
        if (now - self._last_correction).to_sec() > self.params.correction_interval_sec:
            self.odom.correct(msg.x, msg.y, msg.theta)
            self._last_correction = now

    def publish_transform(self):
        self.odom_broadcaster.sendTransform(
            (self.odom.pose.x, self.odom.pose.y, 0.),
            quaternion_from_euler(0, 0, self.odom.pos_theta),
            rospy.Time.now(),
            self.params.child_frame_id,
            self.params.frame_id
        )

    def update_configs(self, param: str = None):
        self.params.update(param)
        self.set_tick_rate(self.tick_rate)
        self.ok_timeout =  self.params.ok_timeout
        self._create_subs_pubs()
        self._create_update_servos()
        self._update_motors()

    @property
    def codes(self):
        return self.params.service_codes

    def _handle_code(self, code: int):
        if code == self.codes.ok:
            self.status.ok = True
        elif code == self.codes.pin_present:
            self.status.pin_present = True
        elif code == self.codes.pin_pulled:
            self.status.pin_present = False
        elif code == self.codes.ball:
            self.balls += 1

    def _motor_info_cb(self, msg: MotorInfo):
        if self.codes.has(msg.num):
            self._handle_code(msg.num)
        elif self.odom.receive(msg):
            self.topic_odom_pub.publish(self.odom.as_msg)
            mv_msg = self.odom.as_move_msg
            self.move_updates_pub.publish(mv_msg)
            self.publish_transform()

    def _pin_reply_cb(self, msg: PinReaderResponce):
        rospy.loginfo(f"Responce from Pin: {msg.pin} --> Value: {msg.value}")

    def _raw_imu_cb(self, msg: RawImu):
        if self.imu.receive(msg):
            self.topic_imu_pub.publish(self.imu.as_msg)

    def _cmd_vel_cb(self, msg: Twist):
        self.general_command.publish(ArduinoCommand(
            function=0,
            x=msg.linear.x,
            y=msg.linear.y,
            z=msg.angular.z
        ))

    def _servo_command_cb(self, msg: ServoCommand):
        self.general_command.publish(ArduinoCommand(
            function=1,
            x=msg.percents,
            aux=msg.num
        ))

    def _lcd_show_cb(self, msg: Int16):
        self.general_command.publish(ArduinoCommand(
            function=2,
            aux=msg.data
        ))
    def _update_motors(self):
        rospy.loginfo(f"Updating motors")
        for motor_params in self.params.all_motors:
            rospy.loginfo(f"Updating motor: {motor_params}")
            self.enqueue_call(self.motors_updater.publish, motor_params.as_params_msg)
            self.enqueue_call(rospy.sleep, self.params.pause_between_motors_create)

    def _create_update_servos(self):
        for num, channel, speed, minVal, maxVal, startPercents in zip(
                self.params.servos.nums,
                self.params.servos.channels,
                self.params.servos.speeds,
                self.params.servos.minVals,
                self.params.servos.maxVals,
                self.params.servos.startPercents):
            self.enqueue_call(self.topic_create_servo.publish, ServoCreateUpdate(num, channel, speed, minVal, maxVal, startPercents))
            self.enqueue_call(rospy.sleep, self.params.pause_between_motors_create)
    
    def enqueue_call(self, call, *args, **kwargs):
        self.pending.append(partial(call, *args, **kwargs))

    def reset_arduino(self):
        rospy.logwarn("Waiting for Arduino Reset Service!")
        self.arduino_resetter.wait_for_service()
        rospy.logwarn("Resetting Arduino!")
        self.arduino_resetter.call(EmptyRequest())
        rospy.logwarn(f"Resetting Arduino Done! Sleeping for bootup time: {self.params.bootup_time}")
        rospy.sleep(self.params.bootup_time)
        self.update_configs()

    def tick(self):
        self.baller.publish(self.balls)
        self.status_pub.publish(self.status)
        while not self.status.ok:
            rospy.sleep(2)
        while self.pending:
            self.pending.pop(0)()
            rospy.sleep(self.params.delay_between_msgs)

    @property
    def tick_rate(self):
        return self.params.tick_rate

def main():
    node_name = "arduino_interface"
    rospy.init_node(node_name)
    interface = ArduinoInterface(NodeBaseSettings(
        node_name=node_name
    ))
    rospy.spin()

if __name__ == "__main__":
    main()