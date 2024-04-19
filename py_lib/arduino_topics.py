from dataclasses import dataclass
from py_lib.rosparams_dataclass import RosparamsDataclass

@dataclass
class ArduinoTopicsParams(RosparamsDataclass):
    pub_update_motor: str = "arduino_raw/update_motor"
    pub_command: str = "arduino_raw/arduino_command"
    pub_pin_command: str = "arduino_raw/pin_reader"
    pub_create_servo: str = "arduino_raw/create_update_servo"

    sub_motors_info: str = "arduino_raw/motor_info"
    sub_pin_reply: str = "arduino_raw/pin_reader_reply"
    sub_raw_imu: str = "arduino_raw/raw_imu"

    pub_odom: str = "odom"
    pub_imu: str = "imu"
    pub_move_updates: str = "move_updates"
    pub_status: str = "arduino/status"  

    sub_cmd_vel: str = "cmd_vel"
    sub_servo_command: str = "servo_command"
    sub_lcd_show: str = "lcd_show"
    sub_correction: str = "filtered_pos"

    sub_pump: str = "arduino/pump"
    sub_valve: str = "arduino/valve"
    sub_enable_arduino: str = "arduino/enable"

    reset_arduino_service: str = "arduino_serial_node/reset_arduino"
    reset_balls: str = "scripts/reset_balls"
    
    pub_balls: str = "arduino/balls"
    sub_led: str = "arduino/led"
    srv_script_led: str = "scripts/led"
    srv_script_led_red: str = "scripts/led_red"
    srv_script_led_off: str = "scripts/led_off"

