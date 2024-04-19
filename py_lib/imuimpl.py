from dataclasses import dataclass
from py_lib.arduino_params import OdomCovariance, MoveCovariance, ImuCovariance, StartPosition
from sensor_msgs.msg import Imu
from py_lib.rosparams_dataclass import RosparamsDataclass
from bigbang_eurobot.msg import RawImu, Move2d
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Vector3

@dataclass
class ImuSettings(RosparamsDataclass):
    covariance: ImuCovariance = ImuCovariance()
    move_covariance: MoveCovariance = MoveCovariance(matrix=[1000, 0, 0,
                                                                 0, 1000, 0,
                                                                 0, 0, 0]) 

class ImuImpl():
    def __init__(self, settings: ImuSettings, start_pos: StartPosition) -> None:
        self.settings = settings
        self.start_pos = start_pos
        self._imu = Imu()
        self._imu.angular_velocity_covariance  = self.settings.covariance.rotation
        self._imu.linear_acceleration_covariance  = self.settings.covariance.acceleration
        self._imu.orientation_covariance = self.settings.covariance.orientaion
        
    # 1) geometry_msgs/Quaternion orientation
    # 2) float64[9] orientation_covariance # Row major about x, y, z axes
    # 3) geometry_msgs/Vector3 angular_velocity
    # 4) float64[9] angular_velocity_covariance # Row major about x, y, z axes
    # 5) geometry_msgs/Vector3 linear_acceleration
    # 6) float64[9] linear_acceleration_covariance # Row major x, y z 

    def receive(self, msg: RawImu):
        self._imu.orientation = Quaternion(*quaternion_from_euler(msg.angX, msg.angY, msg.angZ))
        self._imu.angular_velocity = Vector3(msg.rotX, msg.rotY, msg.rotZ)
        self._imu.linear_acceleration = Vector3(msg.linX, msg.linY, msg.linZ)
        return True
        
    @property
    def as_msg(self):
        return self._imu

    @property
    def as_move(self):
        return Move2d(
            0, 0, self._imu.angular_velocity,self.settings.move_covariance
        )
        