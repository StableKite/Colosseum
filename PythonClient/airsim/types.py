from __future__ import print_function
from math import isnan
from pprint import pformat
from enum import Enum, IntEnum
from dataclasses import dataclass, asdict, field, fields, is_dataclass
from numpy import float32, uint8, uint64, ndarray, nan, sctypes, cross as np_cross, array
from typing import get_type_hints, get_origin, get_args, Self, Iterator, Tuple, List, Dict, TypeAlias, Any, Type


class MsgpackMixin:
    def __repr__(self):
        return f"<{type(self).__name__}> {pformat(asdict(self), indent = 4, width = 1)}"


    def to_msgpack(self) -> Dict[str, Any]:
        def _serialize(obj):
            if isinstance(obj, list):
                return [_serialize(item) for item in obj]
            elif isinstance(obj, Enum):
                return obj.value
            elif is_dataclass(obj):
                return {f.name: _serialize(getattr(obj, f.name)) for f in fields(obj)}
            return obj

        return _serialize(self)


    @classmethod
    def from_msgpack(cls: Type["MsgpackMixin"], encoded: Dict[str, Any]) -> "MsgpackMixin":
        type_hints = get_type_hints(cls)
        
        def _deserialize(field_type, value):
            origin_type = get_origin(field_type) or field_type

            if origin_type is list:
                item_type = get_args(field_type)[0]
                return [_deserialize(item_type, item) for item in value]
            if is_dataclass(field_type):
                if issubclass(field_type, MsgpackMixin):
                    return field_type.from_msgpack(value)
                else:
                    return field_type(**value)
            elif issubclass(field_type, Enum):
                return field_type(value)
            return value

        init_fields = {}
        for field in fields(cls):
            if field.name in encoded:
                field_value = encoded[field.name]
                field_type = type_hints[field.name]
                init_fields[field.name] = _deserialize(field_type, field_value)

        return cls(**init_fields)
        

RgbaType: TypeAlias = Tuple[float, float, float, float]


class MessageSeverity(IntEnum):
    FATAL = 0
    ERROR = 1
    WARNING = 2
    INFO = 3


@dataclass
class DistortionParams:
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    p1: float = 0.0
    p2: float = 0.0


class ImageType(IntEnum):
    SCENE = 0
    DEPTH_PLANAR = 1
    DEPTH_PERSPECTIVE = 2
    DEPTH_VIS = 3
    DISPARITY_NORMALIZED = 4
    SEGMENTATION = 5
    SURFACE_NORMALS = 6
    INFRARED = 7
    OPTICAL_FLOW = 8
    OPTICAL_FLOW_VIS = 9


class DrivetrainType(IntEnum):
    MAX_DEGREE_OF_FREEDOM = 0
    FORWARD_ONLY = 1


class LandedState(IntEnum):
    LANDED = 0
    FLYING = 1


class WeatherParameter(IntEnum):
    RAIN = 0
    ROADWETNESS = 1
    SNOW = 2
    ROAD_SNOW = 3
    MAPLE_LEAF = 4
    ROAD_LEAF = 5
    DUST = 6
    FOG = 7
    ENABLED = 8


@dataclass
class Vector2r(MsgpackMixin):
    x_val: float = 0.0
    y_val: float = 0.0


@dataclass
class Quaternionr(MsgpackMixin):
    x_val: float = 0.0
    y_val: float = 0.0
    z_val: float = 0.0
    w_val: float = 0.0

    @staticmethod
    def nanQuaternionr() -> Self:
        return Quaternionr(nan, nan, nan, nan)

    def containsNan(self) -> bool:
        return (isnan(self.w_val) or isnan(self.x_val) or isnan(self.y_val) or isnan(self.z_val))

    def __add__(self, other: Self) -> Self:
        if type(self) == type(other):
            return Quaternionr(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val, self.w_val + other.w_val)
        else:
            raise TypeError(f"Unsupported operand type(s) for +: {type(self)} and {type(other)}")

    def __mul__(self, other: Self) -> Self:
        if type(self) == type(other):
            t, x, y, z = self.w_val, self.x_val, self.y_val, self.z_val
            a, b, c, d = other.w_val, other.x_val, other.y_val, other.z_val
            return Quaternionr(
                w_val = a * t - b * x - c * y - d * z,
                x_val = b * t + a * x + d * y - c * z,
                y_val = c * t + a * y + b * z - d * x,
                z_val = d * t + z * a + c * x - b * y
            )
        else:
            raise TypeError(f"Unsupported operand type(s) for *: {type(self)} and {type(other)}")

    def __truediv__(self, other: Self) -> Self:
        if type(other) == type(self):
            return self * other.inverse()
        elif type(other) in [int, float] + sctypes["int"] + sctypes["uint"] + sctypes["float"]:
            return Quaternionr( self.x_val / other, self.y_val / other, self.z_val / other, self.w_val / other)
        else:
            raise TypeError(f"Unsupported operand type(s) for /: {type(self)} and {type(other)}")

    def dot(self, other: Self) -> float:
        if type(self) == type(other):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val + self.w_val * other.w_val
        else:
            raise TypeError(f"Unsupported operand type(s) for \"dot\": {type(self)} and {type(other)}")

    def cross(self, other: Self) -> Self:
        if type(self) == type(other):
            return (self * other - other * self) / 2
        else:
            raise TypeError(f"Unsupported operand type(s) for \"cross\": {type(self)} and {type(other)}")

    def outer_product(self, other: Self) -> Self:
        if type(self) == type(other):
            return (self.inverse() * other - other.inverse() * self ) / 2
        else:
            raise TypeError(f"Unsupported operand type(s) for \"outer_product\": {type(self)} and {type(other)}")

    def rotate(self, other: Self) -> Self:
        if type(self) == type(other):
            if other.get_length() == 1:
                return other * self * other.inverse()
            else:
                raise ValueError("length of the other Quaternionr must be 1")
        else:
            raise TypeError(f"Unsupported operand type(s) for \"rotate\": {type(self)} and {type(other)}")

    def conjugate(self) -> Self:
        return Quaternionr(-self.x_val, -self.y_val, -self.z_val, self.w_val)

    def star(self) -> Self:
        return self.conjugate()

    def inverse(self) -> Self:
        return self.star() / self.dot(self)

    def sgn(self) -> Self:
        return self / self.get_length()

    def get_length(self) -> float:
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2 + self.w_val ** 2) ** 0.5

    def to_numpy_array(self) -> ndarray:
        return array([self.x_val, self.y_val, self.z_val, self.w_val], dtype = float32)

    def __iter__(self) -> Iterator:
        return iter((self.x_val, self.y_val, self.z_val, self.w_val))


@dataclass
class Vector3r(MsgpackMixin):
    x_val: float = 0.0
    y_val: float = 0.0
    z_val: float = 0.0

    @staticmethod
    def nanVector3r() -> Self:
        return Vector3r(nan, nan, nan)

    def containsNan(self) -> bool:
        return (isnan(self.x_val) or isnan(self.y_val) or isnan(self.z_val))

    def __add__(self, other: Self) -> Self:
        return Vector3r(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val)

    def __sub__(self, other: Self) -> Self:
        return Vector3r(self.x_val - other.x_val, self.y_val - other.y_val, self.z_val - other.z_val)

    def __truediv__(self, other: Self) -> Self:
        if type(other) in [int, float] + sctypes["int"] + sctypes["uint"] + sctypes["float"]:
            return Vector3r( self.x_val / other, self.y_val / other, self.z_val / other)
        else:
            raise TypeError(f"Unsupported operand type(s) for /: {type(self)} and {type(other)}")

    def __mul__(self, other: Self) -> Self:
        if type(other) in [int, float] + sctypes["int"] + sctypes["uint"] + sctypes["float"]:
            return Vector3r(self.x_val * other, self.y_val * other, self.z_val * other)
        else:
            raise TypeError(f"Unsupported operand type(s) for *: {type(self)} and {type(other)}")

    def dot(self, other: Self) -> Self:
        if type(self) == type(other):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val
        else:
            raise TypeError(f"Unsupported operand type(s) for \"dot\": {type(self)} and {type(other)}")

    def cross(self, other: Self) -> Self:
        if type(self) == type(other):
            cross_product = np_cross(self.to_numpy_array(), other.to_numpy_array())
            return Vector3r(cross_product[0], cross_product[1], cross_product[2])
        else:
            raise TypeError(f"Unsupported operand type(s) for \"cross\": {type(self)} and {type(other)}")

    def get_length(self) -> float:
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2) ** 0.5

    def distance_to(self, other: Self) -> float:
        return ((self.x_val-other.x_val) ** 2 + (self.y_val-other.y_val) ** 2 + (self.z_val-other.z_val) ** 2) ** 0.5

    def to_Quaternionr(self) -> Quaternionr:
        return Quaternionr(self.x_val, self.y_val, self.z_val, 0)

    def to_numpy_array(self) -> ndarray:
        return array([self.x_val, self.y_val, self.z_val], dtype=float32)

    def __iter__(self) -> Iterator:
        return iter((self.x_val, self.y_val, self.z_val))


@dataclass
class Pose(MsgpackMixin):
    position: Vector3r = field(default_factory = Vector3r)
    orientation: Quaternionr = field(default_factory = Quaternionr)

    def __post_init__(self):
        if self.position is None:
            self.position = Vector3r()
        if self.orientation is None:
            self.orientation = Quaternionr()

    @staticmethod
    def nanPose() -> Self:
        return Pose(Vector3r.nanVector3r(), Quaternionr.nanQuaternionr())

    def containsNan(self) -> bool:
        return (self.position.containsNan() or self.orientation.containsNan())

    def __iter__(self) -> Iterator:
        return iter((self.position, self.orientation))


@dataclass
class CollisionInfo(MsgpackMixin):
    has_collided: bool = False
    normal: Vector3r = field(default_factory = Vector3r)
    impact_point: Vector3r = field(default_factory = Vector3r)
    position: Vector3r = field(default_factory = Vector3r)
    penetration_depth: float = 0.0
    time_stamp: float = 0.0
    object_name: str = ""
    object_id: int = -1


@dataclass
class GeoPoint(MsgpackMixin):
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0


@dataclass
class YawMode(MsgpackMixin):
    is_rate: bool = True
    yaw_or_rate: float = 0.0


@dataclass
class RCData(MsgpackMixin):
    timestamp: int = 0
    pitch: float = 0.0
    roll: float = 0.0
    throttle: float = 0.0
    yaw: float = 0.0
    switches: int = 0
    is_initialized: bool = False
    is_valid: bool = False


@dataclass
class ImageRequest(MsgpackMixin):
    camera_name: str = "0"
    image_type: ImageType = ImageType.SCENE
    pixels_as_float: bool = False
    compress: bool = False


@dataclass
class ImageResponse(MsgpackMixin):
    image_data_uint8: uint8 = uint8(0)
    image_data_float: float = 0.0
    camera_position: Vector3r = field(default_factory = Vector3r)
    camera_orientation: Quaternionr = field(default_factory = Quaternionr)
    time_stamp: uint64 = uint64(0)
    message: str = ""
    pixels_as_float: float = 0.0
    compress: bool = True
    width: int = 0
    height: int = 0
    image_type: ImageType = ImageType.SCENE


@dataclass
class CarControls(MsgpackMixin):
    throttle: float = 0.0
    steering: float = 0.0
    brake: float = 0.0
    handbrake: bool = False
    is_manual_gear: bool = False
    manual_gear: int = 0
    gear_immediate: bool = True

    def set_throttle(self, throttle_val: int, forward: bool):
        if forward:
            self.is_manual_gear = False
            self.manual_gear = 0
            self.throttle = abs(throttle_val)
        else:
            self.is_manual_gear = False
            self.manual_gear = -1
            self.throttle = - abs(throttle_val)


@dataclass
class KinematicsState(MsgpackMixin):
    position: Vector3r = field(default_factory = Vector3r)
    orientation: Quaternionr = field(default_factory = Quaternionr)
    linear_velocity: Vector3r = field(default_factory = Vector3r)
    angular_velocity: Vector3r = field(default_factory = Vector3r)
    linear_acceleration: Vector3r = field(default_factory = Vector3r)
    angular_acceleration: Vector3r = field(default_factory = Vector3r)


@dataclass
class EnvironmentState(MsgpackMixin):
    position: Vector3r = field(default_factory = Vector3r)
    geo_point: GeoPoint = field(default_factory = GeoPoint)
    gravity: Vector3r = field(default_factory = Vector3r)
    air_pressure: float = 0.0
    temperature: float = 0.0
    air_density: float = 0.0


@dataclass
class CarState(MsgpackMixin):
    speed: float = 0.0
    gear: int = 0
    rpm: float = 0.0
    maxrpm: float = 0.0
    handbrake: bool = False
    collision: CollisionInfo = field(default_factory = CollisionInfo)
    kinematics_estimated: KinematicsState = field(default_factory = KinematicsState)
    timestamp: uint64 = uint64(0)


@dataclass
class MultirotorState(MsgpackMixin):
    collision: CollisionInfo = field(default_factory = CollisionInfo)
    kinematics_estimated: KinematicsState = field(default_factory = KinematicsState)
    gps_location: GeoPoint = field(default_factory = GeoPoint)
    timestamp: uint64 = uint64(0)
    landed_state: LandedState = LandedState.LANDED
    rc_data: RCData = field(default_factory = RCData)
    ready: bool = False
    ready_message: str = ""
    can_arm: bool = False


@dataclass
class RotorState:
    thrust: float = 0.0
    torque_scaler: float = 0.0
    speed: float = 0.0


@dataclass
class RotorStates(MsgpackMixin):
    timestamp: uint64 = uint64(0)
    rotors: List[RotorState] = field(default_factory = list)


@dataclass
class ProjectionMatrix(MsgpackMixin):
    matrix: List[List[float]] = field(default_factory = list)


@dataclass
class CameraInfo(MsgpackMixin):
    pose: Pose = field(default_factory = Pose)
    fov: int = -1
    proj_mat: ProjectionMatrix = field(default_factory = ProjectionMatrix)


@dataclass
class LidarData(MsgpackMixin):
    point_cloud: List[float] = field(default_factory = list)
    time_stamp: uint64 = uint64(0)
    pose: Pose = field(default_factory = Pose)
    segmentation: List[int] = field(default_factory = list)


@dataclass
class ImuData(MsgpackMixin):
    time_stamp: uint64 = uint64(0)
    orientation: Quaternionr = field(default_factory = Quaternionr)
    angular_velocity: Vector3r = field(default_factory = Vector3r)
    linear_acceleration: Vector3r = field(default_factory = Vector3r)


@dataclass
class BarometerData(MsgpackMixin):
    time_stamp: uint64 = uint64(0)
    altitude: float = 0.0
    pressure: float = 0.0
    qnh: float = 0.0


@dataclass
class MagnetometerData(MsgpackMixin):
    time_stamp: uint64 = uint64(0)
    magnetic_field_body: Vector3r = field(default_factory = Vector3r)
    magnetic_field_covariance: float = 0.0


class GnssFixType(IntEnum):
    GNSS_FIX_NO_FIX = 0
    GNSS_FIX_TIME_ONLY = 1
    GNSS_FIX_2D_FIX = 2
    GNSS_FIX_3D_FIX = 3


@dataclass
class GnssReport(MsgpackMixin):
    geo_point: GeoPoint = field(default_factory = GeoPoint)
    eph: float = 0.0
    epv: float = 0.0
    velocity: Vector3r = field(default_factory = Vector3r)
    fix_type: GnssFixType = GnssFixType.GNSS_FIX_NO_FIX
    time_utc: uint64 = uint64(0)


@dataclass
class GpsData(MsgpackMixin):
    time_stamp: uint64 = uint64(0)
    gnss: GnssReport = field(default_factory = GnssReport)
    is_valid: bool = False


@dataclass
class DistanceSensorData(MsgpackMixin):
    time_stamp: uint64 = uint64(0)
    distance: float = 0.0
    min_distance: float = 0.0
    max_distance: float = 0.0
    relative_pose: Pose = field(default_factory = Pose)


@dataclass
class Box2D(MsgpackMixin):
    min: Vector2r = field(default_factory = Vector2r)
    max: Vector2r = field(default_factory = Vector2r)


@dataclass
class Box3D(MsgpackMixin):
    min: Vector3r = field(default_factory = Vector3r)
    max: Vector3r = field(default_factory = Vector3r)


@dataclass
class DetectionInfo(MsgpackMixin):
    name: str = ""
    geo_point: GeoPoint = field(default_factory = GeoPoint)
    box2D: Box2D = field(default_factory = Box2D)
    box3D: Box3D = field(default_factory = Box3D)
    relative_pose: Pose = field(default_factory = Pose)


PidList: TypeAlias = Tuple[float, float, float]

@dataclass
class PidGains:
    """
    Struct to store values of PID gains. Used to transmit controller
    gain values while instantiating.
    AngleLevel/AngleRate/Velocity/PositionControllerGains objects.

    Attributes:
        kP (float): Proportional gain
        kI (float): Integrator gain
        kD (float): Derivative gain
    """

    kp: float
    ki: float
    kd: float

    def to_list(self) -> PidList:
        return [self.kp, self.ki, self.kd]


PidLists: TypeAlias = Tuple[PidList, PidList, PidList]

@dataclass
class AngleRateControllerGains:
    """
    Struct to contain controller gains
    used by angle level PID controller

    Attributes:
        roll_gains (PidGains): kP, kI, kD for roll axis
        pitch_gains (PidGains): kP, kI, kD for pitch axis
        yaw_gains (PidGains): kP, kI, kD for yaw axis
    """

    roll_gains: PidGains = field(default_factory = lambda: PidGains(0.25, 0.0, 0.0))
    pitch_gains: PidGains = field(default_factory = lambda: PidGains(0.25, 0.0, 0.0))
    yaw_gains: PidGains = field(default_factory = lambda: PidGains(0.25, 0.0, 0.0))


    def to_lists(self) -> PidLists:
        return [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp], \
            [self.roll_gains.ki, self.pitch_gains.ki, self.yaw_gains.ki], \
            [self.roll_gains.kd, self.pitch_gains.kd, self.yaw_gains.kd]


@dataclass
class AngleLevelControllerGains:
    """
    Struct to contain controller gains used by angle rate PID controller

    Attributes:
        roll_gains (PidGains): kP, kI, kD for roll axis
        pitch_gains (PidGains): kP, kI, kD for pitch axis
        yaw_gains (PidGains): kP, kI, kD for yaw axis
    """

    roll_gains: PidGains = field(default_factory = lambda: PidGains(2.5, 0.0, 0.0))
    pitch_gains: PidGains = field(default_factory = lambda: PidGains(2.5, 0.0, 0.0))
    yaw_gains: PidGains = field(default_factory = lambda: PidGains(2.5, 0.0, 0.0))

    def to_lists(self) -> PidLists:
        return [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp], \
            [self.roll_gains.ki, self.pitch_gains.ki, self.yaw_gains.ki], \
            [self.roll_gains.kd, self.pitch_gains.kd, self.yaw_gains.kd]


@dataclass
class VelocityControllerGains:
    """
    Struct to contain controller gains used by velocity PID controller

    Attributes:
        x_gains (PidGains): kP, kI, kD for X axis
        y_gains (PidGains): kP, kI, kD for Y axis
        z_gains (PidGains): kP, kI, kD for Z axis
    """

    x_gains: PidGains = field(default_factory = lambda: PidGains(0.2, 0., 0.))
    y_gains: PidGains = field(default_factory = lambda: PidGains(0.2, 0., 0.))
    z_gains: PidGains = field(default_factory = lambda: PidGains(0., 0., 0.))

    def to_lists(self) -> PidLists:
        return [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp], \
            [self.x_gains.ki, self.y_gains.ki, self.z_gains.ki], \
            [self.x_gains.kd, self.y_gains.kd, self.z_gains.kd]


@dataclass
class PositionControllerGains:
    """
    Struct to contain controller gains used by position PID controller

    Attributes:
        x_gains (PidGains): kP, kI, kD for X axis
        y_gains (PidGains): kP, kI, kD for Y axis
        z_gains (PidGains): kP, kI, kD for Z axis
    """

    x_gains: PidGains = field(default_factory = lambda: PidGains(0.25, 0., 0.))
    y_gains: PidGains = field(default_factory = lambda: PidGains(0.25, 0., 0.))
    z_gains: PidGains = field(default_factory = lambda: PidGains(0., 0., 0.))

    def to_lists(self) -> PidLists:
        return [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp], \
            [self.x_gains.ki, self.y_gains.ki, self.z_gains.ki], \
            [self.x_gains.kd, self.y_gains.kd, self.z_gains.kd]


@dataclass
class MeshPositionVertexBuffersResponse(MsgpackMixin):
    position: Vector3r = field(default_factory = Vector3r)
    orientation: Quaternionr = field(default_factory = Quaternionr)
    vertices: float = 0.0
    indices: float = 0.0
    name: str = ""