from os import name
from sys import stdin
from cv2 import imwrite
from warnings import warn
from cv2.typing import MatLike
from typing import Tuple, Dict, Any
from math import atan2, asin, cos, sin
from inspect import isbuiltin, isfunction, ismethod
from numpy import fromstring, reshape, asarray, uint8, float32, ndarray
from .types import Quaternionr


def string_to_uint8_array(bstr: str | bytes) -> ndarray:
    return fromstring(bstr, uint8)


def string_to_float_array(bstr: str | bytes) -> ndarray:
    return fromstring(bstr, float32)


def list_to_2d_float_array(flst: ndarray, width: int, height: int) -> ndarray:
    return reshape(asarray(flst, float32), (height, width))


def get_pfm_array(response) -> ndarray:
    return list_to_2d_float_array(
        response.image_data_float,
        response.width,
        response.height
    )

    
def get_public_fields(obj: object) -> str:
    return [
        attr for attr in dir(obj)
        if not (attr.startswith("_") 
        or isbuiltin(attr)
        or isfunction(attr)
        or ismethod(attr))
    ]

    
def to_dict(obj) -> Dict[str, Any]:
    return dict([attr, getattr(obj, attr)] for attr in get_public_fields(obj))

    
def to_str(obj: object) -> str:
    return str(to_dict(obj))

    
def write_file(filename, bstr):
    """
    Write binary data to file.
    Used for writing compressed PNG images
    """

    with open(filename, "wb") as afile:
        afile.write(bstr)


def to_eularian_angles(q: Quaternionr) -> Tuple[float, float, float]:
    """
    Helper method for converting getOrientation to roll / pitch / yaw
    https:#en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """

    z = q.z_val
    y = q.y_val
    x = q.x_val
    w = q.w_val
    ysqr = y * y

    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    if (t2 > 1.0):
        t2 = 1
    if (t2 < -1.0):
        t2 = -1.0
    pitch = asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw

    
def to_quaternion(pitch: float, roll: float, yaw: float) -> Quaternionr:
    t0 = cos(yaw * 0.5)
    t1 = sin(yaw * 0.5)
    t2 = cos(roll * 0.5)
    t3 = sin(roll * 0.5)
    t4 = cos(pitch * 0.5)
    t5 = sin(pitch * 0.5)

    q = Quaternionr()
    q.w_val = t0 * t2 * t4 + t1 * t3 * t5  # W
    q.x_val = t0 * t3 * t4 - t1 * t2 * t5  # X
    q.y_val = t0 * t2 * t5 + t1 * t3 * t4  # Y
    q.z_val = t1 * t2 * t4 - t0 * t3 * t5  # Z

    return q

    
def wait_key(message: str = ""):
    """
    Wait for a key press on the console and return it
    """

    if message != "":
        print (message)

    result = None
    if name == "nt":
        import msvcrt
        result = msvcrt.getch()
    else:
        import termios
        fd = stdin.fileno()

        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        try:
            result = stdin.read(1)
        except IOError:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)

    return result


def write_png(filename: str, image: MatLike):
    """
    Image must be numpy array H X W X channels
    """

    ret = imwrite(filename, image)
    if not ret:
        warn(f"Writing PNG file {filename} failed")