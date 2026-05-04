import math
from .data_structures import XY

def rotate_xy(p, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return XY(
        c * p.x - s * p.y,
        s * p.x + c * p.y
    )

def angle_between(a, b):
    return math.atan2(
        b.y - a.y,
        b.x - a.x
    )

def wrap_angle(a):
    return math.atan2(
        math.sin(a),
        math.cos(a)
    )

def rectangular_yaw_diff_deg(a, b):
    return abs(
        (a - b + 90.0) % 180.0 - 90.0
    )
