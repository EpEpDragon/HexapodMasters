from math import sin, asin, cos, acos, tan, atan, pi
from math import sqrt

import numpy as np
import quaternion
from quaternion import from_vector_part, as_vector_part

def lerp(a,b,t):
    return a + (b-a)*t

def clerp(a,b,s):
    """lerp at a constant rate, intended for feedback use"""
    d = b-a
    if d == 0:
        return a
    return a + (abs(d)/(d))*s

def rotate_vec(vector, axis, angle):
    angle /= 2
    s = sin(angle)
    q = np.quaternion(cos(angle), s*axis[0], s*axis[1], s*axis[2])
    return as_vector_part(q*from_vector_part(vector)*q.conjugate())