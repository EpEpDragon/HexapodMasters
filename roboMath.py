from math import sin, asin, cos, acos, tan, atan, pi
from math import sqrt

import numpy as np
import quaternion
from quaternion import from_vector_part, as_vector_part

def lerp(a,b,t):
    return a + (b-a)*t

def slerp(a,b,t):
    ang = acos(a.dot(b)/sqrt(a.dot(a)*b.dot(b)))
    return (sin((1-t)*ang)/sin(ang))*a + (sin(t*ang)/sin(ang))*b

def rotate_vec(vector, axis, angle):
    angle /= 2
    s = sin(angle)
    q = np.quaternion(cos(angle), s*axis[0], s*axis[1], s*axis[2])
    return as_vector_part(q*from_vector_part(vector)*q.conjugate())