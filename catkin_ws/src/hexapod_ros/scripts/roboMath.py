from math import sin, asin, cos, acos, tan, atan, pi
from math import sqrt

import numpy as np
# import quaternion
# from quaternion import from_vector_part, as_vector_part

def lerp(a,b,t):
    return a + (b-a)*t

def clerp(a,b,s):
    """lerp at a constant rate, intended for feedback use"""
    d = b-a
    if d == 0:
        return a
    return a + (abs(d)/(d))*s

def rotate(q, p):
    return p + 2.0*np.cross(q[0:3],np.cross(q[0:3],p)+q[3]*p)

# def rotate_vec(vector, axis, angle):
#     angle /= 2
#     s = sin(angle)
#     q = np.quaternion(cos(angle), s*axis[0], s*axis[1], s*axis[2])
#     return as_vector_part(q*from_vector_part(vector)*q.conjugate())


# def rotate_vec_quat(vector, q):
#     q = np.quaternion(q[0], q[1], q[2], q[3])
#     return as_vector_part(q*from_vector_part(vector)*q.conjugate())

# def rotate_quat(q, axis, angle):
#     angle /= 2
#     s = sin(angle)
#     q2 = np.quaternion(cos(angle), s*axis[0], s*axis[1], s*axis[2])
#     return q2*q*q2.conjugate()

# def rotate_quat_quat(q1,q2):
#     return q2*q1*q2.conjugate()