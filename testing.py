import matplotlib.pyplot as plt
import numpy as np
import cmath
from mpl_toolkits import mplot3d

from math import sin, asin, cos, acos, tan, atan, pi
from math import sqrt

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

a = np.array([1.0, 0.0, 0.0])
b = np.array([2.0, 0.0, 0.0])
c = a + (b - a)/2
diff = b - a
n = np.cross(diff, np.array([0,0,1]))
print(n)
ang = 0 + (pi/4)/2
q1 = np.array([cos(ang), sin(ang)*n[0], sin(ang)*n[1], sin(ang)*n[2]])
q2 = q1*np.array([1,-1,-1,-1])
print(q1)
print(q2)
print()
print(ang)

xline = []
yline = []
zline = []

t = np.linspace(0.0, 1.0, 100)

for i in t:
    ang = 0 + (pi*i)/2
    q1 = np.array([cos(ang), sin(ang)*n[0], sin(ang)*n[1], sin(ang)*n[2]])
    q2 = q1*np.array([1,-1,-1,-1])
    out = q_mult(q_mult(q1,np.append([0],a)),q2)[1:4]
    xline.append(out[0])
    yline.append(out[1])
    zline.append(out[2])

fig = plt.figure
ax = plt.axes(projection='3d')
ax.plot3D(xline,yline,zline)
plt.show()

