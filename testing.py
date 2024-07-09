import numpy as np
import matplotlib.pyplot as plt
import csv
import math

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

x = []
y = []

roll = []
pitch = []

with open('body.csv','r') as csvfile: 
    plots = csv.reader(csvfile, delimiter = ',') 
    for row in plots:
        x.append(round(float(row[0])*1000,2))
        y.append(round(float(row[2])*1000,2))
        r,p,_ = euler_from_quaternion(float(row[3]), float(row[4]),float(row[5]),float(row[6]))
        roll.append(np.rad2deg(r))
        pitch.append(np.rad2deg(p))

fig,ax = plt.subplots(2)
ax[0].plot(x, y, color='black')

ax[1].plot(x, roll, ':', color='black')
ax[1].plot(x, pitch, '--', color='black')
plt.show()


# with open('feet.csv','r') as csvfile: 
#     plots = csv.reader(csvfile, delimiter = ',') 
#     for row in plots:
#         x.append(np.round(np.array(row[::3]).astype(float)*1000,2))
#         y.append(np.round(np.array(row[2::3]).astype(float)*1000,2))

# fig2,ax3 = plt.subplots()
# ax3.set_ylim(0,1000)
# for i in range(6):
#     ax3.plot(x[i::6], y[i::6])

# plt.show()
