import numpy as np
angle = np.deg2rad(-30)
roots = np.zeros((6,3))
for i in range(6):
    roots[i] = 0.8*np.array([(np.cos(angle)), (np.sin(angle)), 0])
    print(np.rad2deg(angle), roots[i])
    angle += np.deg2rad(60)
print(roots)