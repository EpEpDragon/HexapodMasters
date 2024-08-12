import cv2
import time
import os
import numpy as np
import matplotlib.pyplot as plt

files = sorted(os.listdir("Results/Color"))
frame_time_initial = int(os.path.splitext(files[0])[0])/1000_000_000.0
x = np.linspace(0,len(files),len(files))

time_total = int(os.path.splitext(files[-1])[0])/1000_000_000.0 - frame_time_initial
print(time_total)
frametimes = np.zeros(len(files))
delta_frametimes = np.zeros(len(files))

for i in range(len(files)-1):
    frametimes[i] = (int(os.path.splitext(files[i])[0]) - int(os.path.splitext(files[0])[0]))/1_000_000_000
    delta_frame_time = (int(os.path.splitext(files[i+1])[0]) - int(os.path.splitext(files[i])[0]))/1_000_000_000
    delta_frametimes[i] = delta_frame_time
print(frametimes)
plt.plot(x, delta_frametimes)
# plt.plot(x, delta_frametimes,"o")
plt.show()

for i in range(len(files)-1):
    cv2.imshow("Color", cv2.imread(os.path.join("Results/Color", files[i]))[:,:,[2,1,0]])
    cv2.waitKey(1)
    time.sleep(delta_frametimes[i])
cv2.imshow("Color", cv2.imread(os.path.join("Results/Color", files[-1]))[:,:,[2,1,0]])
print("FIN")
cv2.waitKey(0)