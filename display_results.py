import cv2
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import csv


with open(os.path.join("Results", "PoseData.csv"),'r') as file:
    row_count = sum(1 for lines in csv.reader(file))
    file.close()
pose_file = open(os.path.join("Results", "PoseData.csv"),'r')
csvfile = csv.reader(pose_file)
pose_line = next(csvfile)
pose_data = np.zeros((row_count,8))
fig, ax = plt.subplots(2)
ax[0].xaxis.set_inverted(True)
ax[1].xaxis.set_inverted(True)
ax[0].axis('equal')
ax[1].axis('equal')
plt.xlabel('Side to side position (Y)')
ax[0].set(ylabel='Forward Backwards (X)')
ax[1].set(ylabel='Up Down (Z)')
plt.gca().set_aspect('equal')


# ax[0].set_xlim(0.01, -0.2)
# ax[0].set_ylim(-0.13, 0.02)

# plt.axis([-0.01,0.2,-0.13,0.04])
plt.ion()
plt.show()



path = "Results/Color"
files = sorted(os.listdir(path))
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

# plt.plot(x, delta_frametimes)
# plt.plot(x, delta_frametimes,"o")
# plt.show()
pose_i = 0
pose_end = False
print((int(pose_line[0])-int(os.path.splitext(files[0])[0]))/1_000_000_000)
for i in range(len(files)-1):
    if frametimes[i] >= 25.0:
        cv2.imshow("Color", cv2.imread(os.path.join(path, files[i]))[:,:,[2,1,0]])
        if frametimes[i] >= (int(pose_line[0])-int(os.path.splitext(files[0])[0]))/1_000_000_000 and not pose_end:
            pose_data[pose_i] = pose_line
            try:
                pose_line = next(csvfile)
            except StopIteration:
                pose_end = True
            else:
                # print(pose_line)
                pose_i += 1
                ax[0].plot(-pose_data[0:pose_i, 1], pose_data[0:pose_i, 3], color='black')
                ax[0].plot(-pose_data[0:pose_i, 1], pose_data[0:pose_i, 3],'o', color='black')
                ax[0].plot(-pose_data[pose_i-1, 1], pose_data[pose_i-1, 3],'o', color='red')
                ax[0].plot(-pose_data[0, 1], pose_data[0, 3],'o', color='blue')
                h = -pose_data[0:pose_i, 2]
                ax[1].plot(-pose_data[0:pose_i, 1], h, color='black')
                ax[1].plot(-pose_data[0:pose_i, 1], h, 'o', color='black')
                ax[1].plot(-pose_data[pose_i-1, 1], -pose_data[pose_i-1, 2], 'o', color='red')
                ax[1].plot(-pose_data[0, 1], -pose_data[0, 2], 'o', color='blue')
                plt.draw()
                plt.pause(0.001)
        cv2.waitKey(1)
        time.sleep(delta_frametimes[i])
cv2.imshow("Color", cv2.imread(os.path.join(path, files[-1]))[:,:,[2,1,0]])
input("Enter end")