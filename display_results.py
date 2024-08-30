import cv2
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import csv
from catkin_ws.src.hexapod_ros.scripts.roboMath import rotate

cm = 1/2.54
textwidth = 13.98611*cm

normalsize = 12
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "Cambria",
    "font.size": normalsize
})


with open(os.path.join("Results", "PoseData.csv"),'r') as file:
    row_count = sum(1 for lines in csv.reader(file))
    file.close()
pose_file = open(os.path.join("Results", "PoseData.csv"),'r')
csvfile = csv.reader(pose_file)
pose_line = next(csvfile)
pose_data = np.zeros((row_count,8))
fig, ax = plt.subplots(2,1)
plt.figure(1)
# ax[0].xaxis.set_inverted(True)
# ax[1].xaxis.set_inverted(True)
# ax[0].axis('equal')
# ax[1].axis('equal')
plt.xlabel('Side to side position (X, cm)')
ax[0].set(ylabel='Forward Backwards (Y, cm)')
ax[1].set(ylabel='Up Down (Z, cm)')
# plt.gca().set_aspect('equal')

plt.figure(2)
plt.title("Heightmap")
plt.xlabel("X (cm)")
plt.ylabel("Y (cm)")
cbar = None

# ax[0].set_xlim(0.01, -0.2)
# ax[0].set_ylim(-0.13, 0.02)

# plt.axis([-0.01,0.2,-0.13,0.04])
plt.ion()
plt.show()


angle = np.deg2rad(-17)

path_color = "Results/Color"
path_depth = "Results/Depth"
path_hmap = "Results/Hmap"
color_files = sorted(os.listdir(path_color))
depth_files = sorted(os.listdir(path_depth))
hmap_files = sorted(os.listdir(path_hmap))
frame_time_initial = int(os.path.splitext(color_files[0])[0])/1000_000_000.0
x = np.linspace(0,len(color_files),len(color_files))

time_total = int(os.path.splitext(color_files[-1])[0])/1000_000_000.0 - frame_time_initial
print(time_total)
frametimes = np.zeros(len(color_files))
delta_frametimes = np.zeros(len(color_files))

for i in range(len(color_files)-1):
    frametimes[i] = (int(os.path.splitext(color_files[i])[0]) - int(os.path.splitext(color_files[0])[0]))/1_000_000_000
    delta_frame_time = (int(os.path.splitext(color_files[i+1])[0]) - int(os.path.splitext(color_files[i])[0]))/1_000_000_000
    delta_frametimes[i] = delta_frame_time

# plt.plot(x, delta_frametimes)
# plt.plot(x, delta_frametimes,"o")
# plt.show()
pose_i = 0
pose_end = False
i_depth = 0
depth_frames = []
hmap_frames = []

for depth_file in depth_files:
    depth_frames.append(np.load(os.path.join(path_depth,depth_file)))

for hmap_file in hmap_files:
    hmap_frames.append(np.load(os.path.join(path_hmap,hmap_file)))

print((int(pose_line[0])-int(os.path.splitext(color_files[0])[0]))/1_000_000_000)
first = True
first_color = True
for i in range(len(color_files)-1):
    t_now = time.time()
    if frametimes[i] >= 25.0:
        cv2.imshow("Color", cv2.imread(os.path.join(path_color, color_files[i]))[:,:,[2,1,0]])
        # if i_depth < len(depth_frames) and frametimes[i] >= (int(os.path.splitext(depth_files[i_depth])[0]) - int(os.path.splitext(color_files[0])[0]))/1_000_000_000:
            # plt.figure(3)
            # plt.imshow(depth_frames[i_depth])
            # cv2.imshow("Hmap",depth_frames[i_depth])
            # i_depth += 1
        if frametimes[i] >= (int(pose_line[0])-int(os.path.splitext(color_files[0])[0]))/1_000_000_000 and not pose_end:
            hmap_i = hmap_files.index(pose_line[0]+".npy")
            if hmap_i is not None:
                hmap = hmap_frames[hmap_i]
                plt.figure(2)
                hmap[:5,:5] = 0
                himg = plt.imshow(np.flip(hmap.transpose(),1)*10,extent=(-hmap.shape[0]*0.25*1.3333, hmap.shape[0]*0.25*1.3333, -hmap.shape[1]*0.25*1.3333, hmap.shape[1]*0.25*1.3333), cmap='plasma')
                if cbar:
                    cbar.remove()
                    cbar = plt.colorbar(himg)
                    cbar.set_label("cm")
                else:
                    cbar = plt.colorbar(himg)
                    cbar.set_label("cm")
                # cv2.imshow("Hmap", np.flip(hmap.transpose(),1))
                
                # cv2.imshow("Hmap", hmap)

            pose_data[pose_i] = pose_line
            pose_data[pose_i][0] *= 10
            pose_data[pose_i][1] *= 10
            pose_data[pose_i][2] *= -10
            # pose_data[pose_i][1:4] = rotate(np.array([np.sin(angle)*1, np.sin(angle)*0, np.sin(angle)*0, np.cos(angle)]), np.array(pose_line[1:4],dtype=np.float))
            try:
                pose_line = next(csvfile)
            except StopIteration:
                pose_end = True
            else:
                # print(pose_line)
                pose_i += 1
                # ax[0].plot(-pose_data[0:pose_i, 1], pose_data[0:pose_i, 3], color='black')
                # ax[0].plot(-pose_data[0:pose_i, 1], pose_data[0:pose_i, 3],'o', color='black')
                # ax[0].plot(-pose_data[pose_i-1, 1], pose_data[pose_i-1, 3],'o', color='red')
                # ax[0].plot(-pose_data[0, 1], pose_data[0, 3],'o', color='blue')

                # ax[1].plot(-pose_data[0:pose_i, 1], -pose_data[0:pose_i, 2], color='black')
                # ax[1].plot(-pose_data[0:pose_i, 1], -pose_data[0:pose_i, 2], 'o', color='black')
                # ax[1].plot(-pose_data[pose_i-1, 1], -pose_data[pose_i-1, 2], 'o', color='red')
                # ax[1].plot(-pose_data[0, 1], -pose_data[0, 2], 'o', color='blue')

                ax[0].plot(pose_data[0:pose_i, 2], pose_data[0:pose_i, 1], color='black')
                ax[0].plot(pose_data[0:pose_i, 2], pose_data[0:pose_i, 1],'o', color='black')
                ax[0].plot(pose_data[pose_i-1, 2], pose_data[pose_i-1, 1],'o', color='red')
                ax[0].plot(pose_data[0, 2], pose_data[0, 1],'o', color='blue')

                ax[1].plot(pose_data[0:pose_i, 2], pose_data[0:pose_i, 3], color='black')
                ax[1].plot(pose_data[0:pose_i, 2], pose_data[0:pose_i, 3], 'o', color='black')
                ax[1].plot(pose_data[pose_i-1, 2], pose_data[pose_i-1, 3], 'o', color='red')
                ax[1].plot(pose_data[0, 2], pose_data[0, 3], 'o', color='blue')
                plt.draw()
                plt.pause(0.0001)
                
                if first and frametimes[i] >= 25:
                    first = False
                    plt.figure(2)
                    fig.set_figwidth(textwidth)
                    plt.savefig("hmap_start.pdf", format="pdf", bbox_inches="tight")
                    cv2.imwrite("color_start.jpeg", cv2.imread(os.path.join(path_color, color_files[i]))[:,:,[2,1,0]])
                elif first_color and i > 1300:
                    first_color = False
                    plt.figure(2)
                    fig.set_figwidth(textwidth)
                    plt.savefig("hmap_half.pdf", format="pdf", bbox_inches="tight")
                    cv2.imwrite("color_half.jpeg", cv2.imread(os.path.join(path_color, color_files[i]))[:,:,[2,1,0]])

        cv2.waitKey(1)
        t_delta_true = time.time() - t_now
        print(t_delta_true, delta_frametimes[i], i)
        # time.sleep(delta_frametimes[i] - (t_now - time.time()))
cv2.imshow("Color", cv2.imread(os.path.join(path_color, color_files[-1]))[:,:,[2,1,0]])
cv2.imwrite("color.jpeg", cv2.imread(os.path.join(path_color, color_files[i]))[:,:,[2,1,0]])

# while True:
#     angle += np.deg2rad(1)
#     ax[0].plot(-pose_data[:, 1], pose_data[:, 3], color='black')
#     ax[0].plot(-pose_data[:, 1], pose_data[:, 3],'o', color='black')
#     ax[0].plot(-pose_data[-1, 1], pose_data[-1, 3],'o', color='red')
#     ax[0].plot(-pose_data[0, 1], pose_data[0, 3],'o', color='blue')
#     h = -pose_data[:, 2]
#     ax[1].plot(-pose_data[:, 1], h, color='black')
#     ax[1].plot(-pose_data[:, 1], h, 'o', color='black')
#     ax[1].plot(-pose_data[-1, 1], -pose_data[-1, 2], 'o', color='red')
#     ax[1].plot(-pose_data[0, 1], -pose_data[0, 2], 'o', color='blue')
#     plt.draw()
plt.figure(1)
fig.set_figwidth(textwidth)
plt.savefig("pos.pdf", format="pdf", bbox_inches="tight")
plt.figure(2)
fig.set_figwidth(textwidth)
plt.savefig("hmap.pdf", format="pdf", bbox_inches="tight")
input("Enter end")