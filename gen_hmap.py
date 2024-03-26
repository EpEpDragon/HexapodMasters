import cv2
import numpy as np
# from math import copysign
# import matplotlib.pyplot as plt
# from perlin_noise import PerlinNoise

size = 128

hmap = np.zeros((size,size,1), np.float32)
n = 6 * 3.141 / size

for x in range(size):
    for y in range(size):
        hmap[x,y] = round((np.sin(x*n)*np.sin(y*n)/2.0 + 0.5))


# noise1 = PerlinNoise(octaves=2, seed=5426)
# noise2 = PerlinNoise(octaves=4, seed=4321)
# noise3 = PerlinNoise(octaves=8, seed=8621)
# p1 = np.zeros_like(hmap)
# p2 = np.zeros_like(hmap)
# p3 = np.zeros_like(hmap)

# for x in range(size):
#     for y in range(size):
#         hmap[x,y] = ((noise1([x/size, y/size]) + noise2([x/size, y/size])*0.75 + noise3([x/size, y/size])*0.5) + 1) / 2
#         # hmap[x,y] = (noise1([x/size, y/size])+1)/2
#         p1[x,y] = (noise1([x/size, y/size])+1)/2
#         p2[x,y] = (noise2([x/size, y/size])*0.75+1)/2
#         p3[x,y] = (noise3([x/size, y/size])*0.5+1)/2

# # hmap = [[noise([i/xpix, j/ypix]) for j in range(xpix)] for i in range(ypix)]

# cv2.namedWindow("hmap", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("hmap", 1024,1024)
# cv2.namedWindow("p1", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("p1", 1024,1024)
# cv2.namedWindow("p2", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("p2", 1024,1024)
# cv2.namedWindow("p3", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("p3", 1024,1024)

cv2.imshow("hmap", hmap)
# cv2.imshow("p1", p1)
# cv2.imshow("p2", p2)
# cv2.imshow("p3", p3)


# cv2.waitKey(0)

cv2.imwrite("hmap.png", hmap*255)
# cv2.imwrite("hmap_test.png", hmap*255)

#####################

# torque = np.genfromtxt("torque.csv", delimiter=",")
# print(torque)

# # xpoints = np.array(range(torque.shape[0]))
# ypoints = torque[1000:3000]

# plt.plot(ypoints)
# plt.show()

######################