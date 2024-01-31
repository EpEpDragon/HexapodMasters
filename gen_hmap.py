import cv2
import numpy as np
from math import copysign

size = 512

hmap = np.zeros((size,size,1), np.float32)
n = 4 * 3.141 / size

for x in range(size):
    for y in range(size):
        hmap[x,y] = round((np.sin(x*n)*np.sin(y*n) + 1)/2)


cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.imshow("img", hmap)
cv2.waitKey(0)

cv2.imwrite("hmap.png", hmap*255)

# diff = np.array([-1])
# print(copysign(abs(diff)%-160,diff))