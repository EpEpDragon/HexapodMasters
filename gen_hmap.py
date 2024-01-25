import cv2
import numpy as np
from math import copysign

size = 512

hmap = np.zeros((size,size,1), np.float32)

for x in range(size):
    for y in range(size):
        hmap[x,y] = round((np.sin(x/(size/30))*np.sin(y/(size/30)) + 1)/2)


cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.imshow("img", hmap)
cv2.waitKey(0)

cv2.imwrite("hmap.png", hmap*255)

# diff = np.array([-1])
# print(copysign(abs(diff)%-160,diff))