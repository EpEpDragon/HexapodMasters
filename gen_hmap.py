import cv2
import numpy as np

size = 512

hmap = np.zeros((size,size,1), np.float32)

for x in range(size):
    for y in range(size):
        hmap[x,y] = (np.sin(x/(size/15))*np.sin(y/(size/15)) + 1)/2


cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.imshow("img", hmap)
cv2.waitKey(0)

cv2.imwrite("hmap.png", hmap*255)