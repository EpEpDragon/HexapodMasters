import numpy as np
import cv2
from math import cos, sqrt

PI = 3.141592654

def quadratic_kernel(size, max, scale):
    kernel = np.empty((size,size))
    for i in range(size):
        for j in range(size):
            x = (i-size/2)*scale
            y = (j-size/2)*scale
            kernel[i,j] = 5*(x*x+y*y)
    return kernel

def wrap_slice(a, axis, start, stop):
    # fn to convert your start stop to a wrapped range
    if stop<=start:
        stop += a.shape[axis]
    return np.arange(start, stop)%a.shape[axis]


def wrap_block(a, r_start, r_stop, c_start, c_stop):
    rows = wrap_slice(a, 0, r_start, r_stop)
    rows = np.repeat(rows, rows.size)
    cols = wrap_slice(a, 1, c_start, c_stop)
    cols = np.tile(cols, cols.size)
    return rows,cols


def get_wrapped(matrix, i, j):
  m, n = matrix.shape
  rows = [(i-1) % m, i, (i+1) % m]
  cols = [(j-1) % n, j, (j+1) % n]
  return matrix[rows][:, cols]

size = 20
kernel = quadratic_kernel(size, 1, 2/size).flatten()
kernel = kernel.reshape(kernel.shape[0],1)

array = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]])


# r,c = wrap_block(array,2,1,2,1)
# print(array)
# print("")
# print(array[wrap_slice(array,0,2,0)][:,wrap_slice(array,1,2,0)])
# print("")
# print(array[r,c])
# print("")
# array[r,c] = 10
# print(array)
# print("")
# print(array[wrap_slice(array,0,2,0)][:,wrap_slice(array,1,2,0)])

baseimg = cv2.imread("hmap.png").astype(float) / 255
baseimg = cv2.resize(baseimg, (128,128))
baseimg = baseimg/0.8 + 0.2
img = np.ones((128,128,3))
img[:] = baseimg[:]

def draw_kernel(event,x,y,flags,param):
    global mouseX,mouseY
    img[:] = baseimg[:]
    # if event == cv2.EVENT_LBUTTONDOWN:
    r,c = wrap_block(img, (y-int(size/2))%img.shape[0], (y+int(size/2))%img.shape[0], (x-int(size/2))%img.shape[0], (x+int(size/2))%img.shape[0])
    img[r,c] = abs(baseimg[r,c]-baseimg[x,y]) < kernel
    img[y,x] = 1.0
    print(img[r,c].min())
    mouseX,mouseY = x,y

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 1024, 1024)
cv2.setMouseCallback('image',draw_kernel)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):
        print(mouseX,mouseY)
        print(img)