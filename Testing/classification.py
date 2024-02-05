import numpy as np
import cv2
from math import cos, sqrt
import matplotlib.pyplot as plt

PI = 3.141592654

def quadratic_kernel(size, max, scale):
    kernel = np.empty((size,size))
    for i in range(size):
        for j in range(size):
            x = (i-size/2)*scale
            y = (j-size/2)*scale
            kernel[i,j] = 40*(x*x*x*x+y*y*y*y)+0.3
    return kernel

def sample_gaussian(height, offset, stand_dev, dist, size=0):
    # gaussian = np.zeros(size)
    # for x in range(gaussian.size):
    #     gaussian[x] = height*pow(2.71828,-((3*x/size)*(3*x/size))/(2*stand_dev*stand_dev))
    return height*pow(2.71828,-((dist-offset)*(dist-offset))/(2*stand_dev*stand_dev))

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

kernel_size = 6
# kernel = quadratic_kernel(kernel_size, 1, 1/kernel_size).flatten()
# kernel = kernel.reshape(kernel.shape[0],1)
kernel = quadratic_kernel(kernel_size+1, 1, 2/kernel_size)
print(kernel)
kernel = kernel.reshape((kernel.shape[0],kernel.shape[1],1))

array = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]])

SIZE = 128
# baseimg = cv2.imread("hmap_test.png").astype(float) / 255
baseimg = cv2.imread("hmap.png").astype(float) / 255
baseimg = cv2.resize(baseimg, (SIZE,SIZE))
baseimg = baseimg*0.5
img = np.ones((SIZE,SIZE,3))
img[:] = baseimg[:]

def calculate_score(p_x,p_y,p_z,x,y):
    global mouseX,mouseY
    # img[:] = baseimg[:]
    # if event == cv2.EVENT_LBUTTONDOWN:
    # r,c = wrap_block(img, (y-int(kernel_size/2))%img.shape[0], (y+int(kernel_size/2))%img.shape[0], (x-int(kernel_size/2))%img.shape[0], (x+int(kernel_size/2))%img.shape[0])
    # img[r,c] =  kernel - abs(baseimg[r,c]-baseimg[y,x])
    # img[y,x] = 1.0

    temp = kernel - abs(baseimg[wrap_slice(baseimg,0, (y-int(kernel_size/2))%baseimg.shape[0], (y+int(kernel_size/2)+1)%baseimg.shape[0])][:,wrap_slice(baseimg,1,(x-int(kernel_size/2))%baseimg.shape[0], (x+int(kernel_size/2)+1)%baseimg.shape[0])] - baseimg[y,x,0])
    prox_score = max(temp.min(),0)

    dist = sqrt((x-p_x)*(x-p_x)+(y-p_y)*(y-p_y)+8*8*(baseimg[y,x,0]-p_z)*(baseimg[y,x,0]-p_z))
    radius = 10
    stand_dev = 3
    dist_score = max(sample_gaussian(height=1, offset=radius, stand_dev=stand_dev, dist=dist)-pow(2.71828, -2*(dist+1.5*stand_dev-radius))-0.1, 0)
    # temp[int(kernel_size/2), int(kernel_size/2)] = 1.0
    # img[r,c] = (kernel - abs(baseimg[wrap_slice(baseimg,0, (y-int(kernel_size/2))%baseimg.shape[0], (y+int(kernel_size/2))%baseimg.shape[0])][:,wrap_slice(baseimg,1,(x-int(kernel_size/2))%baseimg.shape[0], (x+int(kernel_size/2))%baseimg.shape[0])] - baseimg[x,y]).reshape((100,3)))
    # img[y,x] = 1.0

    mouseX,mouseY = x,y
    # return img[r,c].min()
    return dist_score * prox_score

def poll_value(event,x,y,flags,param):
    print(img[y,x])


cv2.namedWindow('hmap', cv2.WINDOW_NORMAL)
cv2.resizeWindow('hmap', 1024, 1024)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 1024, 1024)
cv2.setMouseCallback('image',poll_value)

# gaussian = sample_gaussian(1,1,20)
# plt.plot(gaussian)
# plt.show()

for r in range(SIZE):
    for c in range(SIZE):
        score = calculate_score(60,60,1,c,r)
        if score <= 0:
            img[r,c] = [0,0,1]
        else:
            img[r,c] = [0,score,0]

cv2.imshow('image',img)
cv2.imshow('hmap',baseimg)
cv2.waitKey(0)


# while(1):
#     cv2.imshow('image',img)
#     k = cv2.waitKey(20) & 0xFF
#     if k == 27:
#         break
#     elif k == ord('a'):
#         print(mouseX,mouseY)
#         print(img)