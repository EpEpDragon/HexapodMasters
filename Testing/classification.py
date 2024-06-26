import numpy as np
import cv2
from math import cos, sqrt, exp
import matplotlib.pyplot as plt

PI = 3.141592654

def quadratic_kernel(size, max, scale):
    kernel = np.empty((size,size))
    for i in range(size):
        for j in range(size):
            x = (i-(size-1)/2.0)*scale
            y = (j-(size-1)/2.0)*scale
            # kernel[i,j] = (x*x + y*y)
            kernel[i,j] = exp(-(x*x + y*y)*0.5)
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

kernel_size = 13
# kernel = quadratic_kernel(kernel_size, 1, 1/kernel_size).flatten()
# kernel = kernel.reshape(kernel.shape[0],1)
kernel = quadratic_kernel(kernel_size, 1, 0.25)
np.set_printoptions(linewidth=np.inf)
print(kernel)
kernel = kernel.reshape((kernel.shape[0],kernel.shape[1],1))

array = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]])

SIZE = 16*12
# baseimg = cv2.imread("hmap_test2.png").astype(float) / 255
baseimg = cv2.imread("hmap_test4.png").astype(float) / 255
baseimg = cv2.resize(baseimg, (SIZE,SIZE))
baseimg = baseimg*5
img = np.ones((SIZE,SIZE,3))
img[:] = baseimg[:]/5.0

def calculate_score(body_pos, foot_pos,x,y):
    global mouseX,mouseY
    # img[:] = baseimg[:]
    # if event == cv2.EVENT_LBUTTONDOWN:
    # r,c = wrap_block(img, (y-int(kernel_size/2))%img.shape[0], (y+int(kernel_size/2))%img.shape[0], (x-int(kernel_size/2))%img.shape[0], (x+int(kernel_size/2))%img.shape[0])
    # img[r,c] =  kernel - abs(baseimg[r,c]-baseimg[y,x])
    # img[y,x] = 1.0

    # temp = kernel - abs(baseimg[wrap_slice(baseimg,0, (y-int(kernel_size/2))%baseimg.shape[0], (y+int(kernel_size/2)+1)%baseimg.shape[0])][:,wrap_slice(baseimg,1,(x-int(kernel_size/2))%baseimg.shape[0], (x+int(kernel_size/2)+1)%baseimg.shape[0])] - baseimg[y,x,0])
    
    r,c = wrap_block(img, (y-int((kernel_size)/2))%img.shape[0], (y+int((kernel_size)/2)+1)%img.shape[0], (x-int((kernel_size)/2))%img.shape[0], (x+int((kernel_size)/2)+1)%img.shape[0])
    temp =  kernel.reshape((kernel.size,1))*(baseimg[r,c,0]-baseimg[y,x,0])
    # temp[int((kernel.size-1)/2)] = 1.0
    terrain_proximity_score = abs(np.average(temp))

    Gx = 0.5*(+baseimg[(y-1)%img.shape[0], (x-1)%img.shape[1],0] - baseimg[(y+1)%img.shape[0], (x-1)%img.shape[1],0]
          +2*baseimg[(y-1)%img.shape[0], (x)%img.shape[1],0] - 2*baseimg[(y+1)%img.shape[0], (x)%img.shape[1],0]
          +baseimg[(y-1)%img.shape[0], (x+1)%img.shape[1],0] - baseimg[(y+1)%img.shape[0], (x+1)%img.shape[1],0])
    
    Gy = 0.5*(+baseimg[(y-1)%img.shape[0], (x-1)%img.shape[1],0] + 2*baseimg[(y)%img.shape[0], (x-1)%img.shape[1],0] + baseimg[(y+1)%img.shape[0], (x-1)%img.shape[1],0]
          -baseimg[(y-1)%img.shape[0], (x+1)%img.shape[1],0] - 2*baseimg[(y)%img.shape[0], (x+1)%img.shape[1],0] - baseimg[(y+1)%img.shape[0], (x+1)%img.shape[1],0])

    # print(baseimg.shape)
    steepness_score = sqrt(Gx*Gx + Gy*Gy)*0.5

    # if x < baseimg.shape[0]-1 and y < baseimg.shape[0]-1:
    #     height = baseimg[y,x,0]
    #     dy = 0.5*(baseimg[y + 1, x,0] - baseimg[y - 1, x,0])
    #     dx = 0.5*(baseimg[y, x + 1,0] - baseimg[y, x - 1,0])

    #     steepness_score = sqrt(dx*dx + dy*dy)
    # else:
    #     steepness_score = 0

    dist = sqrt((x-body_pos[0])*(x-body_pos[0])+(y-body_pos[1])*(y-body_pos[1])+8*8*(baseimg[y,x,0]-body_pos[2])*(baseimg[y,x,0]-body_pos[2]))
    # radius = 15
    # stand_dev = 3
    body_proximity_score = max(sample_gaussian(height=1, offset=15, stand_dev=3.0, dist=dist)-0.1,0-pow(2.71828, -2*(dist+1.51*3-15))-0.1, 0)

    dist = sqrt((x-foot_pos[0])*(x-foot_pos[0])+(y-foot_pos[1])*(y-foot_pos[1]))
    foot_proximity_score = max(sample_gaussian(height=1, offset=0, stand_dev=3.0, dist=dist)-0.1,0)#-pow(2.71828, -2*(dist+1.51*stand_dev-radius))-0.1, 0)

    # temp[int(kernel_size/2), int(kernel_size/2)] = 1.0
    # img[r,c] = (kernel - abs(baseimg[wrap_slice(baseimg,0, (y-int(kernel_size/2))%baseimg.shape[0], (y+int(kernel_size/2))%baseimg.shape[0])][:,wrap_slice(baseimg,1,(x-int(kernel_size/2))%baseimg.shape[0], (x+int(kernel_size/2))%baseimg.shape[0])] - baseimg[x,y]).reshape((100,3)))
    # img[y,x] = 1.0

    mouseX,mouseY = x,y
    # return img[r,c].min()
    return steepness_score/2

def poll_value(event,x,y,flags,param):
    print(img[y,x])


# cv2.namedWindow('hmap', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('hmap', 1024, 1024)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 1024, 1024)

# gaussian = sample_gaussian(1,1,20)
# plt.plot(gaussian)
# plt.show()
search_rad = 8
def compare(y, x, minval):
    if img[y, x, 0] < minval:
        minval = img[y, x, 0]
        minx = x
        miny = y
        if minval < 0.1:
            return True, minx, miny, minval
    return False, x, y, minval

def circle_search(pos_x,pos_y,r,minval):
    # amount = 2*PI*r/
    r2 = r*r
    x =  1
    x_indices = np.array([0])
    y_indices = np.array([r])
    while x <= r*(sqrt(2)/2):
        i_x = int(x)
        i_y = int(np.round(np.sqrt(r2-x*x)))

        # Backfill skipped gridblocks
        # d = abs(i_y - y_indices[-1])
        # if (d > 1):
        #     x_indices = np.append( x_indices, np.full(d-1, i_x) )
        #     y_indices = np.append( y_indices, np.arange(y_indices[-1]-1, i_y,-1) )
        
        x_indices = np.append( x_indices, i_x )
        y_indices = np.append( y_indices, i_y )
        x += 1
    
    x_indices_temp = x_indices
    x_indices = np.append(x_indices, y_indices)
    y_indices = np.append(y_indices, x_indices_temp)

    # x_indices = np.arange(r) + 1
    # y_indices = (np.sqrt(r2-x_indices*x_indices)).astype(int)
    # # for i in range(np.arange(r) + 1):
    # #     y_indices[i]
    img[y_indices+pos_y, x_indices+pos_x] = [1,0,0]
    img[-y_indices+pos_y, -x_indices+pos_x] = [1,0,0]
    img[-y_indices+pos_y, x_indices+pos_x] = [1,0,0]
    img[y_indices+pos_y, -x_indices+pos_x] = [1,0,0]


def find_min(anchor_x, anchor_y):
    minval = 1000
    minx = anchor_x
    miny = anchor_y
    # circle_search(anchor_x, anchor_y,16,minval)
    # for i in range(16):
    #     circle_search(anchor_x, anchor_y,i,minval)
    
    for r in range(search_rad):
        for i in range(r+1):
            # Left side
            should_return, minx, miny, minval = compare(anchor_y+i, anchor_x-r, minval)
            if should_return: return minx, miny
            should_return, minx, miny, minval = compare(anchor_y-i, anchor_x-r, minval)
            if should_return: return minx, miny

            # Right side
            should_return, minx, miny, minval = compare(anchor_y+i, anchor_x+r, minval)
            if should_return: return minx, miny
            should_return, minx, miny, minval = compare(anchor_y-i, anchor_x+r, minval)
            if should_return: return minx, miny
            
            # Top side
            should_return, minx, miny, minval = compare(anchor_y+r, anchor_x+i, minval)
            if should_return: return minx, miny
            should_return, minx, miny, minval = compare(anchor_y+r, anchor_x-i, minval)
            if should_return: return minx, miny

            # Bottom side
            should_return, minx, miny, minval = compare(anchor_y-r, anchor_x+i, minval)
            if should_return: return minx, miny
            should_return, minx, miny, minval = compare(anchor_y-r, anchor_x-i, minval)
            if should_return: return minx, miny
    return minx, miny

def solve_anchor_point(anchor_x, anchor_y):
    minx, miny = find_min(anchor_x, anchor_y)

    # search_block = img[(anchor_y-search_rad):(anchor_y+search_rad), (anchor_x-search_rad):(anchor_x+search_rad)][:,:,0]
    # miny, minx = np.unravel_index(search_block.argmin(), search_block.shape)

    # Coloring
    # img[anchor_y, anchor_x] = [0,0,1]
    img[anchor_y+1, anchor_x] = [0,0,1]
    img[anchor_y, anchor_x+1] = [0,0,1]
    img[anchor_y-1, anchor_x] = [0,0,1]
    img[anchor_y, anchor_x-1] = [0,0,1]

    for i in range(search_rad*2+1):
        img[anchor_y-search_rad-1, anchor_x-search_rad+i] = [1,0,0]
        img[anchor_y+search_rad+1, anchor_x-search_rad+i] = [1,0,0]
        img[anchor_y-search_rad+i, anchor_x-1-search_rad] = [1,0,0]
        img[anchor_y-search_rad+i, anchor_x+1+search_rad] = [1,0,0]
    img[miny+1, minx] = [0,1,0]
    img[miny, minx+1] = [0,1,0]
    img[miny-1, minx] = [0,1,0]
    img[miny, minx-1] = [0,1,0]

def calc_points(event,x,y,flags,param):
    print(f"x: {x} y: {y} {img[y,x]}")

    # if event == cv2.EVENT_LBUTTONDOWN:
    #     img[:] = baseimg[:]
    #     r,c = wrap_block(img, (y-int((kernel_size)/2))%img.shape[0], (y+int((kernel_size)/2)+1)%img.shape[0], (x-int((kernel_size)/2))%img.shape[0], (x+int((kernel_size)/2)+1)%img.shape[0])
    #     img[r,c] =  kernel.reshape((kernel.size,1)) - abs(baseimg[r,c]-baseimg[y,x])
    #     img[y,x] = 1.0

    if event == cv2.EVENT_LBUTTONDOWN:
        img[:] = baseimg[:]
        for r in range(SIZE):
            for c in range(SIZE):
                score = calculate_score(np.array([65,65,10.6]), np.array([x,y,0]),c,r)
                # img[r,c] = score
                if score <= 0:
                    img[r,c] = [0,score,0]
                else:
                    img[r,c] = [score,score,score]
        # solve_anchor_point(63,48)
        # solve_anchor_point(123,57)
        # solve_anchor_point(135,100)
        # solve_anchor_point(45,100)
        # solve_anchor_point(60,152)
        # solve_anchor_point(130,140)
        cv2.imwrite("prox_score_test.png", img*255)
cv2.setMouseCallback('image',calc_points)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):
        print(mouseX,mouseY)
        print(img)