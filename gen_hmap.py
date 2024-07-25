import cv2
import numpy as np
from voronoi import *

pil_img = generate(
    seed = 15321,
    width = 1080,
    height = 1080,
    regions = 40,
    colors = [(30, 30, 30), (23, 23, 23), (60, 60, 60), (80, 80, 80), (90, 90, 90)], #[(255, 255, 255)],
    # colors = [0.2,0.6,0.8,0.1],
    border_size = 10,
    background = "#000000",
    blur = 5,
    # color_algorithm = ColorAlgorithm.no_adjacent_same,
)
hmap = (np.array(pil_img)/255)[:,:,0]
cv2.imwrite("hmap.png", hmap*255)
