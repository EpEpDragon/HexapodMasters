import numpy as np

EXTENTS = 16                        # Extents of SDF block, in distance units
DIVISIOINS = 16                      # Cells per distance unit
INV_DIVISIOINS = 1.0/DIVISIOINS                # For faster computation
HMAP_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells


hmap_index = np.array([0,255])

def global_to_hmap(global_pos):
    """Convert global position to position in hmap grid, which has its corner at 0,0,0"""
    return (((global_pos)%(EXTENTS))*DIVISIOINS).astype(np.int32)

    
def _local_to_hmap(local_pos):
    temp = (np.round(global_to_hmap(-local_pos)) - hmap_index + int(HMAP_EXTENTS*0.5))
    return  temp % HMAP_EXTENTS, temp

def _hmap_to_local(hmap_pos_raw):
    diff = hmap_pos_raw + hmap_index - int(HMAP_EXTENTS*0.5)
    
    x_new = 0
    y_new = 0
    
    if (diff[0]) > HMAP_EXTENTS*0.5: 
        x_new = -INV_DIVISIOINS*( diff[0] % -HMAP_EXTENTS )
    else: 
        x_new = -INV_DIVISIOINS*( diff[0] % HMAP_EXTENTS )

    if (diff[1]) > HMAP_EXTENTS*0.5: 
        y_new = -INV_DIVISIOINS*( diff[1] % -HMAP_EXTENTS )
    else: 
        y_new = -INV_DIVISIOINS*( diff[1] % HMAP_EXTENTS )

    return np.array([x_new, y_new])
    
    


init_pos = np.array([2.195,-0.911])
# for x in range(HMAP_EXTENTS):
#     for y in range(HMAP_EXTENTS):
#         hmap_index[0] = x
#         hmap_index[1] = y
hmap_pos, hmap_pos_raw = _local_to_hmap(init_pos)

pos = _hmap_to_local(hmap_pos_raw)
        # if (pos != init_pos).any():
print("hmap", hmap_index, hmap_pos)
print("result", hmap_index, pos)
print(hmap_index-hmap_pos)