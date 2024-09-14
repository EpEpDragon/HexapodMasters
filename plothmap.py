import matplotlib.pyplot as plt
import numpy as np

hmap0 = np.load('HMap0.npy')
hmap1 = np.load('HMap2.npy')
hmap2 = np.load('HMap4.npy')

plt.figure()
plt.cplot(hmap0,vmax=0.6, cmap='plasma')
plt.xlabel('')
plt.colorbar()

plt.figure()
plt.imshow(hmap1,vmax=0.6, cmap='plasma')
plt.colorbar()

plt.figure()
plt.imshow(hmap2,vmax=0.6, cmap='plasma')
plt.colorbar()
plt.show()