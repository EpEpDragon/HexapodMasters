import matplotlib.pyplot as plt
import numpy as np

cm = 1/2.54
textwidth = 13.98611*cm
normalsize = 12
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "Cambria",
    "font.size": normalsize
})

hmap0 = np.load('HMap0.npy')
hmap1 = np.load('HMap2.npy')
hmap2 = np.load('HMap4.npy')

fig = plt.figure()
plt.imshow(hmap0,vmax=0.6, cmap='plasma', extent=(-hmap0.shape[0]/2, hmap0.shape[0]/2, -hmap0.shape[1]/2, hmap0.shape[1]/2))
plt.xlabel('X(cm)')
plt.ylabel('Y(cm)')
plt.title('Heightmap')
plt.colorbar()
fig.set_figwidth(textwidth)
fig.savefig("hmap0.pdf", format="pdf", bbox_inches="tight")

fig = plt.figure()
plt.imshow(hmap1,vmax=0.6, cmap='plasma', extent=(-hmap0.shape[0]/2, hmap0.shape[0]/2, -hmap0.shape[1]/2, hmap0.shape[1]/2))
plt.xlabel('X(cm)')
plt.ylabel('Y(cm)')
plt.title('Heightmap')
plt.colorbar()
fig.set_figwidth(textwidth)
fig.savefig("hmap1.pdf", format="pdf", bbox_inches="tight")


fig = plt.figure()
plt.imshow(hmap2,vmax=0.6, cmap='plasma',extent=(-hmap0.shape[0]/2, hmap0.shape[0]/2, -hmap0.shape[1]/2, hmap0.shape[1]/2))
plt.xlabel('X(cm)')
plt.ylabel('Y(cm)')
plt.title('Heightmap')
plt.colorbar()
fig.set_figwidth(textwidth)
fig.savefig("hmap2.pdf", format="pdf", bbox_inches="tight")

plt.show()