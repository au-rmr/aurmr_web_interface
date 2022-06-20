#%%
import h5py
from matplotlib import pyplot as plt, cm
from mpl_toolkits.axes_grid1 import ImageGrid
from matplotlib import colors
import numpy as np
import seaborn_image as isns
import random
import open3d as o3d

import simplejson as json

#%%
filename = "datasets/train_shard_000000.h5"
f = h5py.File(filename, "r")
#%%
# List all groups
print("Keys: %s" % f.keys())
for key in f.keys():
    print(f[key])
#%%
# indices = random.choices(range(f["frame1_metadata"].shape[0]), k=9)
indices = [2673, 33, 2205, 3734, 2594, 2426, 3044, 3529, 2427]
imgs = []
imgs_depth = []
img_metadata = []

for i in range(len(indices)):
    imgs.append(f["frame1_data"][indices[i]][:, :, :3])
    imgs_depth.append(f["frame1_depth"][indices[i]])
    img_metadata.append(json.loads(f["frame1_metadata"][indices[i]]))

print("Using indices:", indices)
# %%
color_grid = ImageGrid(
    plt.figure(figsize=(20.0, 20.0)),
    111,
    nrows_ncols=(3, 3),
    axes_pad=0.1,
)

for ax, im in zip(color_grid, imgs):
    ax.imshow(np.flipud(im))

plt.show()
# %%
depth_grid = ImageGrid(
    plt.figure(figsize=(20.0, 20.0)),
    111,
    nrows_ncols=(3, 3),
    axes_pad=0.1,
)

for ax, im in zip(depth_grid, imgs_depth):
    ax.imshow(np.flipud(im), cmap=cm.viridis_r, norm=colors.LogNorm())

plt.show()
# %%
selected_img_idx = 7
selected_depth_img = imgs_depth[selected_img_idx].copy()

# Remove the really low values
reasonable_min = selected_depth_img[selected_depth_img >= -1e+10].max()
selected_depth_img[selected_depth_img <= -1e+10] = reasonable_min

o3d_color_image = o3d.geometry.Image(imgs[selected_img_idx].astype(np.uint8))

depth_range = (selected_depth_img.max() - selected_depth_img.min()) / 255
remapped_depth_image = np.interp(selected_depth_img, (selected_depth_img.min(), selected_depth_img.max()), (0, 255)).astype(np.uint8)
# o3d_depth_image = o3d.geometry.Image(remapped_depth_image)
o3d_depth_image = o3d.geometry.Image(np.ascontiguousarray(selected_depth_img).astype(np.float32))

intrinsic_matrix = np.array(img_metadata[selected_img_idx]['camera']['intrinsic_matrix'])
o3d_intrinsic_matrix = o3d.camera.PinholeCameraIntrinsic()
o3d_intrinsic_matrix.intrinsic_matrix = intrinsic_matrix

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color_image, o3d_depth_image, convert_rgb_to_intensity = False)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic_matrix)

pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

o3d.visualization.draw_geometries([pcd])
    # %%

