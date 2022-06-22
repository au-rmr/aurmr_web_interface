#%%
import h5py
from matplotlib import pyplot as plt, cm
from mpl_toolkits.axes_grid1 import ImageGrid
from matplotlib import colors
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from optimize import simulated_annealing

import random
import simplejson as json
from itertools import chain
import utils
from functools import partial

#%%
filename = "datasets/train_shard_000000.h5"
f = h5py.File(filename, "r")
#%%
# List all groups
print("Keys: %s" % f.keys())
for key in f.keys():
    print(f[key])
#%%
### Select 9 samples
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
### Visualize the sample RGB data
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
### Visualize the sample depth data
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
# Pick one of the 9 samples
selected_img_idx = 7
selected_depth_img = imgs_depth[selected_img_idx].copy()

# Convert the RGB and D images to Open3D format
o3d_color_image = o3d.geometry.Image(imgs[selected_img_idx].astype(np.uint8))
o3d_depth_image = o3d.geometry.Image(
    np.ascontiguousarray(selected_depth_img).astype(np.float32)
)

# Get the camera intrinsics and convert to Open3D format
intrinsic_matrix = np.array(
    img_metadata[selected_img_idx]["camera"]["intrinsic_matrix"]
)
o3d_intrinsic_matrix = o3d.camera.PinholeCameraIntrinsic()
o3d_intrinsic_matrix.intrinsic_matrix = intrinsic_matrix

# Combine the color and depth images into RGBD and generate a point cloud
o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d_color_image, o3d_depth_image, convert_rgb_to_intensity=False
)
o3d_pcd_selected = o3d.geometry.PointCloud.create_from_rgbd_image(
    o3d_rgbd, o3d_intrinsic_matrix
)

# Flip the point cloud so it is facing the right way
o3d_pcd_selected.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#%%
# vis = o3d.visualization.VisualizerWithEditing()
# vis.create_window()
# vis.add_geometry(o3d_pcd_selected)
# vis.run()  # user picks points
# vis.destroy_window()
# print(vis.get_picked_points())

selected_pts = [
    30911,
    31186,
    39893,
    45259,
    39872,
    30366,
    30635,
    34974,
    34726,
    39335,
    42144,
    38814,
    37039,
    23430,
    24973,
    31884,
]

# Pick a specific starting point
selected_point_idx = selected_pts[13]  # 39880
print(o3d_pcd_selected.points[selected_point_idx])

# %%
def objective_function(x, *args):
    pcd = args[0]
    indices = utils.mask_to_index(x)

    filtered_pcd = pcd.select_by_index(indices)

    filtered_convex_hull = filtered_pcd.compute_convex_hull()[0]
    filtered_convex_hull_pcd = filtered_convex_hull.sample_points_uniformly(
        len(filtered_pcd.points) * 10
    )

    update_vis(filtered_pcd, filtered_convex_hull_pcd)

    pcd_dist = filtered_pcd.compute_point_cloud_distance(filtered_convex_hull_pcd)

    dist_term = np.sum(np.power(pcd_dist, 2))

    num_pts_term = len(pcd_dist)

    error = dist_term * 1000 + (-5e-9 * num_pts_term)

    print(error, end="\r")

    return error


def constrain_to_neighboring_pts(curr, prev, pcd):
    c = np.array(curr).astype(int)
    p = np.array(
        utils.index_to_mask(get_candidates(pcd, utils.mask_to_index(prev)), len(prev))
    ).astype(int)
    return np.bitwise_and(c, p).tolist()


def get_candidates(pcd, guess_idxs, max_dist=5e-6):
    guess_pts = np.asarray(pcd.points)[guess_idxs]
    guess_pts = tuple(map(list, guess_pts))

    tree = KDTree(pcd.points)
    results = tree.query_ball_point(guess_pts, max_dist, workers=-1).tolist()

    return list(set(chain(*results)))


def optimize(pcd, starting_pt):
    starting_guesses_idx = []
    starting_guess_mask = [0] * len(pcd.points)

    # TODO: use get_candidates here?
    for i in range(len(pcd.points)):
        if np.linalg.norm(pcd.points[starting_pt] - pcd.points[i]) < 1e-05:
            starting_guesses_idx.append(i)
            starting_guess_mask[i] = 1

    # objective_function(starting_guess_mask)
    # o3d.visualization.draw_geometries(
    #     [
    #         pcd.select_by_index(starting_guesses_idx),
    #         pcd.select_by_index(
    #             np.setdiff1d(get_candidates(starting_guesses_idx), starting_guesses_idx)
    #         ).paint_uniform_color([1, 0, 0]),
    #     ]
    # )

    constraint = partial(constrain_to_neighboring_pts, pcd=pcd)

    simulated_annealing(
        objective_function,
        starting_guess_mask,
        [1] * len(o3d_pcd_selected.points),
        [0] * len(o3d_pcd_selected.points),
        constraint,
        n_iterations=100,
        step_size=10,
        temp=1000,
        args=(pcd,)
    )


# %%
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(o3d_pcd_selected)
vis.run()  # user picks points
vis.destroy_window()

selected_point_idx = vis.get_picked_points()[0]

# %%
viz_geo = o3d.geometry.PointCloud()
viz_geo.points = o3d_pcd_selected.points
viz_convex_hull_geo = o3d.geometry.PointCloud()


def update_vis(pcd, convex_hull):
    global viz_geo
    global viz_convex_hull_geo
    viz_geo.points = pcd.points
    viz_geo.colors = pcd.colors

    viz_convex_hull_geo.points = convex_hull.points
    viz_convex_hull_geo.colors = convex_hull.colors

    # vis.update_geometry(viz_geo)
    vis.update_geometry(viz_convex_hull_geo)
    vis.poll_events()
    vis.update_renderer()


vis = o3d.visualization.Visualizer()
vis.create_window()
# vis.add_geometry(viz_geo)
vis.add_geometry(o3d_pcd_selected)
vis.add_geometry(viz_convex_hull_geo)
optimize(o3d_pcd_selected, selected_point_idx)

vis.run()
vis.destroy_window()
# %%

# %%
