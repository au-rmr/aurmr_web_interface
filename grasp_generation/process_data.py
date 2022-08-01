#%%
import h5py
from matplotlib import pyplot as plt, cm
from mpl_toolkits.axes_grid1 import ImageGrid
from matplotlib import colors
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
from aurmr_web_interface.optimize import simulated_annealing

import random
import simplejson as json
from itertools import chain
from aurmr_web_interface import utils
from functools import partial

#%%
filename = "datasets/train_shard_000000.h5"
f = h5py.File(filename, "r")

real_data = np.load("datasets/image_data.npy", allow_pickle=True).item()
#%%
# List all groups
print("Keys: %s" % f.keys())
for key in f.keys():
    print(f[key])
#%%
### Select 9 samples
# indices = random.choices(range(f["frame1_metadata"].shape[0]), k=9)
fake_data_indices = [2673, 33, 2205, 3734, 2594, 2426, 3044, 3529, 2427]
real_data_indices = [0, 1, 2, 3, 4, 5, 6, 7, 8]
imgs = []
imgs_depth = []
img_metadata = []

# for i in range(len(fake_data_indices)):
#     imgs.append(f["frame1_data"][fake_data_indices[i]][:, :, :3])
#     imgs_depth.append(f["frame1_depth"][fake_data_indices[i]])
#     img_metadata.append(json.loads(f["frame1_metadata"][fake_data_indices[i]])["camera"]["intrinsic_matrix"])

# print("Using fake data indices:", fake_data_indices)

for i in range(len(real_data_indices)):
    imgs.append(real_data["frame0_rgb"][real_data_indices[i]])
    imgs_depth.append(real_data["frame0_depth"][real_data_indices[i]])
    img_metadata.append(
        np.array(real_data["frame0_info"][real_data_indices[i]]["P"]).reshape(3, 4)[
            :, :3
        ]
    )

print("Using real data indices:", real_data_indices)
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
intrinsic_matrix = np.array(img_metadata[selected_img_idx])
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
o3d_pcd_selected = o3d_pcd_selected.crop(
    o3d.geometry.AxisAlignedBoundingBox(
        np.array([-0.2, -0.5, -10]), np.array([0.1, 0.5, 10])
    )
)
# vis = o3d.visualization.VisualizerWithEditing()
# vis.create_window()
# vis.add_geometry(o3d_pcd_selected)
# vis.run()  # user picks points
# vis.destroy_window()
# print(vis.get_picked_points())

# Fake data pts
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
selected_point_idx = 39837  # selected_pts[13]
selected_end_point_idx = 39344

# Real data pts
selected_point_idx = 85972 # 93960, 85972
selected_end_point_idx = 77570

pt_dist = (
    o3d_pcd_selected.points[selected_end_point_idx]
    - o3d_pcd_selected.points[selected_point_idx]
)
theta = np.arctan2(pt_dist[1], pt_dist[0])

print(
    o3d_pcd_selected.points[selected_point_idx],
    o3d_pcd_selected.points[selected_end_point_idx],
)

# %%
def objective_function(x, *args):
    pcd = args[0]
    indices = utils.mask_to_index(x)

    filtered_pcd = pcd.select_by_index(indices)

    filtered_convex_hull = filtered_pcd.compute_convex_hull()[0]
    filtered_convex_hull = filtered_convex_hull.compute_triangle_normals()

    normals = np.asarray(filtered_convex_hull.triangle_normals)
    ind = np.where(np.dot(normals, [0, 0, 1]) < 0.5)[0]
    filtered_convex_hull.remove_triangles_by_index(ind)

    filtered_convex_hull_pcd = filtered_convex_hull.sample_points_uniformly(
        len(filtered_pcd.points) * 10
    )

    # filtered_convex_hull_pcd.estimate_normals(
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    # )

    # normals = np.asarray(filtered_convex_hull_pcd.normals)
    # print(normals, np.array([[0, 0, 1]]*len(filtered_convex_hull_pcd.normals)))
    # dir_diff = np.dot(normals, np.array([[0, 0, 1]]*len(filtered_convex_hull_pcd.normals)))
    # ind = np.where(np.dot(normals, [0,0,-1]) < 0.5)[0]
    # filtered_convex_hull_pcd = filtered_convex_hull_pcd.select_by_index(ind)

    update_vis(filtered_pcd, filtered_convex_hull_pcd)

    # pcd_dist = filtered_pcd.compute_point_cloud_distance(filtered_convex_hull_pcd)
    pcd_dist = filtered_convex_hull_pcd.compute_point_cloud_distance(filtered_pcd)

    dist_term = np.sum(np.power(pcd_dist, 2))

    num_pts_term = len(pcd_dist)

    error = dist_term**2 * 5e2 + (-1e-2 * num_pts_term)

    print(error, end="\r")

    return error


def optimize(pcd, starting_pt):
    starting_guesses_idx = []
    starting_guess_mask = [0] * len(pcd.points)

    # TODO: use get_candidates here?
    for i in range(len(pcd.points)):
        if np.linalg.norm(pcd.points[starting_pt] - pcd.points[i]) < 1e-02:
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

    constraint = partial(utils.constrain_to_neighboring_pts, pcd=pcd, radius=0.01)

    result = simulated_annealing(
        objective_function,
        starting_guess_mask,
        [1] * len(o3d_pcd_selected.points),
        [0] * len(o3d_pcd_selected.points),
        constraint,
        n_iterations=100,
        step_size=10,
        temp=1000,
        args=(pcd,),
    )

    return result[0]


# # %%
# vis = o3d.visualization.VisualizerWithEditing()
# vis.create_window()
# vis.add_geometry(o3d_pcd_selected)
# vis.run()  # user picks points
# vis.destroy_window()

# selected_point_idx = vis.get_picked_points()[0]

# %%
viz_geo = o3d.geometry.PointCloud()
viz_geo.points = o3d_pcd_selected.points
viz_convex_hull_geo = o3d.geometry.PointCloud()


def update_vis(pcd, convex_hull):
    global viz_geo
    global viz_convex_hull_geo
    viz_geo.points = pcd.points
    viz_geo.colors = pcd.colors
    viz_geo.normals = pcd.normals

    viz_convex_hull_geo.points = convex_hull.points
    viz_convex_hull_geo.colors = convex_hull.colors
    viz_convex_hull_geo.normals = convex_hull.normals

    # vis.update_geometry(viz_geo)
    vis.update_geometry(viz_convex_hull_geo)
    vis.poll_events()
    vis.update_renderer()


vis = o3d.visualization.Visualizer()
vis.create_window()
ro = vis.get_render_option().point_show_normal = True

scale = 0.01
sphere = o3d.geometry.TriangleMesh.create_sphere(scale).translate(
    o3d_pcd_selected.points[selected_end_point_idx]
)
sphere += o3d.geometry.TriangleMesh.create_sphere(scale).translate(
    o3d_pcd_selected.points[selected_point_idx]
)
# vis.add_geometry(sphere)
# vis.add_geometry(
#     utils.generate_gripper(
#         thickness=scale,
#         depth=scale * 2,
#         width=np.linalg.norm(pt_dist[:2]),
#         color=[0, 0, 1],
#         trans=(
#             o3d_pcd_selected.points[selected_end_point_idx]
#             + o3d_pcd_selected.points[selected_point_idx]
#         )
#         / 2,
#         rot=Rotation.from_euler("zyx", [theta, 0, 0]).as_matrix(),
#     )
# )

# vis.add_geometry(viz_geo)
vis.add_geometry(o3d_pcd_selected)
vis.add_geometry(viz_convex_hull_geo)
mask = optimize(o3d_pcd_selected, selected_point_idx)

mask_pcd = o3d_pcd_selected.select_by_index(utils.mask_to_index(mask))

R = mask_pcd.get_rotation_matrix_from_xyz((0, 0, -theta))
center = mask_pcd.get_center()
mask_pcd_rotated = mask_pcd.rotate(R, center)

pcd_bbox = mask_pcd_rotated.get_axis_aligned_bounding_box()

max_bound = pcd_bbox.max_bound
min_bound = pcd_bbox.min_bound

R = mask_pcd.get_rotation_matrix_from_xyz((0, 0, theta))
center = mask_pcd.get_center()
pcd_bbox = pcd_bbox.get_oriented_bounding_box()
pcd_bbox.rotate(R, center)
vis.add_geometry(pcd_bbox)

object_width = max_bound[0] - min_bound[0]
object_center = pcd_bbox.get_center()
object_center[2] += 0.00008
object_rotation = theta

gripper_thickness = 0.000005
gripper_depth = 0.00005
gripper_width = object_width + gripper_thickness * 2

gripper_mesh = o3d.geometry.TriangleMesh()
gripper_mesh += o3d.geometry.TriangleMesh.create_box(
    width=gripper_width, height=gripper_thickness, depth=gripper_thickness
)
gripper_mesh += o3d.geometry.TriangleMesh.create_box(
    width=gripper_thickness, height=gripper_thickness, depth=gripper_depth
).translate((0, 0, -gripper_depth))
gripper_mesh += o3d.geometry.TriangleMesh.create_box(
    width=gripper_thickness, height=gripper_thickness, depth=gripper_depth
).translate((gripper_width, 0, -gripper_depth))
# gripper_mesh = gripper_mesh.scale(0.02, gripper_mesh.get_center())
gripper_mesh.compute_vertex_normals()
gripper_mesh.paint_uniform_color([1, 0, 0])

trans = gripper_mesh.translate(
    [object_center[0], object_center[1], object_center[2] - 0.000075], relative=False
)
trans = trans.rotate(pcd_bbox.R, trans.get_center())

vis.add_geometry(
    utils.generate_gripper(
        thickness=scale,
        depth=scale*3,
        width=object_width,
        color=[1, 0, 0],
        trans=[object_center[0], object_center[1], object_center[2] - 0.000075],
        rot=pcd_bbox.R,
    )
)

vis.run()
vis.destroy_window()

# %%
