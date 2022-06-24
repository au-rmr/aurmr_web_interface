import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from itertools import chain
from aurmr_web_interface.optimize import simulated_annealing
from functools import partial
from time import time
from time import perf_counter
from contextlib import contextmanager

@contextmanager
def catchtime() -> float:
    start = perf_counter()
    yield lambda: perf_counter() - start

def timer_func(func):
    # This function shows the execution time of 
    # the function object passed
    def wrap_func(*args, **kwargs):
        t1 = time()
        result = func(*args, **kwargs)
        t2 = time()
        print(f'Function {func.__name__!r} executed in {(t2-t1):.4f}s')
        return result
    return wrap_func

@timer_func
def mask_to_index(mask):
    idx = []
    for i in range(len(mask)):
        if mask[i] > 0.5:
            idx.append(i)
    return idx


@timer_func
def index_to_mask(idx, mask_len):
    mask = [0] * mask_len

    for i in idx:
        mask[i] = 1

    return mask


def create_o3d_pcd(color_img, depth_img, camera_intrinsics, add_noise=False):
    o3d_color_img = o3d.geometry.Image(color_img.astype(np.uint8))
    o3d_depth_img = o3d.geometry.Image(depth_img.astype(np.float32))

    o3d_camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
    o3d_camera_intrinsics.intrinsic_matrix = camera_intrinsics[:, :3]

    o3d_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color_img, o3d_depth_img, convert_rgb_to_intensity=False
    )

    o3d_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        o3d_rgbd_image, o3d_camera_intrinsics
    )

    if add_noise:
        r = np.random.rand(len(o3d_pcd.points), 3) * 1e-10
        o3d_pcd.points = o3d.utility.Vector3dVector(np.asarray(o3d_pcd.points) + r)

    return o3d_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


@timer_func
def constrain_to_neighboring_pts(curr, prev, pcd, radius=5e-6, kdtree=None):
    c = np.array(curr).astype(int)
    p = np.array(
        index_to_mask(get_candidates(pcd, mask_to_index(prev), max_dist=radius, kdtree=kdtree), len(prev))
    ).astype(int)
    return np.bitwise_and(c, p).tolist()


@timer_func
def get_candidates(pcd, guess_idxs, max_dist=5e-6, kdtree=None):
    guess_pts = np.asarray(pcd.points)[guess_idxs]
    guess_pts = tuple(map(list, guess_pts))

    if not kdtree:
        tree = KDTree(pcd.points)
    else:
        tree = kdtree

    with catchtime() as t:
        results = tree.query_ball_point(guess_pts, max_dist, workers=-1).tolist()
        print("queried tree", t())

    return list(set(chain(*results)))


@timer_func
def objective_function(x, *args):
    pcd = args[0]
    viz_function = args[1]
    indices = mask_to_index(x)

    filtered_pcd = pcd.select_by_index(indices)
    print("filtered pcd based on input:", len(filtered_pcd.points))

    filtered_convex_hull = filtered_pcd.compute_convex_hull()[0]
    filtered_convex_hull_pcd = filtered_convex_hull.sample_points_uniformly(
        len(filtered_pcd.points) * 5
    )
    print("generated convex hull")

    viz_function(filtered_convex_hull_pcd)

    pcd_dist = filtered_pcd.compute_point_cloud_distance(filtered_convex_hull_pcd)

    dist_term = np.sum(np.power(pcd_dist, 2))

    num_pts_term = len(pcd_dist)

    error = dist_term * 1000 + (-8e-3 * num_pts_term)
    print("calculated error:", error)

    # print(error, end="\r")

    return error

@timer_func
def optimize(pcd, starting_pt, starting_search_range=1e-5, constrain_range=1e-3, viz_function=None):
    starting_guess_mask = index_to_mask(
        get_candidates(pcd, [starting_pt], max_dist=starting_search_range), len(pcd.points)
    )

    tree = KDTree(pcd.points)

    constraint = partial(constrain_to_neighboring_pts, pcd=pcd, radius=constrain_range, kdtree=tree)
    
    result = simulated_annealing(
        objective_function,
        starting_guess_mask,
        [1] * len(pcd.points),
        [0] * len(pcd.points),
        constraint,
        n_iterations=5,
        step_size=10,
        temp=1000,
        args=(pcd, viz_function),
    )
    
    return result[0]
