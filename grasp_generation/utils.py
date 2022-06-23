import numpy as np
import open3d as o3d


def mask_to_index(mask):
    idx = []
    for i in range(len(mask)):
        if mask[i] > 0.5:
            idx.append(i)
    return idx


def index_to_mask(idx, mask_len):
    mask = [0] * mask_len

    for i in idx:
        mask[i] = 1

    return mask


def create_o3d_pcd(color_img, depth_img, camera_intrinsics):
    o3d_color_img = o3d.geometry.Image(color_img.astype(np.uint8))
    o3d_depth_img = o3d.geometry.Image(depth_img.astype(np.float32))

    o3d_camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
    o3d_camera_intrinsics.intrinsic_matrix = camera_intrinsics

    o3d_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color_img, o3d_depth_img, convert_rgb_to_intensity=False
    )

    o3d_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        o3d_rgbd_image, o3d_camera_intrinsics
    )

    return o3d_pcd
