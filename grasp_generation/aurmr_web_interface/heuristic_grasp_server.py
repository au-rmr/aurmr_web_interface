#!/usr/bin/env python

from __future__ import print_function

from aurmr_web_interface.srv import (
    GenerateHeuristicGrasp,
    GenerateHeuristicGraspResponse,
)
from sensor_msgs.msg import CameraInfo, Image
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import rospy
import tf
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation

from aurmr_web_interface.utils import (
    create_o3d_pcd,
    get_candidates,
    optimize,
    mask_to_index,
    index_to_mask,
)

bridge = CvBridge()

camera_intrinsics = []
color = []
depth = []

pcd = None
time_since_last_pcd = 0

pcd_geo = o3d.geometry.PointCloud()
pcd_convex_hull = o3d.geometry.PointCloud()
bbox = o3d.geometry.OrientedBoundingBox()

viz = o3d.visualization.Visualizer()
viz.create_window()
viz.add_geometry(pcd_convex_hull)
viz.add_geometry(bbox)

tf_listener = None
pose_publisher = rospy.Publisher('heuristic_grasp_pose', PoseStamped, queue_size=1)

def camera_info_callback(camera_info):
    global camera_intrinsics
    camera_intrinsics = np.reshape(camera_info.P, (3, 4))


def color_image_callback(color_image):
    global color
    color = bridge.imgmsg_to_cv2(color_image, "rgb8")[40:760]
    # cv2.imshow("color", color)
    # cv2.waitKey(1)


def depth_image_callback(depth_image):
    global depth
    global pcd
    global time_since_last_pcd
    depth = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
    # cv2.imshow("depth", depth / 500)
    # cv2.waitKey(1)

    if (len(camera_intrinsics) > 0 and len(color) > 0) and len(depth) > 0:
        if rospy.get_time() - time_since_last_pcd >= 1:
            pcd = create_o3d_pcd(color, depth, camera_intrinsics, add_noise=True)
            time_since_last_pcd = rospy.get_time()


def handle_generate_heuristic_grasp(req):
    print(req.se2.x, req.se2.y, req.se2.theta)
    xcoord = int(depth.shape[1] * req.se2.x)
    ycoord = int(depth.shape[0] * req.se2.y)
    print(xcoord, ycoord)

    selected_point = depth.shape[1] * ycoord + xcoord

    bbox.clear()
    downsampled = pcd.voxel_down_sample(5e-3)
    pcd_geo.points = downsampled.points
    pcd_geo.colors = downsampled.colors
    # pcd_geo.colors[selected_point] = [1,1,1]

    # for candidate in get_candidates(pcd_geo, [selected_point], max_dist=5e-3):
    #     pcd_geo.colors[candidate] = [1,1,1]

    # pcd_geo.clear()

    def viz_convex_hull(updated_pcd):
        pcd_convex_hull.points = updated_pcd.points
        pcd_convex_hull.colors = updated_pcd.colors
        print("updated viz")

    closes_pt_idx = 0
    for pt in range(len(downsampled.points)):
        if np.linalg.norm(
            downsampled.points[closes_pt_idx] - pcd.points[selected_point]
        ) > np.linalg.norm(downsampled.points[pt] - pcd.points[selected_point]):
            closes_pt_idx = pt

    mask = optimize(
        downsampled,
        closes_pt_idx,
        starting_search_range=1e-2,
        constrain_range=2e-2,
        viz_function=viz_convex_hull,
    )
    mask_pcd = downsampled.select_by_index(mask_to_index(mask))

    R = mask_pcd.get_rotation_matrix_from_xyz((0, 0, -req.se2.theta))
    center = mask_pcd.get_center()
    mask_pcd_rotated = mask_pcd.rotate(R, center)

    # pcd_geo.clear()
    # viz_convex_hull(mask_pcd_rotated.paint_uniform_color([0, 0, 1]))

    pcd_bbox = mask_pcd_rotated.get_axis_aligned_bounding_box()

    max_bound = pcd_bbox.max_bound
    min_bound = pcd_bbox.min_bound

    R = mask_pcd.get_rotation_matrix_from_xyz((0, 0, req.se2.theta))
    center = mask_pcd.get_center()
    pcd_bbox = pcd_bbox.get_oriented_bounding_box()
    pcd_bbox.rotate(R, center)
    bbox.center = pcd_bbox.center
    bbox.color = pcd_bbox.color
    bbox.extent = pcd_bbox.extent
    bbox.R = pcd_bbox.R

    object_width = max_bound[0] - min_bound[0]
    object_center = pcd_bbox.get_center()
    # object_center[1] -= 0.5
    object_rotation = req.se2.theta

    response = GenerateHeuristicGraspResponse()
    response.grasp.width = object_width


    r = Rotation.from_euler('zyx', [0, -90, 0], degrees=True)
    q = r.as_quat()

    p = PoseStamped()
    p.header.frame_id = "camera_wrist_depth_optical_frame"
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    p.pose.position.x = object_center[0]
    p.pose.position.y = -object_center[1]
    p.pose.position.z = -object_center[2]

    p = tf_listener.transformPose('world', p)

    pose_publisher.publish(p)

    response.grasp.pose.position = p.pose.position
    response.grasp.pose.orientation = p.pose.orientation

    return response


def generate_heuristic_grasp_server():
    global time_since_last_pcd
    global tf_listener
    rospy.init_node("generate_heuristic_grasp_server")

    tf_listener = tf.TransformListener()

    time_since_last_pcd = rospy.get_time()

    rospy.Service(
        "generate_heuristic_grasp",
        GenerateHeuristicGrasp,
        handle_generate_heuristic_grasp,
    )

    rospy.Subscriber(
        "/camera_wrist/depth/camera_info", CameraInfo, camera_info_callback
    )
    rospy.Subscriber(
        "/camera_wrist/color/image_raw", numpy_msg(Image), color_image_callback
    )
    rospy.Subscriber(
        "/camera_wrist/depth/image_raw", numpy_msg(Image), depth_image_callback
    )

    first_time = True

    while not rospy.is_shutdown():
        if pcd:
            if first_time:
                pcd_geo.points = pcd.points
                pcd_geo.colors = pcd.colors
                first_time = False
                viz.add_geometry(pcd_geo)

            viz.update_geometry(pcd_geo)
            viz.update_geometry(pcd_convex_hull)
            viz.update_geometry(bbox)
        viz.poll_events()
        viz.update_renderer()
    viz.destroy_window()


if __name__ == "__main__":
    generate_heuristic_grasp_server()
