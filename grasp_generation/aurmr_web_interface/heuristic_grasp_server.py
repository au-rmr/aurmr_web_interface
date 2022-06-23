#!/usr/bin/env python

from __future__ import print_function

from aurmr_web_interface.srv import (
    GenerateHeuristicGrasp,
    GenerateHeuristicGraspResponse,
)
from sensor_msgs.msg import CameraInfo, Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
import numpy as np
import rospy
import open3d as o3d
import cv2

from aurmr_web_interface.utils import create_o3d_pcd

bridge = CvBridge()

camera_intrinsics = []
color = []
depth = []

pcd = None
time_since_last_pcd = 0

pcd_geo = o3d.geometry.PointCloud()
viz = o3d.visualization.Visualizer()
viz.create_window()


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
            pcd = create_o3d_pcd(color, depth, camera_intrinsics)
            time_since_last_pcd = rospy.get_time()


def handle_generate_heuristic_grasp(req):
    print(req.se2.x, req.se2.y, req.se2.theta)
    return GenerateHeuristicGraspResponse(result="test")


def generate_heuristic_grasp_server():
    global time_since_last_pcd
    rospy.init_node("generate_heuristic_grasp_server")

    time_since_last_pcd = rospy.get_time()

    rospy.Service(
        "generate_heuristic_grasp",
        GenerateHeuristicGrasp,
        handle_generate_heuristic_grasp,
    )

    rospy.Subscriber(
        "/camera_wrist/color/camera_info", CameraInfo, camera_info_callback
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
            pcd_geo.points = pcd.points
            pcd_geo.colors = pcd.colors

            if first_time:
                viz.add_geometry(pcd_geo)
                first_time = False
            else:
                viz.update_geometry(pcd_geo)
                
        viz.poll_events()
        viz.update_renderer()
    viz.destroy_window()


if __name__ == "__main__":
    generate_heuristic_grasp_server()
