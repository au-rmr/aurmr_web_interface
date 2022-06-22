#!/usr/bin/env python

from __future__ import print_function

from aurmr_web_interface.srv import GenerateHeuristicGrasp, GenerateHeuristicGraspResponse
from sensor_msgs.msg import CameraInfo, Image
from rospy.numpy_msg import numpy_msg
import numpy as np
import rospy

camera_intrinsics = []
color = []
depth = []

def camera_info_callback(camera_info):
    global camera_intrinsics
    camera_intrinsics = np.reshape(camera_info.P, (3,4))


def color_image_callback(color_image):
    global color
    color = np.frombuffer(color_image.data, dtype=np.uint8).reshape(color_image.height, color_image.width, -1)

def depth_image_callback(depth_image):
    global depth
    depth = np.frombuffer(depth_image.data, dtype=np.uint8).reshape(depth_image.height, depth_image.width, -1)

def handle_generate_heuristic_grasp(req):
    print(req.se2.x, req.se2.y, req.se2.theta)
    return GenerateHeuristicGraspResponse(result="test")

def generate_heuristic_grasp_server():
    rospy.init_node('generate_heuristic_grasp_server')

    rospy.Service('generate_heuristic_grasp', GenerateHeuristicGrasp, handle_generate_heuristic_grasp)

    rospy.Subscriber("/camera_wrist/color/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/camera_wrist/color/image_raw", numpy_msg(Image), color_image_callback)
    rospy.Subscriber("/camera_wrist/depth/image_raw", numpy_msg(Image), depth_image_callback)
    
    rospy.spin()

if __name__ == "__main__":
    generate_heuristic_grasp_server()