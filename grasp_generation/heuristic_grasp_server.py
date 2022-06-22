#!/usr/bin/env python

from __future__ import print_function

from aurmr_web_interface.srv import GenerateHeuristicGrasp, GenerateHeuristicGraspResponse
import rospy

# Topics that we need to listen to
# /camera_wrist/color/camera_info - sensor_msgs/CameraInfo (p is the intrinsic matrix)
# /camera_wrist/color/image_raw - sensor_msgs/Image
# /camera_wrist/depth/image_raw - sensor_msgs/Image

def handle_generate_heuristic_grasp(req):
    print(req.se2.x, req.se2.y, req.se2.theta)
    return GenerateHeuristicGraspResponse(result="test")

def generate_heuristic_grasp_server():
    rospy.init_node('generate_heuristic_grasp_server')
    s = rospy.Service('generate_heuristic_grasp', GenerateHeuristicGrasp, handle_generate_heuristic_grasp)
    rospy.spin()

if __name__ == "__main__":
    generate_heuristic_grasp_server()