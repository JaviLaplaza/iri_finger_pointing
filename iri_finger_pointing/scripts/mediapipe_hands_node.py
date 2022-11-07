#!/usr/bin/env python
from __future__ import print_function

import numpy as np

import roslib
roslib.load_manifest('iri_finger_point')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from iri_skeleton_msgs import skeleton2DArray
from geometry_msgs import PoseArray
from sensors_msgs import CameraInfo


class IRI_Finger_Point:
  def __init__(self):
    self.3Dpoint = rospy.Publisher("output_3D_point",PoseArray)

    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/head_camera/aligned_depth_to_color/image_raw", Image,self.depth_callback)
    self.skeleton_sub = rospy.Subscriber("/landmarks", skeleton2DArray, self.skeleton_callback)
    self.camera_info = rospy.Subscriber("/camera_info", CameraInfo, self.camera_info_callback)

    self.depth_image = []
    self.finger_tip = []
    self.goal_msg = PoseArray()

    self.central_pixel = np.array([])
    rospy.set_param('set_point', False)

  def depth_callback(self, data):
    if not np.any(self.central_pixel):

      # print(data.shape)
      self.central_pixel = np.array([data.shape[0], data.shape[1]])

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      self.depth_image = cv_image
    except CvBridgeError as e:
      print(e)

  def skeleton_callback(self, data):
    self.finger_tip[0, 1] = data[8, 0:2]
    wrist = data[0, 0:2]
    hand_direction = self.finger_tip - wrist

    depth_windows_height = self.finger_tip[1]-3, self.finger_tip[1]+3



    if hand_direction[0] > 0:
      depth_windows_width = self.finger_tip[0], self.finger_tip[0]+5
      self.finger_tip[0] += 3
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[0], depth_windows_width[1]]

    else:
      depth_windows_width = self.finger_tip[0], self.finger_tip[0]-5
      self.finger_tip[0] -= 3
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[0], depth_windows_width[1]]

    depth_goal = np.nanmean(goal)

    self.finger_tip[2] = depth_goal

    # Transform point from pixel coordinates to world coordinates
    self.finger_tip[0], self.finger_tip[1] = self.pixels_to_camera_coord(self.K, self.finger_tip[0], self.finger_tip[1], self.finger_tip[2])

    self.goal_msg.poses[0].position.x = self.finger_tip[0]
    self.goal_msg.poses[0].position.y = self.finger_tip[1]
    self.goal_msg.poses[0].position.z = self.finger_tip[2]

    if rospy.get_param("set_point"):
      self.3Dpoint.publish(self.goal_msg)
      rospy.set_param('set_point', False)

  def camera_info_callback(self, data):
    self.K = data.K

  def cross_fingers(self):
    print()

  def pixels_to_camera_coord(self, K, x, y, z):
    x_real = (z / K[0]) * (x - K[2])
    y_real = (z / K[4]) * (y - K[5])

    return x_real, y_real

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
