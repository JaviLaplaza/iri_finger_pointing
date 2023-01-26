#!/usr/bin/env python
from __future__ import print_function

import numpy as np

import roslib
roslib.load_manifest('iri_finger_pointing')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from iri_skeleton_msgs.msg import skeleton2DArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from dynamic_reconfigure.server import Server
from iri_finger_pointing.cfg import Reconfig_paramsConfig

class IRI_Finger_Pointing:
  def __init__(self):
    self._3Dpoint = rospy.Publisher("target_pose",PoseStamped)

    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,self.depth_callback)
    self.skeleton_sub = rospy.Subscriber("/landmarks", skeleton2DArray, self.skeleton_callback)
    self.camera_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
    # cam_inf=rospy.wait_for_message("/camera/color/camera_info", CameraInfo)

    self.depth_image = []
    self.finger_tip = np.array([0., 0., 0.])
    self.goal_msg = PoseStamped()

    self.central_pixel = np.array([])
    srv = Server(Reconfig_paramsConfig, self.reconfigure_callback)
    # rospy.set_param('/set_point', False)
    self.set_point="False"
  def reconfigure_callback(self, config,level):
    self.set_point = ("{select_point}".format(**config))
    config.__setitem__("select_point",False)
    return config
  def depth_callback(self, data):
    # self.central_pixel = np.array([data.shape[0], data.shape[1]])

    try:
      # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      flatten_depth_img = np.frombuffer(data.data, dtype=np.uint16)  # shape =(width*height,)
      depth_img = flatten_depth_img.reshape(data.height, data.width) # shape =(width, height)
      # print(cv_image)

      self.depth_image = depth_img
    except CvBridgeError as e:
      print(e)

  def skeleton_callback(self, data):
    self.finger_tip[0] = data.skeletons[0].skeleton_points[8].pose.x
    self.finger_tip[1] = data.skeletons[0].skeleton_points[8].pose.y

    wrist = np.array([data.skeletons[0].skeleton_points[0].pose.x, data.skeletons[0].skeleton_points[0].pose.y, 0])
    hand_direction = self.finger_tip - wrist

    depth_windows_height = int(self.finger_tip[1]-3), int(self.finger_tip[1]+3)



    if hand_direction[1] > 0:
      depth_windows_width = int(self.finger_tip[0]), int(self.finger_tip[0]+5)
      self.finger_tip[0] += 3
      # print(depth_windows_height[0], depth_windows_height[1], depth_windows_width[1], depth_windows_width[0])
      # print(self.depth_image.shape)
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[0]:depth_windows_width[1]]

    else:
      depth_windows_width = int(self.finger_tip[0]), int(self.finger_tip[0]-5)
      self.finger_tip[0] -= 3
      # print(depth_windows_height[0], depth_windows_height[1], depth_windows_width[1], depth_windows_width[0])
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[1]:depth_windows_width[0]]
    # print(goal)

    depth_goal = np.nanmean(goal)

    self.finger_tip[2] = depth_goal/1000.

    # print(self.finger_tip)

    # Transform point from pixel coordinates to world coordinates
    self.finger_tip[0], self.finger_tip[1] = self.pixels_to_camera_coord(self.K, self.finger_tip[0], self.finger_tip[1], self.finger_tip[2])

    posemsg = PoseStamped()
    posemsg.header.frame_id="wrist_camera"
    posemsg.header.stamp=rospy.get_rostime()

    posemsg.pose.position.x = round(self.finger_tip[0], 2)
    posemsg.pose.position.y = round(self.finger_tip[1], 2)
    posemsg.pose.position.z = round(self.finger_tip[2], 2)
    posemsg.pose.orientation.x = 0.
    posemsg.pose.orientation.y = 0.
    posemsg.pose.orientation.z = 0.
    posemsg.pose.orientation.w = 1.

    self.goal_msg = posemsg

    if self.set_point=="True":
      self._3Dpoint.publish(self.goal_msg)
      self.set_point="False"

  def camera_info_callback(self, data):
    self.K = data.K

  def cross_fingers(self):
    print()

  def pixels_to_camera_coord(self, K, x, y, z):
    x_real = (z / K[0]) * (x - K[2])
    y_real = (z / K[4]) * (y - K[5])

    return x_real, y_real

def main(args):
  rospy.init_node('iri_finger_pointing', anonymous=True)
  ic = IRI_Finger_Pointing()


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
