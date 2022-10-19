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


class IRI_Finger_Point:
  def __init__(self):
    self.3Dpoint = rospy.Publisher("output_3D_point",PoseArray)

    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/head_camera/aligned_depth_to_color/image_raw", Image,self.depth_callback)
    self.skeleton_sub = rospy.Subscriber("/landmarks", skeleton2DArray, self.skeleton_callback)

    self.depth_image = []
    self.finger_tip = []
    self.goal_msg = PoseArray()

  def depth_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      self.depth_image = cv_image
    except CvBridgeError as e:
      print(e)

  def skeleton_callback(self, data):
    self.finger_tip[0, 1] = data[8, 0:2]
    wrist = data[0, 0:2]
    hand_direction = self.finger_tip - wrist

    depth_windows_height = self.finger_tip[0]-3, self.finger_tip[0]+3


    if hand_direction[1] > 0:
      depth_windows_width = self.finger_tip[1], self.finger_tip[1]+5
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[0], depth_windows_width[1]]

    else:
      depth_windows_width = self.finger_tip[1], self.finger_tip[1]-5
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[0], depth_windows_width[1]]

    depth_goal = np.nanmean(goal)

    self.finger_tip[2] = depth_goal

    self.goal_msg.poses[0].position.x = self.finger_tip[0]
    self.goal_msg.poses[0].position.y = self.finger_tip[1]
    self.goal_msg.poses[0].position.z = self.finger_tip[2]

    self.3Dpoint.publish(self.goal_msg)


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
