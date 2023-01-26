#!/usr/bin/env python
from __future__ import print_function

import numpy as np

import roslib
roslib.load_manifest('iri_finger_pointing')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from iri_skeleton_msgs.msg import skeleton2DArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CameraInfo
from dynamic_reconfigure.server import Server
from iri_finger_pointing.cfg import Reconfig_paramsConfig
from apriltag_ros.msg import AprilTagDetectionArray


from tree_transform.msg import PoseStampedArray,PoseStampedCustom # importing custom message PoseStampedArray
from tree_transform.srv import tree_transformService, tree_transformServiceResponse # importing the srv file top and bottom
from tree_transform.srv import database_manager_load, database_manager_update, database_manager_clean, database_manager_upload, database_manager_get_poses


class IRI_Finger_Pointing:
  def __init__(self):
    self._3Dpoint = rospy.Publisher("~target_pose",PoseStamped)

    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("~camera/aligned_depth_to_color/image_raw", Image,self.depth_callback)
    self.skeleton_sub = rospy.Subscriber("~landmarks", skeleton2DArray, self.skeleton_callback)
    self.camera_info = rospy.Subscriber("~camera/color/camera_info", CameraInfo, self.camera_info_callback)
    self.detect_sub = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.detection_callback)
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
    self.data_d=data
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

    depth_windows_height = int(self.finger_tip[1]-5), int(self.finger_tip[1]+5)


    #rospy.logwarn("Skel callback")
    if hand_direction[1] > 0:
      depth_windows_width = int(self.finger_tip[0]), int(self.finger_tip[0]+10)
      self.finger_tip[0] += 3
      # print(depth_windows_height[0], depth_windows_height[1], depth_windows_width[1], depth_windows_width[0])
      # print(self.depth_image.shape)
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[0]:depth_windows_width[1]]

    else:
      depth_windows_width = int(self.finger_tip[0]), int(self.finger_tip[0]-10)
      self.finger_tip[0] -= 3
      # print(depth_windows_height[0], depth_windows_height[1], depth_windows_width[1], depth_windows_width[0])
      goal = self.depth_image[depth_windows_height[0]:depth_windows_height[1], depth_windows_width[1]:depth_windows_width[0]]
    # print(goal)

    depth_goal = np.nanmean(np.where(goal >= 0.3, goal, np.nan))

    posemsg = PoseStamped()

    if depth_goal != 0:

      self.finger_tip[2] = depth_goal/1000.

      # print(self.finger_tip)

      # Transform point from pixel coordinates to world coordinates
      self.finger_tip[0], self.finger_tip[1] = self.pixels_to_camera_coord(self.K, self.finger_tip[0], self.finger_tip[1], self.finger_tip[2])

      posemsg.header.frame_id=self.data_d.header.frame_id
      posemsg.header.stamp=rospy.get_rostime()

      posemsg.pose.position.x = round(self.finger_tip[0], 2)
      posemsg.pose.position.y = round(self.finger_tip[1], 2)
      posemsg.pose.position.z = round(self.finger_tip[2], 2)
      posemsg.pose.orientation.x = 0.
      posemsg.pose.orientation.y = 0.
      posemsg.pose.orientation.z = 0.
      posemsg.pose.orientation.w = 1.

    self.goal_msg = posemsg
    #rospy.logwarn(self.set_point)
    status=1
    if self.set_point=="True":
        self._3Dpoint.publish(self.goal_msg)
        rospy.wait_for_service('tree_transf_srv') # wait for service to be available
        try:
            client_handle = rospy.ServiceProxy('tree_transf_srv', tree_transformService) # calling the service that we want to call
            # wait topic
            detections=PoseArray()
            detections.header=posemsg.header
            detections.poses.append(posemsg.pose)
            response1 = client_handle(self.tag,detections) # passing the two received messages to the service and saving the response
            rospy.wait_for_service('load_srv') # wait for service to be available
            try:
                  client_handle = rospy.ServiceProxy('load_srv', database_manager_load) # calling the service that we want to call
                  csv=String()
                # csv.data="pointing_db.csv"
                  csv.data=rospy.get_param("~csv_name")
                  response_load = client_handle(csv) # passing the two received messages to the service and saving the response
                  if response_load.status.data==0:
                      rospy.wait_for_service('update_srv') # wait for service to be available
                      try:
                          client_handle = rospy.ServiceProxy('update_srv', database_manager_update) # calling the service that we want to call
                          response_update = client_handle(response1.detection_res) # passing the two received messages to the service and saving the response
                          if response_update.status.data==0:
                            rospy.wait_for_service('clean_srv') # wait for service to be available
                            try:
                                client_handle = rospy.ServiceProxy('clean_srv', database_manager_clean) # calling the service that we want to call
                                distance_clean=Float64()
                                distance_clean.data=0.0
                                response_clean = client_handle(distance_clean) # passing the two received messages to the service and saving the response
                                if response_clean.status.data==0:
                                    rospy.wait_for_service('upload_srv') # wait for service to be available
                                    try:
                                        client_handle = rospy.ServiceProxy('upload_srv', database_manager_upload) # calling the service that we want to call
                                        response_upload = client_handle(csv) # passing the two received messages to the service and saving the response
                                        if response_upload.status.data==0:
                                            rospy.loginfo("Everything ok")
                                            status=0
                                    except rospy.ServiceException as e:
                                        rospy.logerr("Service call failed: %s"%e)
                            except rospy.ServiceException as e:
                                rospy.logerr("Service call failed: %s"%e)
                      except rospy.ServiceException as e:
                          rospy.logerr("Service call failed: %s"%e)
            except rospy.ServiceException as e:
                  rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
        self.set_point="False"
        if status!=0:
              rospy.logerr("Point not saved")

  def camera_info_callback(self, data):
    self.K = data.K
  def detection_callback(self,data):
    self.tag=data
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
