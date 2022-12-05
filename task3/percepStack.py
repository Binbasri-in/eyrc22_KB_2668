#! /usr/bin/env python3

# all the values should be with respect to the depth camera specifications

# import libraries
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import tf
import math

# create a class for the camera work
class PerceptionNode:
    # constructor
    def __init__(self):
        # initialize the publisher
        self.pub_tf = tf.TransformBroadcaster()
        # initialize the bridge
        self.bridge = CvBridge()
        # initialize the global variables
        self.pose_g = []
        self.yellow_poses = []
        self.red_poses = []
        # focal length of the camera fx, fy
        self.focal_length = [554.3827128226441, 554.3827128226441]
        # center of the image cx, cy
        self.center = [320.5, 240.5]
        # the value to be published continuously
        self.two_found = 0
        self.red_pose = []
        self.yellow_pose = []

    def start_subscribers(self):
        # subscribe to the image topic
        rospy.Subscriber("/camera/color/image_raw2", Image, self.img_clbck)
        # subscribe to the depth topic
        rospy.Subscriber("/camera/depth/image_raw2",Image, self.depth_clbck)

    def img_clbck(self, img_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.loginfo(f"Error: {e}")

        # check if the image is not corrupted
        if image is None:
            return
        
        # call the image processing function
        self.two_found = self.image_processing(image)

    # this function will have the main task where the final values will be calculated and saved in their respected list
    def depth_clbck(self, depth_msg):
        depth_val = []
        ############################### Add your code here #######################################
        # Convert the depth image to cv2 format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.loginfo(f"Error: {e}")

        # check if the image is not corrupted
        if depth_image is None:
            return

        # get the latest centroid of the red object
        if len(self.red_poses) != 0:
            red_pose = self.red_poses[-1]
            # get the depth value of the centroid pixel
            depth_val = depth_image[red_pose[0]*640 // 800, red_pose[1]*480 // 600]
            # save the depth value
            self.red_poses[-1].append(depth_val)

        # get the latest centroid of the yellow object
        if len(self.yellow_poses) != 0:
            yellow_pose = self.yellow_poses[-1]
            # get the depth value of the centroid pixel
            depth_val = depth_image[yellow_pose[0]*640 // 800, yellow_pose[1]*480 // 600]
            # save the depth value
            self.yellow_poses[-1].append(depth_val)

        # save the last red and yellow pose
        if self.two_found == 2:
            self.red_pose = self.red_poses[-1]
            self.yellow_pose = self.yellow_poses[-1]
            self.broadcast_tf()
        ##########################################################################################


    # helping functions for image processing
    def _reduce_noise(img):
        # clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask_red_closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
        mask_red_closed_then_opened = cv2.morphologyEx(
            mask_red_closed, cv2.MORPH_OPEN, kernel)
        return mask_red_closed_then_opened

    # function to get the centroid of the image
    def _get_centroid(img):
        M = cv2.moments(img)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return cx, cy
        else:
            return 0, 0

    # a function to get the biggest contour and return a mask of that contour

    # TODO: add a condition to check the area is not too small

    def _get_biggest_contour(img):
        # get the contours
        contours, hierarchy = cv2.findContours(
            img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # check if there is any contour
        if len(contours) != 0:
            # get the biggest contour
            biggest_contour = max(contours, key=cv2.contourArea)
            # create a mask of the biggest contour
            mask = np.zeros(img.shape, np.uint8)
            cv2.drawContours(mask, [biggest_contour], 0, 255, -1)
        else:
            mask = img
        return mask

    # function to get the coordinates of the object in the camera frame


    # image processing function
    def image_processing(self, image):
        # return value is count
        count = 0
        blur = cv2.GaussianBlur(image, (7, 7), 0)
        # convert to HSV color space
        hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

        # start with red color mask
        min_red = np.array([0, 100, 80])
        max_red = np.array([10, 256, 256])
        # Threshold the HSV image to get only yellow colors
        mask_red = cv2.inRange(hsv, min_red, max_red)
        # reduce noise
        mask_clean = self._reduce_noise(mask_red)
        # get the mask of the biggest contour
        mask_biggest_contour = self._get_biggest_contour(mask_clean)
        # get the centroid of the biggest contour
        cx, cy = self._get_centroid(mask_biggest_contour)
        # if the centroid is not (0,0) then append it to the center list
        if cx != 0 and cy != 0:
            self.red_poses.append([cx, cy])
            count += 1

        # now do the same for yellow color
        min_yellow = np.array([10, 150, 150])
        max_yellow = np.array([19, 256, 256])
        # Threshold the HSV image to get only yellow colors
        mask_yellow = cv2.inRange(hsv, min_yellow, max_yellow)
        # reduce noise
        mask_clean = self._reduce_noise(mask_yellow)
        # get the mask of the biggest contour
        mask_biggest_contour = self._get_biggest_contour(mask_clean)
        # get the centroid of the biggest contour
        cx, cy = self._get_centroid(mask_biggest_contour)
        # if the centroid is not (0,0) then append it to the center list
        if cx != 0 and cy != 0:
            self.yellow_poses.append([cx, cy])
            count += 1

        return count

    # broadcast the transform of the camera frame to the world frame (base_link)
    def broadcast_tf(self):
        # get coordinates of red object
        if self.two_found == 2:
            # get the last element in the list
            red_pose = self.red_pose
            # convert the pixel coordinates to the camera frame
            red_pose = self.pixel2cam(red_pose)
            # broadcast the transform
            self.pub_tf.sendTransform(red_pose, (0, 0, 0, 1), rospy.Time.now(),
                                      "fruit_red", "camera_link")

        # get coordinates of yellow object
        if self.two_found == 2:
            # get the last element in the list
            yellow_pose = self.yellow_pose
            # convert the pixel coordinates to the camera frame
            yellow_pose = self.pixel2cam(yellow_pose)
            # broadcast the transform
            self.pub_tf.sendTransform(yellow_pose, (0, 0, 0, 1), rospy.Time.now(),
                                      "fruit_yellow", "camera_link")

    # function to convert pixel coordinates to camera frame
    def pixel2cam(self, pixel):
        # get the depth value of the centroid pixel
        depth_val = pixel[2]
        # get the focal length of the camera
        f = self.focal_length
        # get the center of the image
        center = self.center
        # get the pixel coordinates
        x, y = pixel[0], pixel[1]
        # calculate the coordinates of the object in the camera frame
        x = (x - center[0]) * depth_val / f
        y = (y - center[1]) * depth_val / f
        z = depth_val
        return x, y, z
        
'''
[ERROR] [1670259484.121675, 7.814000]: bad callback: <bound method PerceptionNode.img_clbck of <percepStack.PerceptionNode object at 0x7f401036bc10>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/hashirama_b/catkin_ws/src/eyrc-2022_krishibot/scripts/percepStack.py", line 52, in img_clbck
    self.two_found = self.image_processing(image)
  File "/home/hashirama_b/catkin_ws/src/eyrc-2022_krishibot/scripts/percepStack.py", line 147, in image_processing
    mask_clean = self._reduce_noise(mask_red)
TypeError: _reduce_noise() takes 1 positional argument but 2 were given

'''
