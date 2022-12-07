#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ eYRC#KB#2668 ]
# Author List:		[ Sai Harshit B, Pranesh Kannan, S Guru Prasad, Mohammed Ali Alsakkaf ]
# Filename:		KB_2668.py
# Functions:
# 			[ image_processing, img_clbck, depth_clbck, main ]


####################### IMPORT MODULES #######################
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import tf
# You can add more if required
##############################################################


# Initialize Global variables
pub_rgb = rospy.Publisher('/center_rgb', String, queue_size=1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size=1)
pose_g_red = []
pose_g_yellow = []
is_two_found = 0
red_pose = []
yellow_pose = []
red_object = ()
yellow_object = ()
orent_g = ()
br = tf.TransformBroadcaster()


################# ADD UTILITY FUNCTIONS HERE #################
# a function to reduce the noise in the image


def reduce_noise(img):
    # clean up
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask_red_closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    mask_red_closed_then_opened = cv2.morphologyEx(
        mask_red_closed, cv2.MORPH_OPEN, kernel)
    return mask_red_closed_then_opened

# function to get the centroid of the image


def get_centroid(img):
    M = cv2.moments(img)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy
    else:
        return 0, 0

# a function to get the biggest contour and return a mask of that contour


def get_biggest_contour(img):
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
##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic
    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.
    Input Args:
    -----
    img_msg: Callback message.
    '''
    global red_pose, yellow_pose  # , add global variable if any

    ############################### Add your code here #######################################
    ################### broadcast the transform
    if is_two_found == 3:
        br.sendTransform(red_object,
                         orent_g,
                         rospy.Time.now(),
                         "fruit_red",
                         "ebot_base")
        br.sendTransform(yellow_object,
                         orent_g,
                         rospy.Time.now(),
                         "fruit_yellow",
                         "ebot_base")
        rospy.loginfo("transform published")
        return
    ##########################################################################################
    # Convert the image to cv2 format
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.loginfo(f"Error: {e}")

    # check if the image is not corrupted
    if image is None:
        return
    rospy.loginfo(f"image shape: {image.shape}")
    # clear global variables if not already found
    if is_two_found != 2:
        red_pose = []
        yellow_pose = []
        image_processing(image)
    ##########################################################################################

    


def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic
    Purpose:
        --- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'
    NOTE: the shape of depth and rgb image is different. 
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    ############################### Add your code here #######################################
    ################### broadcast the transform
    if is_two_found == 3:
        br.sendTransform(red_object,
                         orent_g,
                         rospy.Time.now(),
                         "fruit_red",
                         "ebot_base")
        br.sendTransform(yellow_object,
                         orent_g,
                         rospy.Time.now(),
                         "fruit_yellow",
                         "ebot_base")
        rospy.loginfo("transform published")
        return
    ##########################################################################################

    #####################
    global is_two_found
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "16UC1")

    # check if the image is not corrupted
    if depth_img is None:
        return

    # print the image shape
    rospy.loginfo(f"depth image shape: {depth_img.shape}")

    if is_two_found == 2:
        calculate_coordinates(depth_img)

    

    
    ##########################################################################################
    


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.
    Input Args:
    ------
    image: Converted image in cv2 format.
    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''

    ############### Write Your code to find centroid of the bell peppers #####################
    global red_pose, yellow_pose, is_two_found, pose_g_red, pose_g_yellow
    # blur the image
    blur = cv2.GaussianBlur(image, (7, 7), 0)
    # convert to HSV color space
    hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

    # start with red color mask
    min_red = np.array([0, 100, 80])
    max_red = np.array([10, 256, 256])
    # Threshold the HSV image to get only yellow colors
    mask_red = cv2.inRange(hsv, min_red, max_red)
    # reduce noise
    mask_clean = reduce_noise(mask_red)
    # get the mask of the biggest contour
    mask_biggest_contour = get_biggest_contour(mask_clean)
    # get the centroid of the biggest contour
    cx, cy = get_centroid(mask_biggest_contour)
    # if the centroid is not (0,0) then append it to the center list
    if cx != 0 and cy != 0:
        red_pose.append(cx)
        red_pose.append(cy)
        is_two_found += 1
        rospy.loginfo(f"red_pose: {red_pose}")# print the red_pose

    # now do the same for yellow color
    min_yellow = np.array([10, 150, 150])
    max_yellow = np.array([19, 256, 256])
    # Threshold the HSV image to get only yellow colors
    mask_yellow = cv2.inRange(hsv, min_yellow, max_yellow)
    # reduce noise
    mask_clean = reduce_noise(mask_yellow)
    # get the mask of the biggest contour
    mask_biggest_contour = get_biggest_contour(mask_clean)
    # get the centroid of the biggest contour
    cx, cy = get_centroid(mask_biggest_contour)
    # if the centroid is not (0,0) then append it to the center list
    if cx != 0 and cy != 0:
        yellow_pose.append(cx)
        yellow_pose.append(cy)
        is_two_found += 1
        rospy.loginfo(f"yellow_pose: {yellow_pose}")# print the yellow_pose

        if is_two_found == 2:
            pose_g_red = red_pose
            pose_g_yellow = yellow_pose
        #cv2.circle(image, (cX, cY), 5, (255, 255, 255), -1)
        # write a text for centroid
        #cv2.putText(image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    # cv2.drawContours(image,contours,-1,(0,255,0),3)
    #cv2.imwrite(f"/home/binbasri0/catkin_ws/src/eyrc-2022_krishibot/scripts/original{counter}.png", image)

    ##########################################################################################

# function to calculate the x y z with respect to the ebot base frame
def calculate_coordinates(depth_img):
    # the global values to be used
    global pose_g_red, pose_g_yellow, is_two_found, red_object, yellow_object, orent_g
    # the focal length of the camera
    f = 554.3827128226441
    # the center from camera
    center = [320.5, 240.5]

    # calculate the depth of the red pepper
    red_depth = depth_img[pose_g_red[0], pose_g_red[1]]
    pose_g_red.append(red_depth)
    # calculate the depth of the yellow pepper
    yellow_depth = depth_img[pose_g_yellow[0], pose_g_yellow[1]]
    pose_g_yellow.append(yellow_depth)

    # calculate the x y z of the red pepper
    red_x = (pose_g_red[0] - center[0]) * red_depth / f
    red_y = (pose_g_red[1] - center[1]) * red_depth / f
    
    # calculate the x y z of the yellow pepper
    yellow_x = (pose_g_yellow[0] - center[0]) * yellow_depth / f
    yellow_y = (pose_g_yellow[1] - center[1]) * yellow_depth / f

    # print the values
    rospy.loginfo(f"Red Pepper: x: {red_x}, y: {red_y}, z: {red_depth}")
    rospy.loginfo(f"Yellow Pepper: x: {yellow_x}, y: {yellow_y}, z: {yellow_depth}")

    # get the coordinates of camera frame
    li = tf.TransformListener()
    while True:
        try:
            (trans, orents) = li.lookupTransform('ebot_base', 'camera_link2', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    # calculate the final coordinates with respect to the ebot base frame
    red_object = (trans[0] + red_x, trans[1] + red_y,trans[2] + red_depth)
    yellow_object = (trans[0] + yellow_x, trans[1] + yellow_y, trans[2] + yellow_depth)
    orent_g = orents

    is_two_found = 3






def main():
    '''
    MAIN FUNCTION
    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.
    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.
    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####

    rospy.init_node("percepStack", anonymous=True)
    # define a rate
    rate = rospy.Rate(10)

    # start looping
    while not rospy.is_shutdown():
        # subscribe to the image topic
        rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
        # subscribe to the depth topic
        rospy.Subscriber("/camera/depth/image_raw2",Image, depth_clbck)
        rate.sleep()

    ####################################################################################################


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")
