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

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			percepStack.py
# Functions:
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
# You can add more if required
##############################################################


# Initialize Global variables
pub_rgb = rospy.Publisher('/center_rgb', String, queue_size=1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size=1)
pose_g = list()


################# ADD UTILITY FUNCTIONS HERE #################

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
    global pub_rgb  # , add global variable if any

    ############################### Add your code here #######################################
    # Convert the image to cv2 format
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    # show the image
    cv2.imshow("Image 2window", image)
    cv2.waitKey(0)
    ##########################################################################################
    pose = image_processing(image)
    global pose_g
    pose_g = pose
    pub_rgb.publish(str(pose))


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
    depth_val = []
    ############################### Add your code here #######################################
    # print the message to check the format of the message
    global pose_g
    rospy.loginfo(f"Depth: {pose_g}")
    ##########################################################################################
    pub_depth.publish(str(depth_val))


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
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################
    # Convert the image to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # Define the lower and upper range of the color
    lower = np.array([0, 100, 100])
    upper = np.array([10, 255, 255])
    # Create a mask
    mask = cv2.inRange(hsv, lower, upper)
    # Find the contours
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Find the centroid of the contours
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            pose.append([cx, cy])
    ##########################################################################################
    return pose


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
    cv2.destroyAllWindows()
    rospy.init_node("percepStack", anonymous=True)
    # subscribe to image topic 1
    sub_image_color_1 = rospy.Subscriber(
        "/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    sub_image_depth_1 = rospy.Subscriber(
        "/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)

    # subscribe to image topic 2
    sub_image_color_2 = rospy.Subscriber(
        "/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    sub_image_depth_2 = rospy.Subscriber(
        "/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)

    # subscribe to image topic 2
    sub_image_color_3 = rospy.Subscriber(
        "/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    sub_image_depth_3 = rospy.Subscriber(
        "/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)

    ####################################################################################################
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")
