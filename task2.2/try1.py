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
pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
pose_g = []
counter = 0

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
    global pub_rgb, counter #, add global variable if any

    ############################### Add your code here #######################################
    # Convert the image to cv2 format
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(f"Error: {e}")

    # check if the image is not corrupted
    if image is None:
        counter += 1
        return
    rospy.loginfo(f"image shape: {image.shape}")
    
    ##########################################################################################
    pose = image_processing(image)
    
    rospy.loginfo(f"center: {pose}")
    global pose_g
    pose_g = pose
    pub_rgb.publish(str(pose))
    counter += 1

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
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "16UC1")

    # check if the image is not corrupted
    if depth_img is None:
        return
    
    # get each centroid
    for c in pose_g:
        x = c[0] * 480 // 720
        y = c[1] * 848 // 1280
        depth_val.append(depth_img[x][y]/1000)
    rospy.loginfo(f"depth shape: {depth_img.shape}")
    rospy.loginfo(f"depth: {depth_val}")
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
    # convert the image to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Remove noise from the image
    # define the kernel size
    kernel_size = 5
    image = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    # convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # Filter the image using blue color
    # define two blue boundaries, one for red and one for golden yellow
    # for red
    blueLower1 = np.array([166,100,100])
    blueUpper1 = np.array([186,255,255])
    # for golden yellow
    blueLower2 = np.array([5,100,100])
    blueUpper2 = np.array([25,255,255])
    # Threshold the HSV image to get only blue colors
    mask1 = cv2.inRange(hsv, blueLower1, blueUpper1)
    mask2 = cv2.inRange(hsv, blueLower2, blueUpper2)
    # combine the two masks
    mask = mask1 | mask2

    # save the dimentions of the mask in variable
    height, width = mask.shape
    # create an image with the same dimentions as the mask
    mask_image = np.zeros((height, width, 3), np.uint8)
    # fill the image with the color white
    mask_image[:] = (255, 255, 255)
    # Bitwise-AND mask and masked image
    res = cv2.bitwise_and(mask_image,mask_image, mask= mask)

    # use Morphological Transformations to remove noise
    # define the kernel size
    kernel = np.ones((5,5),np.uint8)
    # apply the erosion
    erosion = cv2.erode(res,kernel,iterations = 12)
    # apply dilation
    dilation = cv2.dilate(erosion,kernel,iterations = 12)

    # create a mask from the image
    mask = cv2.inRange(dilation, (254,254,254), (255,255,255))
    
    #cv2.imwrite(f"/home/binbasri0/catkin_ws/src/eyrc-2022_krishibot/scripts/original{counter}.png", mask)
    # find the contours in the mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # get the center of each contour
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        pose.append([cX, cY])
        #cv2.circle(image, (cX, cY), 5, (255, 255, 255), -1)
        # write a text for centroid
        #cv2.putText(image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    #cv2.drawContours(image,contours,-1,(0,255,0),3)
    #cv2.imwrite(f"/home/binbasri0/catkin_ws/src/eyrc-2022_krishibot/scripts/original{counter}.png", image)
    
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

    rospy.init_node("percepStack", anonymous=True)
    # define a rate
    rate = rospy.Rate(4)
    
    # start looping
    while not rospy.is_shutdown():
        # data 1
        rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
        rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
        rospy.sleep(0.5)

        # data 2
        rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
        rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
        rospy.sleep(0.5)

        # data 3
        rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
        rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
        rospy.sleep(0.5)





    ####################################################################################################

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")