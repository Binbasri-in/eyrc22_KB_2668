#! /usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

count = 0

def img_clbck(img_msg):
    global count
    # Convert the image to cv2 format
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        global count
        cv2.imwrite(f"{count}.png", image)
    except CvBridgeError as e:
        rospy.loginfo(f"Error: {e}")

    # save the images to a folder
    count += 1
    cv2.imwrite(f"~/catkin_ws/src/eYRC-2022_KrishiBot/eyrc-2022_krishibot/scripts/{count}.png", image)

    # check if the image is not corrupted
    if image is None:
        return
    #rospy.loginfo(f"image shape: {image.shape}")


if __name__ == '__main__':
    rospy.init_node('get_img', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw2', Image, img_clbck)
    rospy.spin()