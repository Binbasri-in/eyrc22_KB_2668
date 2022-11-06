#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of Krishi Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
# Team ID:			[ eYRC#KB#2668 ]
# Author List:		[ Sai Harshit B, Pranesh Kannan, S Guru Prasad, Mohammed Ali Alsakkaf ]
# Filename:			KB_2668.py
# Functions:
# 					[ revolving, move_stright, callback, main ]
# Nodes:		    Add your publishing and subscribing node: D_shape_draw



####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
##############################################################

################# ADD GLOBAL VARIABLES HERE #################
cur_pose = Pose()
PI = 3.1415926535897
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

def revolving(pubs: rospy.Publisher, l_speed: float,r_speed: float, angle):
    """
	Purpose:
	---
	This function can be used to draw circle or rotate the turtle around itself

	Input Arguments:
	---
        'pubs': publisher
                to send the velocity to the topic cmd_vel
        'linear_speed': float
                the linear speed of the turtle
        'angular_speed': float
                the angular speed of the turtle
        'angle': float
            the target or goal theta of the turtle to get, in degrees

	Returns:
	---
        None
	Example call:
	---
        None
	"""
    # initiate a message to be sent to turtle to draw a circle of 1 unit
    vel_msg = Twist()
    # initiate linear velocity only x
    vel_msg.linear.x = l_speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    # initiate the angular velocity only z
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = r_speed

    # set current angle and the target angle
    target_angle = angle * PI / 180
    current_angle = 0
    buffer = 0.25

    # get the starting time to get angle from angular velocity
    t_start = rospy.Time().now().to_sec()

    # set a rate to see the result slowly
    rate = rospy.Rate(1000)

    # publish the velocity while the angle is less than what we want
    # for half circle the turtle must rotate 180 degree
    while current_angle < target_angle + buffer:
        # publish velocit to cmd_vel topic
        pubs.publish(vel_msg)
        # get current time after publishing
        t_current = rospy.Time().now().to_sec()
        # calculate the angle moved so far
        current_angle = vel_msg.angular.z * (t_current - t_start)
        # slow rate
        rate.sleep()
        # special case for self rotary no need for buffer
        if current_angle >= target_angle+0.001 and vel_msg.linear.x == 0:
            break

    # stop linear velocity
    vel_msg.linear.x = 0
    # stop angular velocity
    vel_msg.angular.z = 0
    # stop the operation of the turtle
    pubs.publish(vel_msg)

##############################################################

def move_stright(pubs: rospy.Publisher, speed: float, units):
    """
	Purpose:
	---
	This function moves the turtle in straight lines for n units

	Input Arguments:
	---
        'pubs': publisher
                to send the velocity to the topic cmd_vel
        'speed': float
                the linear speed of the turtle
        'units': float
            the target or goal distence of the turtle to get

	Returns:
	---
        None

	Example call:
	---
        None
	"""
    # initiate a message to be sent to turtle to draw a circle of 1 unit
    vel_msg = Twist()
    # initiate linear velocity only x
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    # initiate the angular velocity only z
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # get the starting time to get angle from angular velocity
    t_start = rospy.Time().now().to_sec()

    # set a rate to see the result slowly
    rate = rospy.Rate(1000)

    # get the time would take to reach the goal
    time = units/speed

    # stop after moving n units
    # since linear velocity is speed per sec
    # stop after n/speed seconds
    while not rospy.is_shutdown():
        # publish velocit to cmd_vel topic
        pubs.publish(vel_msg)
        # get current time after publishing
        t_current = rospy.Time().now().to_sec()
        # slow rate
        rate.sleep()
        # when to stop
        if t_current - t_start >= time:
            break

    # stop linear velocity
    vel_msg.linear.x = 0
    # stop the operation of the turtle
    pubs.publish(vel_msg)


##############################################################

def callback(msg: Pose):
    """
	Purpose:
	---
	This function should be used as a callback. Refer Example #1:
    Pub-Sub with Custom Message in the Learning Resources Section of the Learning Resources.
    You can write your logic here.
    NOTe: Radius value should be 1. Refer expected output in document
    and make sure that the tu rtle traces "same" path.

	Input Arguments:
	---
        `data`  : []
            data received by the call back function

	Returns:
	---
        None

	Example call:
	---
        None
	"""
    current_angle = msg.theta
    if current_angle < -PI/2 + 0.05 and current_angle > -PI/2 - 0.05:
        rospy.loginfo("Moving in stright line \n Turtle X = %f: Y = %f: theta = %f:\n",\
             msg.x, msg.y , msg.theta)
    elif current_angle < -0.5*PI and current_angle > -PI:
        rospy.loginfo("Turning Downward \n Turtle X = %f: Y = %f: theta = %f:\n",\
             msg.x, msg.y , msg.theta)
    else:
        rospy.loginfo("Moving in circle \n Turtle X = %f: Y = %f: theta = %f:\n",\
             msg.x, msg.y , msg.theta)




def main():
    """
	Purpose:
	---
	This function will be called by the default main function given below.
    You can write your logic here.

	Input Arguments:
	---
        None

	Returns:
	---
        None
	Example call:
	---
        main()
	"""
    # Initialize the node that will control and recive the turtlesim node
    rospy.init_node("D_shape_draw", anonymous=True)

    # create a publisher to the command_velocity topic to control the velocity
    vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    # create a subscriber to the pose topic to get info about the turtle
    rospy.Subscriber("/turtle1/pose", Pose, callback)

    # draw the semi circle by seting linear 1, rotating 1, and degree 180
    revolving(vel_pub, 0.5, 0.5, 180)

    # rotate the turtle to face downward
    revolving(vel_pub, 0.0, 0.5, 90)

    # draw stright line of n unit
    move_stright(vel_pub, 0.5, 2)






######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########    
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()
   
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
