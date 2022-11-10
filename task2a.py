#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

rospy.init_node('ur5_controller', anonymous=True)
commander = moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

def main():
    exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    exectute_trajectory_client.wait_for_server()

    arm.set_named_target("face_red")
    plan = arm.plan()
    goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    try:
        goal.trajectory = plan[1]
    except:
        goal.trajectory = plan
    exectute_trajectory_client.send_goal(goal)
    exectute_trajectory_client.wait_for_result()
    
if __name__ == '__main__':
    main()