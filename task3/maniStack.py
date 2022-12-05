#! /usr/bin/env python3
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

#------------------------------Constructor--------------------------------------------------------
    def __init__(self):

        rospy.init_node('task3', anonymous=True)


        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=5)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self.arm.get_planning_frame()
        self._eef_link = self.arm.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

#---------------------------------------------------------------------------------------------------------
#Function to go to pose obtained by camera.
    def go_to_pose(self, arg_pose):

        pose_values = self.arm.get_current_pose().pose
        
        self.arm.set_pose_target(arg_pose)
        flag_plan = self.arm.go(wait=True)  # wait=False for Async Move

        pose_values = self.arm.get_current_pose().pose
        
        list_joint_values = self.arm.get_current_joint_values()
        
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
#--------------------------------------------------------------------------------------------------
#Function to go to a predefined pose with arm planning group

#---------------------------------- Destructor-----------------------------------------------------
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
#---------------------------------END OF UR5 CLASS-------------------------------------------------

def main():

    ur5 = Ur5Moveit()

    ur5_pose_1 = geometry_msgs.msg.Pose()
    
    while not rospy.is_shutdown():
        #sequence is defined here.
        print('This block needs the sequence')
        break
    del ur5

#----------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    main()
