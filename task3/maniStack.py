#! /usr/bin/env python3
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import percepStack

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
#Function to go to a predefined pose 


    def go_to_predefined_pose(self, group, arg_pose_name):

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self.group.set_named_target(arg_pose_name)
        plan = self.group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
           goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
#---------------------------------- Destructor-----------------------------------------------------
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
#---------------------------------END OF UR5 CLASS-------------------------------------------------

def main():

    ur5 = Ur5Moveit()
    cam=percepStack.PerceptionNode()

    
    pose_red = geometry_msgs.msg.Pose()
    pose_yellow= geometry_msgs.msg.Pose()
    
    flag = 0
    while not rospy.is_shutdown():
        #sequence of actions
        ur5.go_to_predefined_pose(ur5.arm,"face_tree")
        rospy.sleep(1)
        if flag == 0:
            cam.start_subscribers()
            rospy.sleep(1)
            listner = tf.TransformListener()
            flag = 1
            
        try:
            (trans_red,rot_red) = listner.lookupTransform('/ebot_base', '/fruit_red', rospy.Time(0))
            #(trans_yellow,rot_yellow)= listner.lookupTransform('/ebot_base', '/fruit_yellow', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        if len(trans_red) != 0:
            # set the new pose
            pose_red.position.x = trans_red[0]
            pose_red.position.y = trans_red[1]
            pose_red.position.z = trans_red[2]
            pose_red.orientation.x = rot_red[0]
            pose_red.orientation.y = rot_red[1]
            pose_red.orientation.z = rot_red[2]
            pose_red.orientation.w = rot_red[3]
            # make the robot go to the pose rest position
            ur5.go_to_predefined_pose(ur5.arm,"allZeros")
            rospy.sleep(1)
            ur5.go_to_pose(pose_red)
            rospy.sleep(1)
            ur5.go_to_predefined_pose(ur5.arm,"allZeros")
            rospy.sleep(1)
    del ur5

#----------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    main()
