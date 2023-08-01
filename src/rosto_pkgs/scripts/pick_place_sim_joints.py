#! /usr/bin/env python3

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import attach as att
import detach as deatt
from sensor_msgs.msg import Joy 
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    

group_arm = moveit_commander.MoveGroupCommander("arm")
group_gripper = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=1)


group_arm_variable_values = group_arm.get_current_joint_values()
group_gripper_variable_values = group_gripper.get_current_joint_values()

#rospy.init_node('manupilator')

def move_arm(joint1,joint2,joint3,joint4):
    group_arm_variable_values[0] = joint1
    group_arm_variable_values[1] = joint2
    group_arm_variable_values[2] = joint3
    group_arm_variable_values[3] = joint4
    group_arm.set_joint_value_target(group_arm_variable_values)
    group_arm.plan()
    group_arm.go(wait=True)
    print("\nManipulator joint values: \n",group_arm.get_current_pose().pose)



def move_gripper(gripper1,gripper2):
    group_gripper_variable_values[0] = gripper1
    group_gripper_variable_values[1] = gripper2
    group_gripper.set_joint_value_target(group_gripper_variable_values)
    group_gripper.plan()
    group_gripper.go(wait=True)
    print("\nGripper value: \n",group_gripper.get_current_joint_values())

def pick(ob,l):
    move_gripper(0.01,0.01)
    current_pose = group_arm.get_current_pose().pose
    move_arm(0,1.39626,-0.575958,-0.8203)  
    move_gripper(-0.002,-0.002)
    att.attach(ob,l)
    current_pose = group_arm.get_current_pose().pose
    move_arm(0,-1.4137,-0.13962,1.58824)
    move_arm(2.82743,-1.4137,-0.13962,1.58824)

def place(ob,l):
    current_pose = group_arm.get_current_pose().pose
    move_arm(0,1.39626,-0.575958,-0.8203)
    move_gripper(0.01,0.01)
    deatt.detach(ob,l)
    current_pose = group_arm.get_current_pose().pose
    move_arm(0,-1.4137,-0.13962,1.58824)
    move_gripper(-0.01,-0.01)

