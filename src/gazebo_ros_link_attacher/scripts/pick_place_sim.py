#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Joy 
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import test as att
import test2 as det
moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('manipulator', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    

group_arm = moveit_commander.MoveGroupCommander("arm")
group_gripper = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=1)

group_arm.set_pose_reference_frame('base_footprint')
group_arm.set_end_effector_link('link5')

group_gripper_variable_values = group_gripper.get_current_joint_values()




def move_arm(x_pos,y_pos,z_pos,x_orien,y_orien,z_orien,w_orien):
    target = Pose()
    global current_pose 
    current_pose = Pose()

    #target.header.frame_id = 'base_footprint'

    target.position.x = x_pos
    target.position.y = y_pos
    target.position.z = z_pos 

    target.orientation.x = x_orien
    target.orientation.y = y_orien
    target.orientation.z = z_orien
    target.orientation.w = w_orien
    
    #group_arm.set_start_state(robot.get_current_state())
    group_arm.set_pose_target(target)
    group_arm.plan()
    group_arm.go(wait=True)
    print("\nManipulator pose \n",group_arm.get_current_pose().pose)


def move_gripper(gripper1,gripper2):
    group_gripper_variable_values[0] = gripper1
    group_gripper_variable_values[1] = gripper2
    group_gripper.set_joint_value_target(group_gripper_variable_values)
    group_gripper.plan()
    group_gripper.go(wait=True)
    print("\nGripper value: \n",group_gripper.get_current_joint_values())


def pick():
    move_gripper(0.01,0.01)
    current_pose = group_arm.get_current_pose().pose
    move_arm(0.12,current_pose.position.y,0.11,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
    att.attach()
    move_gripper(-0.002,-0.002)
    current_pose = group_arm.get_current_pose().pose
    move_arm( -0.01,current_pose.position.y,0.3,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)

def place():
    det.detach()
    current_pose = group_arm.get_current_pose().pose
    move_arm(0.12,0.11,current_pose.position.y,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
    # det.detach()
    move_gripper(0.01,0.01)
    current_pose = group_arm.get_current_pose().pose
    move_arm( -0.01,0.3,current_pose.position.y,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w) 
    move_gripper(-0.01,-0.01)

if __name__ == "__main__":
    rospy.init_node('demo_attach_links')
    # pick()
    place()