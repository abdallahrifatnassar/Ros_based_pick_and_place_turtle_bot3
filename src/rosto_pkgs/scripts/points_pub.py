#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import pick_place_sim_joints as pp

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def movebase_client(x_dist,y_dist,yaw):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_dist
    goal.target_pose.pose.position.y = y_dist
    x_angle , y_angle , z_angle , w_angle  = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = x_angle 
    goal.target_pose.pose.orientation.y = y_angle 
    goal.target_pose.pose.orientation.z = z_angle 
    goal.target_pose.pose.orientation.w = w_angle 

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


    if result.outcome == MoveBaseResult.SUCCESS:
        rospy.loginfo("Follow path action succeeded")
    else:
        rospy.logerr("Follow path action failed with error code [%d]: %s", result.outcome, result.message)



if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    to_A_1=[[0.308390,-0.101102,-1.001648],[0.575126,-0.468315,-1.001623],[1.40,-0.422577,1.568804],[1.40,0.291471,1.568804],[1.5,1.483165,1.568804],[1.500000,2.210000,1.57]]
# 111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
    to_A_2=[[1.40,2,-1.568804],[1.409991,1,-1.565896], [1.384052,-0.337588,-1.568804],[1.202157,-0.498335,-2.227602],[0.979087,-0.513880,-2.991903],[0.341309,-0.610155,-3.134385],[-0.615796,-0.610155,-3.134385],[-0.804595,-0.485550,2.799913],[-1.069139,0.568481,3.126370],[-1.582679,0.751244,1.567183],[-1.582679,1.755895,1.567183],[-1.463658,2.654564,-0.021656],[-0.954773,2.6,-0.021656]]
# 222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
    to_B_1=[[-0.954745,2.599970,1.576634],[-1.322061,2.65,-3.117752],[-1.6,2.268702,-1.596162],[-1.587451,1.383785,-1.596168],[-1.486317,0.6,-0.643905],[-1,0.5,0.0],[-0.5,0.5,0],[-0.19,0.5,0.0]]
# 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
    to_B_2=[[-0.169514,0.45,-3.127427],[-0.988158,0.45,-3.127427],[-1.556641,0.65,1.565961],[-1.587451,1.383785,1.565961],[-1.460991,2.551966,1.596461]]
# 444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
    to_origin=[[-1.465256,2.550063,-1.501917],[-1.5,1.8,-1.595973],[-1.587451,1.668548,-1.596168],[-1.486317,0.55,-0.643905],[-0.945276,0.275303,-1.202575],[-0.543489,-0.640379,-0.018826],[0,-0.640379,-0.018826],[0.65,-0.640379,1.854906],[0.575126,-0.468315,1.858501],[0.308390,-0.101102,2.449045],[0,0,-3.054583],[0,-0.1,0]]
    list_points=[to_A_1,to_A_2,to_B_1,to_B_2,to_origin]
    t = 0
    current_target=1
    
    for i in list_points:
        rospy.loginfo("Moving to target [%d]", current_target)
        current_via_point=1
        for j in i:
            rospy.loginfo("Moving to via_point [%d]", current_via_point)


            x=j[0]
            y=j[1]
            yaw=j[2]
            

            rospy.loginfo("Goal execution Started!")
            result = movebase_client(x,y,yaw)
            if result:
                rospy.loginfo("Goal execution done!")

            else:
                rospy.loginfo("Goal execution Failed!")
            current_via_point=current_via_point+1
        current_target=current_target+1
        if i== to_A_1:
            pp.pick("obj1","link_1")
        elif i== to_A_2:
            pp.place("obj1","link_1")
        elif i== to_B_1:
            pp.pick("obj2","link_2")
        elif i== to_B_2:
            pp.place("obj2","link_2")