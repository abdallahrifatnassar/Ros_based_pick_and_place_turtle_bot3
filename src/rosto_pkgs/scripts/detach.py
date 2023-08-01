#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy


attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
attach_srv.wait_for_service()
rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")
# Link them
rospy.loginfo("Attaching cube1 and cube2")
def detach(ob2,l2):
    req = AttachRequest()
    # req.model_name_1 = "cube1"
    # req.link_name_1 = "link"
    # req.model_name_2 = "cube2"
    # req.link_name_2 = "link"

    req.model_name_1 = "robot"
    req.link_name_1 = "gripper_link"
    req.model_name_2 = ob2
    req.link_name_2 = l2
    attach_srv.call(req)


