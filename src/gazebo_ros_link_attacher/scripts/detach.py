#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "gripper_link"
    req.model_name_2 = "try_0"
    req.link_name_2 = "link_0"

    attach_srv.call(req)
