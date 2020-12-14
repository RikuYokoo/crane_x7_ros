#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose2D
import rosnode
import math
from tf.transformations import quaternion_from_euler

correct_pose_z = 0.3

see_x = 0
see_y = 0
see_z = 0.3

def cb(pose):
    x = pose.x
    y = pose.y
    theta = pose.theta
    print(x,y,theta)

if __name__ == "__main__":
    rospy.init_node('tameshi')
    sub = rospy.Subscriber("pose", Pose2D, cb)
    rospy.spin()
    

