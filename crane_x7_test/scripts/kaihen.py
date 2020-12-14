#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose2D
import rosnode
import math
from tf.transformations import quaternion_from_euler

rospy.init_node("kaihen")
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.1)
gripper = moveit_commander.MoveGroupCommander("gripper")

while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    rospy.sleep(1.0)
rospy.sleep(1.0)

mode = True

see_x = 0.10
see_y = -0.10
see_z = 0.3
pick_z = 0.13

def arm_move(x, y, z):
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(- math.pi, 0.0,- math.pi)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()
def main():
    #self.pub =()
    #self.sub = rospy.Subscriber("pose", Pose2D, self.pickcallback)
    rospy.init_node("kaihen")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)


    arm.set_named_target("home")
    arm.go()

    arm_move(see_x, see_y, see_z)
    #ハンコを探すための位置
    #arm_move(see_x, see_y, see_z)
    """
    def arm_move(x, y, z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        q = quaternion_from_euler(- math.pi,0.0,- math.pi)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
    """
        

    #arm_move(see_x, see_y, see_z)
#def arm_move(x,y,z):
    """
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    """

    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(- math.pi,0.0,- math.pi)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()
    """

def pick_seal():
    global see_x, see_y, pick_z
    arm_move(see_x, see_y, pick_z)


def pickcallback(pose):
    print("pickcallback")
    limit = pose.theta
    pose_x = pose.x
    pose_y = pose.y
    global mode

    global see_x , see_y

    """
    if(pose_x < -20 and pose_y < -20):
        see_x -= 0.01
        see_y += 0.01
    elif(pose_x < -20 and pose_y >= 20):
        see_x += 0.01
        see_y += 0.01
    elif(pose_x >= 20 and pose_y < -20):
        see_x -= 0.01
        see_y -= 0.01
    elif(pose_x >= 20 and pose_y >= 20):
        see_x += 0.01
        see_y -= 0.01
    """
    if(mode == True):
        print("ifbunndesu")
        print("zahyou", pose_x, pose_y)
        if(limit == 0):
            print("探す")
            arm_move(see_x, see_y, see_z)
            see_x += 0.01
        elif(pose_x < 60):
            print(1)
            see_x -= 0.01
        elif(pose_x > 70):
            print(2)
            see_x += 0.01
        elif(pose_y < -90):
            print(3)
            see_y -= 0.01
        elif(pose_y > -80):
            print(4)
            see_y += 0.01
        else:
            print(5)
            flag = 1
            mode = False
            pick_seal()

        print(see_x, see_y, see_z)

        arm_move(see_x, see_y, see_z)
        rospy.sleep(1.0)
        print("pickcallback,end")

    else:
        pass




    """
    def see_seal():
        #See_and_pick_seal.arm_move(correct_pose_x, correct_pose_y, correct_pose_z)
        sub = rospy.Subscriber("pose", Pose2D, self.pickcallback)
    """




if __name__ == "__main__":
    main()

    try:
        if not rospy.is_shutdown():
            #rospy.init_node("pick_hanko")
            sub = rospy.Subscriber("pose", Pose2D, pickcallback, queue_size=1)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

