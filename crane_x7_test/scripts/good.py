#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

see_x = 0.10
see_y = -0.10
see_z = 0.3

def main():
    #rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.7)
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

    arm.set_named_target("vertical")
    arm.go()

    #ハンドを開く/閉じる
def move_gripper(pou):
    gripper.set_joint_value_target([pou, pou])
    gripper.go()

    #アームを移動する
def move_arm(pos_x, pos_y, pos_z):
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = pos_x
    target_pose.position.y = pos_y
    target_pose.position.z = pos_z
    q = quaternion_from_euler(-3.14, 0.0, -3.14)  # 上方から掴みに行く場合, 引数(-3.14, 0.0, -3.14/2.0) -> (-3.14, 0.0, -3.14)に変更
   # q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    #探索ポジション？
    print("tansaku")
    move_arm(see_x, see_y, see_z)
    print("1")
    rospy.sleep(1)
    x = -0.30
    y = -0.30
    print(123)
    rospy.sleep(2)
    move_arm(0.30, 0.2, 0.3)
    rospy.sleep(1)
    print("start")
    move_arm(0.2, -0.3, 0.3)
    print("max")
    rospy.sleep(1.0)

def pickcallback(pose):
    seal_x = pose.x
    seal_y = pose.y
    mode = pose.theta





    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.30
    target_pose.position.y = -0.15
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14)  
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
    """

    print("end")

if __name__ == "__main__":
    try:
        if not rospy.is_shutdown():
            main()
            rospy.init_node("good")
            sub = rospy.Subscriber("pose", Pose2D, pickcallback)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
