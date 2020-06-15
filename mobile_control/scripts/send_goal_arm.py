#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def sendGoalToArm():
    print "============ Starting setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('SendGoal')

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander(robot_description="mmrobot/robot_description",ns="mmrobot")

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.
    group_name = "arm_group"
    move_group = moveit_commander.MoveGroupCommander(group_name,robot_description="mmrobot/robot_description")

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi / 8
    joint_goal[2] = 0
    joint_goal[3] = -pi / 8
    joint_goal[4] = 0
    joint_goal[5] = pi / 8
    joint_goal[6] = 0

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0.0
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0
    pose_goal.position.z = 1.2

    # Change either to joint_goal or pose_goal
    current_goal = pose_goal;

    # Execute
    move_group.set_pose_target(current_goal)
    move_group.go(current_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    move_group.clear_pose_targets()

if __name__ == "__main__":
    try:
        sendGoalToArm()
    except rospy.ROSInterruptException:
        pass