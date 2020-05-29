#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import moveit_msgs.msg
import actionlib


def sendGoalToArm():
    rospy.init_node("SendSingleGoal")
    action = actionlib.SimpleActionClient('move_group', moveit_msgs.msg.MoveGroupAction)
    action.wait_for_server()
    rate = rospy.Rate(10)
    # example position
    p = [1.5, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0]
    tol = 0.001
    goal = moveit_msgs.msg.MoveGroupGoal()
    c = moveit_msgs.msg.Constraints()
    for i in range(7):
        c.joint_constraints.append(moveit_msgs.msg.JointConstraint())
        c.joint_constraints[i].joint_name = "mmrobot_joint" + str(i+1)
        c.joint_constraints[i].position = p[i]
        c.joint_constraints[i].tolerance_above = tol
        c.joint_constraints[i].tolerance_below = tol
        c.joint_constraints[i].weight = 1.0
    goal.request.goal_constraints.append(c)
    goal.request.planner_id = "RRTConnect"
    goal.request.group_name = "arm_group"
    goal.request.num_planning_attempts = 10.0
    goal.request.allowed_planning_time = 5.0
    action.send_goal(goal)


if __name__ == "__main__":
    try:
        sendGoalToArm()
    except rospy.ROSInterruptException:
        pass
