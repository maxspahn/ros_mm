#!/usr/bin/env python

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    rospy.init_node('movebase_client_py')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    print("Waiting for server to become available")
    client.wait_for_server()

    goal = MoveBaseGoal()
    x = [1.5, 2.0, 2.5]
    y = [2.0, 2.0, 2.0]
    goal.target_pose.header.frame_id = "base_link"
    for i in range(3):
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x[i]
        goal.target_pose.pose.orientation.w = 1.0

        print(goal)
        client.send_goal(goal)
        print("the goal was sent to the server")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print(client.get_result())

if __name__ == '__main__':
    try:
        result = movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
