import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

def movebase_client():

    rospy.init_node('movebase_client_py')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    print("Waiting for server to become available")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.5
    goal.target_pose.pose.orientation.w = 2.0

    print(goal)
    client.send_goal(goal)
    print("the gola was sent to the server")
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def moveBoth():
    rospy.init_node('trajectory_planner')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    pubArm = rospy.Publisher('/mmrobot/joint_position_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.5
    goal.target_pose.pose.orientation.w = 2.0

    client.send_goal(goal)
    wait = client.wait_for_result()

    f = Float64MultiArray()
    f.data = [0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t1 = rospy.Time.now().to_sec() - t0
        if (t1 > 5) :
            f.data[0] = 90
            f.data[5] = 30
        if (t1 > 10) : 
            f.data[1] = 60
            f.data[2] = -50
        pubArm.publish(f)

if __name__ == "__main__":
    try:
        moveBoth()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    try:
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
