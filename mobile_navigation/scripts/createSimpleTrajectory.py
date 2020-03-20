import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def sendGoal():
    rospy.init_node('movearm_pub')
    pub = rospy.Publisher('/mmrobot/joint_trajectory_controller/command', JointTrajectory,
queue_size = 10)
    myTraj = JointTrajectory()
    rate = rospy.Rate(10)

    myTraj.header = Header()
    myTraj.header.stamp = rospy.Time.now() + rospy.Duration(100.0)

    myTraj.joint_names.append('mmrobot_joint1')
    myTraj.joint_names.append('mmrobot_joint2')
    myTraj.joint_names.append('mmrobot_joint3')
    myTraj.joint_names.append('mmrobot_joint4')
    myTraj.joint_names.append('mmrobot_joint5')
    myTraj.joint_names.append('mmrobot_joint6')
    myTraj.joint_names.append('mmrobot_joint7')

    myFirstPoint = JointTrajectoryPoint()
    myFirstPoint.positions.append(0.0)
    myFirstPoint.positions.append(1.0)
    myFirstPoint.positions.append(0.0)
    myFirstPoint.positions.append(0.0)
    myFirstPoint.positions.append(0.0)
    myFirstPoint.positions.append(0.0)
    myFirstPoint.positions.append(0.0)

    myTraj.points.append(myFirstPoint)
    myTraj.points[0].time_from_start  = rospy.Duration(2.0)
    myTraj.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    while not rospy.is_shutdown():
        pub.publish(myTraj)


if __name__ == "__main__":
    try:
        sendGoal()
    except rospy.ROSInterruptException: pass

