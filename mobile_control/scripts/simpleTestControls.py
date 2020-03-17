#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def moveBoth():
    rospy.init_node('simpleCommander')
    #pubArm = rospy.Publisher('/mmrobot/joint_position_controller/command',
    pubArm = rospy.Publisher('/mmrobot/panda_arm_controller/command',
Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    f = Float64MultiArray()
    f.data = [0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
    pub = rospy.Publisher('/mmrobot/boxer_velocity_controller/cmd_vel', Twist,
queue_size=10)
    t = Twist()
    t.linear.x = 10.0
    t.angular.z = 10.0
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
        #pub.publish(t)

if __name__ == "__main__":
    try:
        moveBoth()
    except rospy.ROSInterruptException: pass
