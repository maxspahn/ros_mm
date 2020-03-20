#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class MultiJointPublisher():

    """"""

    def __init__(self):
        rospy.init_node('multiJointController')
        self.multiControlSub = rospy.Subscriber("/mmrobot/multijoint_command",
Float64MultiArray, self.multijoint_callback)
        self.publisher =[]
        for i in range(7):
            topicName = '/mmrobot/mmrobot_joint' + str(i+1) + '_controller/command'
            self.publisher.append(rospy.Publisher(topicName, Float64, queue_size=10))
        self.rate = rospy.Rate(10)

    def multijoint_callback(self, data):
        f = Float64()
        for i in range(7):
            f = data.data[i]
            self.publisher[i].publish(f)

    def runNode(self):
        rospy.spin()
        pass

if __name__ == "__main__":
    try:
        myMultiJointPublisher = MultiJointPublisher()
        myMultiJointPublisher.runNode()
    except rospy.ROSInterruptException: pass
