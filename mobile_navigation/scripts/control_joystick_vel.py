#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

class ControllerNode():

    """Docstring for controllerNode.

     b6                     b7
    b4                       b5
--------------------------------------
      a5     b8     b9       b3
   -a4  a4    --   --      b0  b2
     -a5                     b1
           a1           a3
         a0 -a0       a2 -a2
          -a1          -a3
    """

    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.joySub = rospy.Subscriber("joy", Joy, self.joystick_listener_callback)
        self.jointStatesSub = rospy.Subscriber("mmrobot/joint_states", JointState,
self.joint_state_listener_callback)
        self.armPub = rospy.Publisher('/mmrobot/multijoint_command', Float64MultiArray, queue_size=10)
        self.basePub = rospy.Publisher('/mmrobot/boxer_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.activeJoint = 1
        self.jointTarget = Float64MultiArray()
        self.baseVel = Twist()


    def setPosition(self):
        self.armPub.publish(self.jointTarget)
        self.basePub.publish(self.baseVel)

    def joystick_listener_callback(self, data):
        if (data.buttons[4] == 1 and self.activeJoint > 0) : self.activeJoint -= 1
        if (data.buttons[5] == 1 and self.activeJoint < 7) : self.activeJoint += 1
        if (data.buttons[1]) : self.goHomePosition()
        self.jointTarget.data[self.activeJoint] += 0.01 * data.axes[5]
        self.baseVel.linear.x += 0.03 * data.axes[3]
        self.baseVel.angular.z += 0.01 * data.axes[2]
        self.setPosition()
        print(self.jointTarget.data)
        print("Active Joint : ", self.activeJoint)
        rospy.loginfo("Received new commands")

    def joint_state_listener_callback(self, data):
        self.jointStates = []
        for i in range(7):
            self.jointStates.append(data.position[i+3])
        #print(self.jointStates)
        #print(self.f.data)

    def goHomePosition(self):
        self.jointTarget.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.baseVel = Twist()
        self.setPosition()


    def runNode(self):
        rospy.spin()

    def printInfo(self):
        print("Press Up/Down/Left/Right to move the joint")
        print("Change the joint with R1/L1")

if __name__ == "__main__":
    try:
        myControlNode = ControllerNode()
        myControlNode.goHomePosition()
        myControlNode.printInfo()
        myControlNode.runNode()
    except rospy.ROSInterruptException:
        pass
