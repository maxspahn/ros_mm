#!/usr/bin/env python

import os
import sys

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "../forcesLib/mobileManipulator/"
    )
)

import rospy
import time
import tf
import numpy as np
import mm_MPC_py
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


class MPCController(object):
    def __init__(self):
        print("Initialization of MPC node")
        rospy.init_node("mpcNode")
        self.pubLeftWheel = rospy.Publisher(
            "/mmrobot/left_wheel/command", Float64, queue_size=10
        )
        self.pubRightWheel = rospy.Publisher(
            "/mmrobot/right_wheel/command", Float64, queue_size=10
        )
        self.pubArm = rospy.Publisher(
            "/mmrobot/multijoint_command", Float64MultiArray, queue_size=10
        )
        self.subOdom = rospy.Subscriber(
            "/mmrobot/ground_truth_odom", Odometry, self.odom_cb
        )
        self.subJointPosition = rospy.Subscriber(
            "/mmrobot/joint_states", JointState, self.jointState_cb
        )
        self.subGoal = rospy.Subscriber(
            "/mmrobot/mpc_goal", Float64MultiArray, self.goal_cb
        )
        self.curU = np.zeros(9)
        self.curState = np.zeros(10)
        self.goal = np.zeros(10)
        ex1Goal = np.array([5, 7, 0, 1, 1, 0, -1, 0, 2.5, 2])
        ex2Goal = np.zeros(10)
        self.goal = ex2Goal
        self.time_horizon = 15
        self.PARAMS = {}
        self.dt = 0.10
        self.problemSetup()
        time.sleep(1)
        print("MPC Node initialized")

    def odom_cb(self, odometry):
        curPos = np.array(
            [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        )
        curQuat = odometry.pose.pose.orientation
        quaternion = (curQuat.x, curQuat.y, curQuat.z, curQuat.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        curOri = np.array([euler[2]])
        self.curState[0:3] = np.concatenate((curPos, curOri))

    def jointState_cb(self, jointStates):
        self.curState[3:11] = np.array(jointStates.position[3:10])
        self.curU[2:10] = np.array(jointStates.velocity[3:10])

    def goal_cb(self, goalData):
        self.goal = np.array(goalData.data)

    def computeError(self):
        return np.linalg.norm(
            self.curState[0:2] - self.goal[0:2]
        ) + 0.5 * np.linalg.norm(self.curState[3:10] - self.goal[3:10])

    def problemSetup(self):
        wheel_radius = 0.08
        wheel_distance = 0.544
        self.setup = np.array([self.dt, wheel_radius, wheel_distance])

    def solve(self):
        """Solves the MPC problem for the current state

        :curState: actual robot state
        :returns: x_exp expected next state
                  u_opt optimal control input for next timestep

        """
        xinit = np.concatenate((self.curState, self.curU))
        x0 = np.tile(xinit, self.time_horizon)
        obstacle = np.array([1.5, 3.5, 0, 3])
        singleParam = np.concatenate((self.setup, self.goal, obstacle))
        print("Goal : ", self.goal)
        print("CurState : ", self.curState)
        print("Difference : ", self.goal - self.curState)
        params = np.tile(singleParam, self.time_horizon)
        self.PARAMS["xinit"] = xinit
        self.PARAMS["x0"] = x0
        self.PARAMS["all_parameters"] = params
        solution = mm_MPC_py.mm_MPC_solve(self.PARAMS)
        x02 = solution[0]["x02"]
        x_exp = x02[0:10]
        u_opt = x02[10:19]
        return [x_exp, u_opt]

    def publishVelocities(self, u):
        u_right = Float64(u[0])
        u_left = Float64(u[1])
        u_joints = Float64MultiArray()
        u_joints.data = []
        for i in range(7):
            u_joints.data.insert(i, u[i + 2])
        self.pubRightWheel.publish(u_right)
        self.pubLeftWheel.publish(u_left)
        self.pubArm.publish(u_joints)

    def singleMPCStep(self):
        [x_exp, u_opt] = self.solve()
        self.curU = u_opt
        self.publishVelocities(u_opt)
        time.sleep(self.dt)

    def executeRequest(self):
        error = 100
        while True:
            oldError = error
            error = self.computeError()
            print(error)
            if error < 0.1:
                break
            self.singleMPCStep()
        self.publishVelocities(np.zeros(9))


if __name__ == "__main__":
    mpc_cont = MPCController()
    while not rospy.is_shutdown():
        mpc_cont.executeRequest()
        print("Request executed")
        print("Waiting for new Requests, you can use the send_mpc_mm_goal script")
        time.sleep(10)
