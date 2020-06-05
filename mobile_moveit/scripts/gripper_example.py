#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    :param: goal       A list of floats, a Pose or a PoseStamped
    :param: actual     A list of floats, a Pose or a PoseStamped
    :param: tolerance  A float
    :rtype: bool
    """
    if isinstance(goal, list):
        for index, g in enumerate(goal):
          if abs(actual[index] - g) > tolerance:
            return False

    elif isinstance(goal, geometry_msgs.msg.PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)

    elif isinstance(goal, geometry_msgs.msg.Pose):
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class PandaMoveGroupInterface:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander(robot_description="mmrobot/robot_description",ns="mmrobot")

        self._scene = moveit_commander.PlanningSceneInterface()

        self._arm_group = moveit_commander.MoveGroupCommander("arm_group", robot_description="mmrobot/robot_description")

        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                           moveit_msgs.msg.DisplayTrajectory,
                                           queue_size=20)

        self._default_ee = 'mmrobot_hand'
        self._arm_group.set_end_effector_link(self._default_ee)

        self._gripper_group = moveit_commander.MoveGroupCommander("hand", robot_description="mmrobot/robot_description")

    @property
    def robot_state_interface(self):
        """
            :getter: The RobotCommander instance of this object
            :type: moveit_commander.RobotCommander
        
            .. note:: For available methods for RobotCommander, refer `RobotCommander <http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html>`_.
                
        """
        return self._robot
    
    @property
    def scene(self):
        """
            :getter: The PlanningSceneInterface instance for this robot. This is an interface
                    to the world surrounding the robot
            :type: franka_moveit.ExtendedPlanningSceneInterface
            .. note:: For other available methods for planning scene interface, refer `PlanningSceneInterface <http://docs.ros.org/indigo/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html>`_. 
        """
        return self._scene

    @property
    def arm_group(self):
        """
        :getter: The MoveGroupCommander instance of this object. This is an interface
            to one group of joints.  In this case the group is the joints in the Panda
            arm. This interface can be used to plan and execute motions on the Panda.
        :type: moveit_commander.MoveGroupCommander
        .. note:: For available methods for movegroup, refer `MoveGroupCommander <http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html>`_. 
        """
        return self._arm_group

    @property
    def gripper_group(self):
        """
        :getter: The MoveGroupCommander instance of this object. This is an interface
            to one group of joints.  In this case the group is the joints in the Panda
            arm. This interface can be used to plan and execute motions on the Panda.
        :type: moveit_commander.MoveGroupCommander
        .. note:: For available methods for movegroup, refer `MoveGroupCommander <http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html>`_. 
        """
        return self._gripper_group

    def go_to_joint_positions(self, positions, wait = True, tolerance = 0.005):
        """
            :return: status of joint motion plan execution
            :rtype: bool
            :param positions: target joint positions (ordered)
            :param wait: if True, function will wait for trajectory execution to complete
            :param tolerance: maximum error in final position for each joint to consider
             task a success
            :type positions: [double]
            :type wait: bool
            :type tolerance: double
        """
        self._arm_group.clear_pose_targets()

        rospy.logdebug("Starting positions: {}".format(self._arm_group.get_current_joint_values()))

        self._arm_group.go(positions[:7], wait = wait)

        if len(positions) > 7:
            self._gripper_group.go(positions[7:], wait = wait)

        if wait:
            self._arm_group.stop()
            gripper_tolerance = True
            if len(positions)> 7: 
                self._gripper_group.stop()
                gripper_tolerance = all_close(positions[7:], self._gripper_group.get_current_joint_values(), tolerance)
            return all_close(positions[:7], self._arm_group.get_current_joint_values(), tolerance) and gripper_tolerance

        return True

    def plan_cartesian_path(self, poses):
        """
            Plan cartesian path using the provided list of poses.
            :param poses: The cartesian poses to be achieved in sequence. 
                (Use :func:`franka_moveit.utils.create_pose_msg` for creating pose messages easily)
            :type poses: [geomentry_msgs.msg.Pose] 
        """
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))

        plan, fraction = self._arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def set_velocity_scale(self, value, group = "arm"):
        """
            Set the max velocity scale for executing planned motion.
            
            :param value: scale value (allowed (0,1] )
            :type value: float
        """
        if group == "arm":
            self._arm_group.set_max_velocity_scaling_factor(value)
        elif group == "gripper":
            self._gripper_group.set_max_velocity_scaling_factor(value)
        else:
            raise ValueError("PandaMoveGroupInterface: Invalid group specified!")
        rospy.logdebug("PandaMoveGroupInterface: Setting velocity scale to {}".format(value))

    def plan_joint_path(self, joint_position):
        """
        :return: plan for executing joint trajectory
        :param joint_position: target joint positions
        :type joint_position: [float]*7
        """
        return self._arm_group.plan(joint_position)


    def close_gripper(self, wait = True):
        """
            Close gripper. (Using named states defined in urdf.)
            .. note:: If this named state is not found, your ros environment is
                probably not using the right panda_moveit_config package. Ensure
                that sourced package is from this repo -->
                https://github.com/justagist/panda_moveit_config
        """
        self._gripper_group.set_named_target("closed")
        return self._gripper_group.go(wait = wait)

    def open_gripper(self, wait = False):
        """
            Open gripper. (Using named states defined in urdf)
            .. note:: If this named state is not found, your ros environment is
                probably not using the right panda_moveit_config package. Ensure
                that sourced package is from this repo -->
                https://github.com/justagist/panda_moveit_config.
        """
        self._gripper_group.set_named_target("open")
        return self._gripper_group.go(wait = wait)

    def display_trajectory(self, plan):
        """
        Display planned trajectory in RViz. Rviz should be open and Trajectory
        display should be listening to the appropriate trajectory topic.
        :param plan: the plan to be executed (from :func:`plan_joint_path` or
            :py:meth:`plan_cartesian_path`)
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self._display_trajectory_publisher.publish(display_trajectory)

    def move_to_neutral(self, wait = True):
        """
            Send arm group to neutral pose defined using named state in urdf.
            :param wait: If True, will wait till target is reached
            :type wait: bool
        """
        self._arm_group.set_named_target("ready")
        return self._arm_group.go(wait = wait)

    def execute_plan(self, plan, group = "gripper", wait = True):
        """
            Execute the planned trajectory 
            :param plan: The plan to be executed (from :py:meth:`plan_joint_path` or
                :py:meth:`plan_cartesian_path`)
            :param group: The name of the move group (default "arm" for robot; use "hand" 
                for gripper group)
            :type group: str
            :param wait: If True, will wait till plan is executed
        """
        if group == "arm":
            self._arm_group.execute(plan, wait = wait)
        elif group == "gripper":
            self._gripper_group.execute(plan, wait = wait)
        else:
            raise ValueError("PandaMoveGroupInterface: Invalid group specified!")

if __name__ == '__main__':

    rospy.init_node("test_moveit")
    mvt = PandaMoveGroupInterface()

    mvt.close_gripper()
    rospy.sleep(3)
    mvt.open_gripper()
    #mvt.execute_plan("close")
