### ROS for mobile manipulators

ROS implemantation for a mobile manipulator. The manipulator is a combination of the
Clearpath boxer and a Franka Emika Panda manipulator.


### Organization
The repository provides four packages for ros melodic. 
- mobile_manipulator : urdf files and setup for gazebo
- mobile_control : controllers
- mobile_navigation : navigation stack for ros
- mobile_moveit : moveit connfiguration for the manipulator

### Currently working
Currently, the gazebo setup is possible with different control modes for the panda
manipulator. Control of the model in gazebo are able via a normal gamecontroller.
The motion of the base can be controlled using the move_base action client.
Plans can be made using the rviz interface.

In the current state, the **mobile_moveit package is not working properly**.

### Different Control Modes
There are multiple control modes for the panda manipulator: Two different implementation
of PositionControl and VelocityControl.
The more stable position control mode relies on an implentation from
[panda_simulation](https://github.com/erdalpekel/panda_simulation). 
This package must be installed in the catkin_workspace.

### Usage
Launch the gazebo simulation

```
roslaunch mobile_manipulator mmrobot_gazebo.launch panda_hi:="EffortJointInterface"
```

Launch the navigation stack for the mobile base (the boxer)
```
roslaunch mobile_navigation mmrobot_navigation.launch
```

Launch the navigation stack for the manipulator (the panda)
This is only possible when the EffortJointInterface was selected during the gazebo
startup.
```
roslaunch mobile_moveit planning_execution.launch
```


