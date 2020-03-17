### ROS for mobile manipulators

ROS implemantation for a mobile manipulator. The manipulator is a combination of the
Clearpath boxer and a Franka Emika Panda manipulator.

In the long term, they should be seen as one robot performing mobile manipulation tasks
in crowded environments.

### Organization
The repository provides four packages for ros melodic. 
mobile_manipulator : urdf files and setup for gazebo
mobile_control : controllers
mobile_navigation : navigation stack for ros
mobile_moveit : moveit connfiguration for the manipulator

### Simple start up
Launch the gazebo simulation
```
roslaunch mobile_manipulator mmrobot_gazebo.launch
```

Launch the navigation stack for the mobile base (the boxer)
```
roslaunch mobile_navigation mmrobot_navigation.launch
```

Launch the navigation stack for the manipulator (the panda)
```
roslaunch mobile_moveit planning_execution.launch
```

