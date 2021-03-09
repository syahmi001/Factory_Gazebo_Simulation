# two_scara_collaboration

Simulation of two SCARA robots collaboratively sorting items from a conveyor belt with collision avoidance and task distribution.

Check the video demonstration here: [https://youtu.be/I47J0p1j1zU](https://youtu.be/I47J0p1j1zU)

## Test command
```
roslaunch two_scara_collaboration initialize.launch
```
Gazebo simulator is paused when initialized, in case the arms of the two scara robots appear at same location and bounce away each other. Click start button to start.

Start the two independant motion planners
```
rosrun two_scara_collaboration scara_left_motion_planner
```
```
rosrun two_scara_collaboration scara_right_motion_planner
```


## Progress (May 7, 2016)
Switch the joint control method from off-the-shelf ros_control package to self-defined PD controller.

Collision avoidance using a further simplified algorithm to dynamically allocate each robot's workspace. Task distribution follows a first come, first get principle. One important thing is that, the motion planners are independant. One planner for one SCARA robot. And they are pretty robust observed from the video.

Some parameters, like time and exerted force or torque can be further tuned, to get better motion performance.


## Progress and problems (May 3, 2016)
The scara robots, conveyor belt and cylinder blocks were modeled in urdf.

A spawner node can automatically spawn cylinder blocks and let them slide on conveyor with an initial speed.

A cylinder blocks position publisher node was added to publish the 3-D position of all current cylinders.

A gripper control action server node can keep gripper at desired location, and receiving command on the gripper action.

Steps not finished in the plan:

    joint position controller of scara (problem with ros_control)

    inverse kinematics of scara robot

    collision avoidance

    task distribution

The problem with position control of scara using ros_control is that, when load the controller from a launch file, it always looks for robot model at "/robot_description", instead of what I defined "/scara_robot_left/robot_description" in the parameter server. I can change name to"/robot_description", but there is no good way to control multiple different robots and having their models under same name in parameter. Some posts about this problem:

[gazebo ros control tutorial](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros)

[adding robot_description to parameter server](http://answers.ros.org/question/61479/adding-robot_description-to-parameter-server/)

[Running controller_manager spawner with /mybot/robot_description?](http://answers.ros.org/question/198929/running-controller_manager-spawner-with-mybotrobot_description/)

[gazebo_ros_control: Use the model NodeHandle to get the robot_description parameter](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/134)

[Using gazebo_ros_control with SDF instead of URDF](http://answers.ros.org/question/223196/using-gazebo_ros_control-with-sdf-instead-of-urdf/)

[gazebo_ros_control: plugin doesn't look for the robot model in its own namespace](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/112)




