# Robot-Programming-with-ROS

## Requirements

  ### Install the plugin libhwi_switch_gazebo_ros_control.so
  
  Download it from [here](https://github.com/ipa320/cob_gazebo_plugins/tree/kinetic_dev/cob_gazebo_ros_control).

This plugin extends the default plugin available in gazebo_ros_control. In particular, it adds the following additional feature:

- Enable joint filtering

Since the robot is made of a UR5 and a Pisa/IIT hand, we need to assign only a specific set of joints to one plugin and then use several plugins under different robotNamespaces. This is possible using the tag filterJointsParam.

  Install it, following this procedure.
  
  - Create the build directory
```
$ mkdir ~/cob_gazebo_plugins/cob_gazebo_ros_control/build
$ cd ~/cob_gazebo_plugins/cob_gazebo_ros_control/build
```
- Compile the code.
```
$ cmake ../
$ make
```
Compiling will result in a shared library,  ` ~/cob_gazebo_plugins/cob_gazebo_ros_control/build/devel/lib/libhwi_switch_gazebo_ros_control.so`, that can be inserted in a Gazebo simulation.

- Lastly, add your library path to the GAZEBO_PLUGIN_PATH:
```
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/cob_gazebo_plugins/cob_gazebo_ros_control/build/devel/lib/

```
Note: This changes the path only for the current shell. If you want to use your plugin for every new temrinal you open, append the line above to the `~/.bashrc` file.


  ### Install Point Cloud Libraries

```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

## Launch the simulation

```
roslaunch ur5_plan_grasp ur5_sim.launch 
```
Spawn the UR5 robot and Pisa/IIT hand on a table.

The following controllers are enabled:
```
-joint_state_controller
-ur5/arm_controller/joint_trajectory_controller
-ur5/arm_controller/follow_joint_trajectory
-soft_hand/joint_trajectory_controller
```
## Launch Rviz

```
roslaunch ur5_moveit moveit_rviz.launch
```

## Launch vision node

```
roslaunch ur5_plan_grasp vision.launch 
```
In Rviz you can visualize:
  - PointCloud2 
  - MarkerArray
  - PoseArray 

## Launch trajectory planner

```
roslaunch ur5_plan_grasp trajectory_planner.launch 
```
Two trajectories will be planned: 
- Approach: the hand will go close to the object
- Lift off: the hand will raise up while grabbing the object

For the second trajectory the planner will wait for the success of the grasping action.

## Launch Grasp action

```
roslaunch ur5_plan_grasp graspit.launch 
```
For a tighter grasp you can adjust the value of the parameter `grasp_value` within the launch file.
