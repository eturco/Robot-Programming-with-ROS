# Robot-Programming-with-ROS

## Requirements

  ### Install libhwi_switch_gazebo_ros_control.so
  
  from [here](https://github.com/ipa320/cob_gazebo_plugins/tree/kinetic_dev/cob_gazebo_ros_control).

This plugin extends the default plugin available in gazebo_ros_control. In particular, it adds the following additional feature:

- Enable joint filtering

Since the robot is made of a UR5 and a Pisa/IIT hand, we need to assign only a specific set of joints to one plugin and then use several plugins under different robotNamespaces. This is possible using the tag filterJointsParam.

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

## Launch Grasp action

```
roslaunch ur5_plan_grasp graspit.launch 
```
