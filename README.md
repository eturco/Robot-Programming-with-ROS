# Robot-Programming-with-ROS

## Requirements

  ### PCL Libraries

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

## Launch vision node

```
roslaunch ur5_plan_grasp vision.launch 
```

## Launch trajectory planner

```
roslaunch ur5_plan_grasp trajectory_planner.launch 
```

## Grasp action

```
roslaunch ur5_plan_grasp graspit.launch 
```

## Rviz visualization

```
roslaunch ur5_moveit moveit_rviz.launch
```
