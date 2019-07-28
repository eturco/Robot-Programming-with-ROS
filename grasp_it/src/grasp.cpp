//
// Created by enrico on 23/07/19.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"

#include <stdlib.h>
#include <string>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"
#include <geometry_msgs/PoseArray.h>

#include <controller_manager/controller_manager.h>

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Char.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <std_srvs/SetBool.h>

ros::Publisher pub;
std_msgs::Float64 state;

//bool ack(std_srvs::SetBool::Request  &req,
//         std_srvs::SetBool::Response &res)
// {
//   res.success = 1;
//   ROS_INFO("Grasp!");
//   return true;
// };

void jointStates(const sensor_msgs::JointState msg) {
    ros::Time now = ros::Time::now();
    state.data = msg.position.back();
};

int setValeurPoint(trajectory_msgs::JointTrajectory* trajectoire,int pos_tab, double val){
    trajectoire->points[0].positions[pos_tab] = val;
    return 0;
}

void grasp(double i){

    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint points_n;

    traj.header.frame_id = "base_link";
    traj.joint_names.resize(1);
    traj.points.resize(1);

    traj.points[0].positions.resize(1);

    traj.joint_names[0] ="soft_hand_synergy_joint";

    traj.header.stamp = ros::Time::now();

    for(int j=0; j<1; j++)
    {
            setValeurPoint(&traj,j,i);
    }

    traj.points[0].time_from_start = ros::Duration(1);

    pub.publish(traj);

    ros::spinOnce();

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp");
    ros::NodeHandle nh("~");
    ros::Subscriber sub;

    ros::Rate loop_rate(10);


    sub = nh.subscribe("/soft_hand/joint_states", 1, jointStates);
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("/soft_hand/soft_hand/joint_trajectory_controller/command", 1);

    double g;

    nh.getParam("grasp_value", g);    /// 0 - OPEN THE HAND    /// 1 - CLOSE THE HAND


    while(ros::ok()) {
        grasp(g);

        //if (state.data >= 0.5 ){
           // ROS_INFO("GRASP IT");
           // ros::ServiceServer service = nh.advertiseService("grasp_action", ack);
            loop_rate.sleep();
        //}
    }
    return 0;
}
