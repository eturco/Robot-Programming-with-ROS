#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>

geometry_msgs::PoseStamped obj_pose;

std_msgs::Float32 state;

void jointStates(const sensor_msgs::JointState msg) {  //::ConstPtr
    ros::Time now = ros::Time::now();
    state.data = msg.position.back();
};


void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    obj_pose.pose = msg->pose;
};


bool approach(){

    static const std::string PLANNING_GROUP = "manipulator"; //manipulator move_group

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization

    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // Planning to a Pose goal

        geometry_msgs::PoseStamped target_pose;
        geometry_msgs::PoseStamped tool_pose;

        tf::TransformListener listener;
        tf::StampedTransform transform;

        bool tf_ok = true;
        try {
            ros::Time now = ros::Time::now();
            listener.waitForTransform("/world", "/soft_hand_palm_link", now, ros::Duration(3.0));

            listener.lookupTransform("/world", "/soft_hand_palm_link", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            tf_ok = false;
        }
        if (tf_ok) {

            tool_pose.header.stamp = ros::Time::now();
            tool_pose.header.frame_id = "/world";
            tool_pose.pose.orientation.x = transform.getRotation().getX();
            tool_pose.pose.orientation.y = transform.getRotation().getY();
            tool_pose.pose.orientation.z = transform.getRotation().getZ();
            tool_pose.pose.orientation.w = transform.getRotation().getW();
        }

        double ro, po, yo, rt, pt, yt, rn, pn, yn;
        tf::Quaternion q_tool, q_obj, q_new, q_rot, q_rot2;

        quaternionMsgToTF(obj_pose.pose.orientation, q_obj);  // Get the original orientation of 'obj_pose'

        quaternionMsgToTF(tool_pose.pose.orientation, q_tool);
        tf::Matrix3x3(q_tool).getEulerYPR(yt, pt, rt);


        tf::Matrix3x3(q_obj).getEulerYPR(yo, po, ro);
        ROS_INFO("Object - roll: %f, pitch: %f, yaw: %f", ro, po, yo);

        double Delta=0.01;
        tf::Matrix3x3 R;

        q_rot = tf::createQuaternionFromRPY(rt,pt,yt);
        q_tool = q_tool*q_rot.inverse();
        tf::Matrix3x3(q_tool).getEulerYPR(yt, pt, rt);
        ROS_INFO("tool - roll: %f, pitch: %f, yaw: %f", rt, pt, yt); //Reset tool orientation

        double r = 1.57, p = 1.57, y = 0;  // Rotate the previous pose by 90° around X and 90° around Y
        std::vector<double> col;

        q_rot = tf::createQuaternionFromRPY(r, p, y);

        R.setValue(0,1,0,0,0,-1,-1,0,0);
        R.getRPY(r,p,y);
        q_rot = tf::createQuaternionFromRPY(r,p,y);

        obj_pose.pose.position.x = obj_pose.pose.position.x - 0.12*sin(yo);
        obj_pose.pose.position.y = obj_pose.pose.position.y + 0.12*cos(yo); //0.125 0.085

        q_new = q_obj * q_rot;

        q_new.normalize();
        tf::Matrix3x3(q_new).getEulerYPR(yn, pn, rn);

        quaternionTFToMsg(q_new, obj_pose.pose.orientation);
        target_pose.header.frame_id = "world";
        target_pose.pose.position= obj_pose.pose.position;
        target_pose.pose.position.z= obj_pose.pose.position.z + Delta;
        target_pose.pose.orientation = obj_pose.pose.orientation;

        move_group.setPoseTarget(target_pose);


        for (int i=0; i<=2; i++){

            ROS_INFO("Attempt n. %d", i+1);

            move_group.setPlanningTime(10.0);

            move_group.setGoalPositionTolerance(0.01);
            move_group.setGoalOrientationTolerance(0.01);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;

            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                do{
                    move_group.move();
                }while(move_group.move().SUCCESS==0);
                ROS_INFO("MOVE IT");
                return move_group.move().SUCCESS;

            }
        }
};

void lift_off(){

    static const std::string PLANNING_GROUP = "manipulator"; //manipulator move_group

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization

    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Planning to a Pose goal

    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped tool_pose;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    bool tf_ok = true;
    try {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/world", "/soft_hand_palm_link", now, ros::Duration(3.0));

        listener.lookupTransform("/world", "/soft_hand_palm_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        tf_ok = false;
    }
    if (tf_ok) {

        tool_pose.header.stamp = ros::Time::now();
        tool_pose.header.frame_id = "/world";
        tool_pose.pose.position.x = transform.getOrigin().getX();
        tool_pose.pose.position.y = transform.getOrigin().getY();
        tool_pose.pose.position.z = transform.getOrigin().getZ();
        tool_pose.pose.orientation.x = transform.getRotation().getX();
        tool_pose.pose.orientation.y = transform.getRotation().getY();
        tool_pose.pose.orientation.z = transform.getRotation().getZ();
        tool_pose.pose.orientation.w = transform.getRotation().getW();
    }

    target_pose.header.frame_id = "world";
    target_pose.pose.position= tool_pose.pose.position;
    target_pose.pose.position.z= tool_pose.pose.position.z + 0.15;
    target_pose.pose.orientation = tool_pose.pose.orientation;

    move_group.setPoseTarget(target_pose);

    for (int i=0; i<=2; i++){

        ROS_INFO("Attempt n. %d", i+1);

        move_group.setPlanningTime(10.0);

        move_group.setGoalPositionTolerance(0.01);
        move_group.setGoalOrientationTolerance(0.01);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            do{
                move_group.move();
            }while(move_group.move().SUCCESS==0);
            ROS_INFO("MOVE IT");
            break;
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle n;
    ros::Subscriber sub_1 = n.subscribe("/bbox_pose", 1000, callback);
    ros::Subscriber sub_2 = n.subscribe("/soft_hand/joint_states", 1, jointStates);

    //ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("grasp_action");
    //grasp_action::GraspAction grasp;

    std_srvs::SetBool grasp;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    bool S = approach();


    if (S) {
        do {
            ROS_INFO("Wait for Grasp!");
        }while (state.data <= 0.5);
    }

    lift_off();

    //client.waitForExistence(ros::Duration(-1));
    //if (S && client.exists()){
        //grasp.request.data = 1;
        //do{
        //    client.call(grasp);
        //}while(grasp.response.success == 0);

        //if (grasp.response.success == 1 ) {
            //lift_off();

    return 0;

}
