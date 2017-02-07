#ifndef _SCHUNK_MOTION_PLANNING_H
#define _SCHUNK_MOTION_PLANNING_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
//#include <Eigen/Dense>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <brics_actuator/JointVelocities.h>

// MoveIt includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

class SchunkMotionPlanning
{
public:
    /// Constructor
    SchunkMotionPlanning();
    /// Destructor
    ~SchunkMotionPlanning();
    /// Member function
    void topicCallback_JointState(const sensor_msgs::JointState& joint_state);
    void topicCallback_CmdVel(const geometry_msgs::Twist& vel);
    void computeJointVel(const sensor_msgs::JointState& joint_state);

    void publishJointVel();

public:
    /// node handle
    ros::NodeHandle m_nh;

    /// Declaration of topics to subscribe.
    ros::Subscriber m_topicSub_JointState;
    ros::Subscriber m_topicSub_CmdVel;

    /// Declaration of topics to publish.
    ros::Publisher m_topicPub_JointVel;

    /// Joint state. 
    sensor_msgs::JointState m_JointState;
    /// Joint velocities.
    Eigen::VectorXd m_CmdVel;
    Eigen::VectorXd m_JointVel;
    /// Robot model.
    robot_model::RobotModelPtr m_RobotModel;
    robot_state::RobotStatePtr m_RobotState;
    const robot_state::JointModelGroup* m_pJointModelGroup;
};

#endif // _SCHUNK_MOTION_PLANNING_H
