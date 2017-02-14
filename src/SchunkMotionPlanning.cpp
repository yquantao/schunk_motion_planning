#include "SchunkMotionPlanning.h"

using namespace std;

SchunkMotionPlanning::SchunkMotionPlanning()
{
    // Load the robot model.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    m_RobotModel = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", m_RobotModel->getModelFrame().c_str());
    m_RobotState = robot_state::RobotStatePtr(new robot_state::RobotState(m_RobotModel));
    m_RobotState->setToDefaultValues();
    m_pJointModelGroup = m_RobotModel->getJointModelGroup("lwa");

    // Implementation of topics to subscribe.
    m_topicSub_CmdVel = m_nh.subscribe("/schunk/cmd_vel", 1, &SchunkMotionPlanning::topicCallback_CmdVel, this);
    m_topicSub_JointState = m_nh.subscribe("/joint_states", 1, &SchunkMotionPlanning::topicCallback_JointState, this);
    // Implementation of topics to publish.
    //m_topicPub_JointVel = m_nh.advertise<brics_actuator::JointVelocities>("command_vel", 1);
}

SchunkMotionPlanning::~SchunkMotionPlanning()
{
    if (m_pJointModelGroup != NULL)
        delete m_pJointModelGroup;
}

void SchunkMotionPlanning::topicCallback_JointState(const sensor_msgs::JointState& joint_state)
{
    m_JointState = joint_state;

    //m_RobotState->setVariableValues(joint_state);
}

void SchunkMotionPlanning::topicCallback_CmdVel(const geometry_msgs::Twist& vel)
{
    ROS_INFO("Camera velocity in the reference frame:\n");
    cout<<vel<<"\n";

    // angular and linear camera velocity in the object reference frame
    Eigen::VectorXd twistAng(3);
    Eigen::VectorXd twistLin(3);
    // angular and linear camera velocity in the robot base frame
    Eigen::Vector3d outAng(3);
    Eigen::Vector3d outLin(3);
    // 6-dimension transformed camera velocity in robot base frame
    //Eigen::VectorXd cmd_vel(6);

    // initialize angular velocity
    twistAng(0)=vel.angular.x;
    twistAng(1)=vel.angular.y;
    twistAng(2)=vel.angular.z;
    // initialize linear velocity
    twistLin(0)=vel.linear.x;
    twistLin(1)=vel.linear.y;
    twistLin(2)=vel.linear.z;

    //********rotation matrix************
    Eigen::Matrix3d Rot, RotX, RotY, RotZ;
    double angleX, angleY, angleZ;
    double Pi = 3.1415926;
    angleX = 90*Pi/180;
    angleY = -180*Pi/180;
    angleZ = 0*Pi/180;
    // Rotation around the reference X axis.
    RotX << 1,     0,            0,
            0, cos(angleX), -sin(angleX),
            0, sin(angleX),  cos(angleX);
    // Rotation around the reference Y axis.
    RotY << cos(angleY),  0, sin(angleY),
                0,        1,     0,
            -sin(angleY), 0, cos(angleY);
    // Rotation around the reference Z axis.
    RotZ << cos(angleZ), -sin(angleZ), 0,
            -sin(angleZ), cos(angleZ), 0,
                 0,           0,       1;
    // Calculate the whole rotation matrix.
    Rot = RotZ * RotY * RotX;

    //*********translation vector**********
    double px, py, pz;
    px = 0.0;
    py = -1.0;
    pz = 0.5;
    Eigen::Vector3d Trans(px, py, pz);
    // Calculate the out velocity.
    outAng = Rot * twistAng;
    outLin = Rot * twistLin + Trans.cross(outAng);

    // Write to m_CmdVel.
    m_CmdVel.resize(6);
    m_CmdVel(0)=outLin(0);
    m_CmdVel(1)=outLin(1);
    m_CmdVel(2)=outLin(2);
    m_CmdVel(3)=outAng(0);
    m_CmdVel(4)=outAng(1);
    m_CmdVel(5)=outAng(2);

    // Transformed velocity
    geometry_msgs::Twist out_vel;
    out_vel.linear.x=outLin(0);
    out_vel.linear.y=outLin(1);
    out_vel.linear.z=outLin(2);
    out_vel.angular.x=outAng(0);
    out_vel.angular.y=outAng(1);
    out_vel.angular.z=outAng(2);
    cout<<"Transformed camera velocity:\n";
    cout<<out_vel<<"\n";
}

void SchunkMotionPlanning::computeJointVel(const sensor_msgs::JointState& joint_state)
{
    m_RobotState->setVariableValues(joint_state);
    m_JointVel.resize(7);
    if (m_CmdVel.size() > 0)
        m_RobotState->computeVariableVelocity(m_pJointModelGroup, m_JointVel, m_CmdVel,
                                         m_RobotState->getLinkModel(m_pJointModelGroup->getLinkModelNames().back()));
}

void SchunkMotionPlanning::publishJointVel()
{
    const unsigned int uiDOF = 7;
    stringstream ssJointNames;

    // Set up joint velocity msg.
    brics_actuator::JointVelocities joint_velocity_msg;
    joint_velocity_msg.velocities.resize(uiDOF);
    for(unsigned int i = 0; i < uiDOF; i++)
    {
        //joint_velocity_msg.poisonStamp = ros::Time::now();
        ssJointNames << "arm_" << i+1 << "_joint";
        joint_velocity_msg.velocities[i].joint_uri = ssJointNames.str();
        joint_velocity_msg.velocities[i].unit = "rad";
        joint_velocity_msg.velocities[i].value = m_JointVel(i);
    }

    // Publish joint velocity msg.
    ROS_INFO("Start to publish joint velocities.");
    m_topicPub_JointVel.publish(joint_velocity_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "schunk_motion_planning");
    //ros::AsyncSpinner spinner(2);
    //spinner.start();
    // Create a Schunk Motion Planning node.
    SchunkMotionPlanning m_node;

    /***************Calculate and publish joint velocity.******************/
    //m_node.m_JointVel.resize(7);
    ros::Rate r(10); // 10 hz
    while(m_node.m_nh.ok())
    {
        //ros::spin();
        ROS_INFO("test");

        // Compute the joint velocities.
        m_node.computeJointVel(m_node.m_JointState);
        // Publish the joint velocities.
        //m_node.publishJointVel();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
