/**
 * /file M1DemoMachineROS.h
 * /author Emek Baris Kucuktabak
 * /brief ROS part of the M1DemoMachine
 * /version 0.1
 * /date 2020-11-03
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_M1DEMOMACHINEROS_H
#define SRC_M1DEMOMACHINEROS_H

// msg types
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>

#include "RobotM1.h"
#include "ros/ros.h"  // This state machine requires ROS

class M1DemoMachineROS {
public:
    M1DemoMachineROS(RobotM1 *robot);
    ~M1DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

    ros::NodeHandle* nodeHandle_;

    Eigen::VectorXd jointPositionCommand_, jointVelocityCommand_, jointTorqueCommand_;
    Eigen::VectorXd interactionTorqueCommand_;

private:
    // Subscriber and callback func for joint command subscription
    ros::Subscriber jointCommandSubscriber_;
    void jointCommandCallback(const sensor_msgs::JointState &msg);

    // Subscriber and callback func for interaction torque subscription
    ros::Subscriber interactionTorqueCommandSubscriber_;
    void interactionTorqueCommandCallback(const std_msgs::Float64MultiArray &msg);

    // Publisher and message for joint state publication
    ros::Publisher jointStatePublisher_;
    sensor_msgs::JointState jointStateMsg_;

    // Publisher and message for interaction wrench publication
//    ros::Publisher interactionWrenchPublisher_;
//    geometry_msgs::WrenchStamped interactionWrenchMsg_;
    RobotM1 *robot_;

    // dynamic reconfigure server and callback
//    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server;
//    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);
};

#endif  //SRC_M1DEMOMACHINEROS_H
