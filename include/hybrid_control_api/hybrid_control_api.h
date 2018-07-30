#ifndef HYBRID_CONTROL_H
#define HYBRID_CONTROL_H

#include <vector>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

void parseWeightData(const std::string &path);

bool hybrid(hybrid_control_api::hybrid::Request &req,
             hybrid_control_api::hybrid::Response &res);

thormang3::KinematicsDynamics *robotis_;

/* inverse kinematics */
Eigen::MatrixXd ik_weight_;

#endif

