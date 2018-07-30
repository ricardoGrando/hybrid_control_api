#include "ros/ros.h"
#include "hybrid_control_api/hybrid.h"
#include "hybrid_control_api/hybrid_control_api.h"
#include <stdio.h>
#include <iostream>

void parseWeightData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    //ROS_INFO("Trying to load YAML file");
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    //ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node ik_weight_node = doc["weight_value"];
  for (YAML::iterator it = ik_weight_node.begin(); it != ik_weight_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    ik_weight_.coeffRef(id, 0) = value;
    //ROS_INFO("ik_weight_.coeffRef(%d, 0) = %f", id, value);
  }
}

bool hybrid(hybrid_control_api::hybrid::Request &req,
             hybrid_control_api::hybrid::Response &res)
{
  
  // od of the joints.
  int ik_id_start_ = 2;
  int ik_id_end_ = 14;  // 14 is the last 34 is the end effector


  // set all joint angles
  for (auto jointPose = req.currJointPose.begin(); jointPose != req.currJointPose.end(); jointPose++)
  {
    for (int id = ik_id_start_ ; id <= ik_id_end_ ; id += 2)
    {
      if (robotis_->thormang3_link_data_[id]->name_ == jointPose->name)
      {
        robotis_->thormang3_link_data_[id]->joint_angle_ = jointPose->value;
      }
    }
  }  

  // calc the forward kinematics for the left arm (from 2 to 34). Starts in 2
  robotis_->calcForwardKinematics(2); 


  // // calc werr

  // Eigen::MatrixXd curr_position = robotis_->thormang3_link_data_[34]->position_;
  // Eigen::MatrixXd curr_orientation = robotis_->thormang3_link_data_[34]->orientation_;

  // //OS_INFO("Creating desired position.");
  // Eigen::MatrixXd target_position  = Eigen::MatrixXd::Zero(3,1);
  // target_position.coeffRef(0, 0) = req.desiredPose.position.x;
  // target_position.coeffRef(1, 0) = req.desiredPose.position.y;
  // target_position.coeffRef(2, 0) = req.desiredPose.position.z;

  // //ROS_INFO("Creating desired orientation.");
  // Eigen::MatrixXd target_rotation  = Eigen::MatrixXd::Zero(4,1);
  // target_rotation.coeffRef(0, 0) = req.desiredPose.orientation.x;
  // target_rotation.coeffRef(1, 0) = req.desiredPose.orientation.y;
  // target_rotation.coeffRef(2, 0) = req.desiredPose.orientation.z;
  // target_rotation.coeffRef(3, 0) = req.desiredPose.orientation.w;

  // Eigen::MatrixXd err = robotis_->calcVWerr(target_position, curr_position, target_rotation, curr_orientation);

  // // returh the values
  // jointPose.value = err(0,0);
  // res.targJointPose.push_back(jointPose);
  // jointPose.value = err(1,0);
  // res.targJointPose.push_back(jointPose);
  // jointPose.value = err(2,0);
  // res.targJointPose.push_back(jointPose);
  // jointPose.value = err(3,0);
  // res.targJointPose.push_back(jointPose);
  // jointPose.value = err(4,0);
  // res.targJointPose.push_back(jointPose);
  // jointPose.value = err(5,0);
  // res.targJointPose.push_back(jointPose);

  thormang3_manipulation_module_msgs::JointPose jointPose;

  jointPose.value = 0;
  res.targJointPose.push_back(jointPose);
  jointPose.value = 0;
  res.targJointPose.push_back(jointPose);
  jointPose.value = 0;
  res.targJointPose.push_back(jointPose);
  jointPose.value = 0;
  res.targJointPose.push_back(jointPose);
  jointPose.value = 0;
  res.targJointPose.push_back(jointPose);
  jointPose.value = 0;
  res.targJointPose.push_back(jointPose);





 
  
  

   
  // joints id's for jacobian calculation
  std::vector<int> idx = robotis_->findRoute(ik_id_start_, ik_id_end_);
  
  //calc the jacobian for them
  Eigen::MatrixXd jacobian = robotis_->calcJacobian(idx);

  ROS_INFO("Here is the jacobian matrix m:");
  std::cout << jacobian << std::endl;

  ik_id_start_ = 2;
  ik_id_end_ = 34;  // 14 is the last 34 is the end effector

  idx = robotis_->findRoute(ik_id_start_, ik_id_end_);





  //thormang3_manipulation_module_msgs::JointPose jointPose;
        

  jointPose.name = robotis_->thormang3_link_data_[ik_id_end_]->name_;
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->position_[0];
  res.targJointPose.push_back(jointPose);
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->position_[1];
  res.targJointPose.push_back(jointPose);
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->position_[2];
  res.targJointPose.push_back(jointPose);
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->orientation_[0];
  res.targJointPose.push_back(jointPose);
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->orientation_[1];
  res.targJointPose.push_back(jointPose);
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->orientation_[2];
  res.targJointPose.push_back(jointPose);
  jointPose.value = robotis_->thormang3_link_data_[ik_id_end_]->orientation_[3];
  res.targJointPose.push_back(jointPose);

  // std::cout << "Position Ennd-effector:\n" << robotis_->thormang3_link_data_[34]->position_ << std::endl;
  // std::cout << "Orientation Ennd-effector:\n" << robotis_->thormang3_link_data_[34]->orientation_ << std::endl;

  jointPose.value = jacobian(0,0);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(0,1);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(0,2);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(0,3);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(0,4);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(0,5);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(0,6);
  res.targJointPose.push_back(jointPose);

  jointPose.value = jacobian(1,0);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(1,1);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(1,2);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(1,3);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(1,4);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(1,5);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(1,6);
  res.targJointPose.push_back(jointPose);

  jointPose.value = jacobian(2,0);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(2,1);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(2,2);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(2,3);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(2,4);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(2,5);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(2,6);
  res.targJointPose.push_back(jointPose);

  jointPose.value = jacobian(3,0);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(3,1);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(3,2);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(3,3);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(3,4);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(3,5);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(3,6);
  res.targJointPose.push_back(jointPose);

  jointPose.value = jacobian(4,0);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(4,1);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(4,2);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(4,3);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(4,4);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(4,5);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(4,6);
  res.targJointPose.push_back(jointPose);

  jointPose.value = jacobian(5,0);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(5,1);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(5,2);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(5,3);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(5,4);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(5,5);
  res.targJointPose.push_back(jointPose);
  jointPose.value = jacobian(5,6);
  res.targJointPose.push_back(jointPose);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fk_jacobian_werr_server");
  ros::NodeHandle n;

  // Setup
  //ROS_INFO("Loading link weight info");
  ik_weight_            = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1, 1);
  ik_weight_.fill(1.0);
  std::string _path = ros::package::getPath("hybrid_control_api") + "/config/ik_weight.yaml";
  parseWeightData(_path);
  
  //ROS_INFO("Creating the KinematicsDynamics object");
  robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
  // These are declared in thormang3_kinematics_dynamics/kinematics_dynamics.h

  ros::ServiceServer service = n.advertiseService("hybrid_control_api/hybrid", hybrid);
  ROS_INFO("The service is ready!");

  ros::spin();

  return 0;
}

