#pragma once
/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <controller_interface/controller.h>
#include <subscription_notifier/subscription_notifier.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosdyn_core/primitives.h>
#include <name_sorting/name_sorting.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace cnr_control
{
class CartImpedanceController : public cnr::control::JointCommandController<
        hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate  (const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:
  hardware_interface::PosVelEffJointInterface* m_hw;
  std::vector<hardware_interface::PosVelEffJointHandle> m_joint_handles;
  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  ros::CallbackQueue m_queue;

  unsigned int m_nAx;
  std::vector< std::string > m_joint_names;
  std::string m_base_frame;
  std::string m_tool_frame;
  std::string m_sensor_frame;
  bool m_base_is_reference;
  rosdyn::ChainPtr m_chain_bt;
  rosdyn::ChainPtr m_chain_bs;
  Eigen::Vector3d grav;


  Eigen::VectorXd m_target;
  Eigen::VectorXd m_Dtarget;

  Eigen::VectorXd m_x;
  Eigen::VectorXd m_Dx;
  Eigen::VectorXd m_DDx;

  Eigen::VectorXd m_velocity_limits;
  Eigen::VectorXd m_acceleration_limits;
  Eigen::VectorXd m_effort_limits;
  Eigen::VectorXd m_upper_limits;
  Eigen::VectorXd m_lower_limits;

  Eigen::Vector6d m_Jinv;
  Eigen::Vector6d m_Jinv_init;
  Eigen::VectorXd m_damping;
  Eigen::VectorXd m_damping_init;
  Eigen::VectorXd m_k;
  Eigen::VectorXd m_k_init;
  Eigen::Vector6d m_wrench_deadband;
  Eigen::Vector6d m_wrench_0;
  Eigen::VectorXd m_DDq_deadband;

  Eigen::Vector6d m_wrench_of_tool_in_base_with_deadband;
  Eigen::Vector6d m_wrench_of_sensor_in_sensor;
  Eigen::Affine3d m_T_tool_sensor;
  Eigen::Affine3d m_T_base_tool;

  bool m_is_configured;
  bool m_target_ok;
  bool m_effort_ok;
  bool m_init_wrench;

  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_target_sub;
  std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>> m_wrench_sub;




  void setTargetCallback(const sensor_msgs::JointStateConstPtr& msg);
  void setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg);


};


}  // end namespace cnr_control

