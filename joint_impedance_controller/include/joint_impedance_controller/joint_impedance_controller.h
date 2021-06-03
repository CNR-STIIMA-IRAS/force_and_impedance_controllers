#pragma once

//#include <controller_interface/controller.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <subscription_notifier/subscription_notifier.h>
#include <rosdyn_core/primitives.h>
#include <thread>
#include <mutex>
#include <boost/graph/graph_concepts.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/callback_queue.h>
#include <name_sorting/name_sorting.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <eigen_matrix_utils/overloads.h>

namespace cnr_control
{

//class JointImpedanceController : public controller_interface::Controller<hardware_interface::PosVelEffJointInterface>
class JointImpedanceController : public cnr::control::JointCommandController<
        hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
    bool doInit();
    bool doUpdate  (const ros::Time& time, const ros::Duration& period);
    bool doStarting(const ros::Time& time);
    bool doStopping(const ros::Time& time);

protected:

  ros::CallbackQueue m_queue;
  boost::shared_ptr<ros::AsyncSpinner> m_spinner;
  bool m_is_configured;
  bool m_target_ok;
  bool m_effort_ok;
  bool m_init_wrench;
  bool m_use_wrench;
  std::vector<hardware_interface::PosVelEffJointHandle> m_joint_handles;

  Eigen::VectorXd m_target;
  Eigen::VectorXd m_Dtarget;

  Eigen::VectorXd m_x;
  Eigen::VectorXd m_Dx;
  Eigen::VectorXd m_DDx;
  Eigen::VectorXd m_Jinv;
  Eigen::VectorXd m_damping;
  Eigen::VectorXd m_damping_dafault;
  Eigen::VectorXd m_k;
  Eigen::VectorXd m_torque_deadband;

  Eigen::VectorXd m_velocity_limits;
  Eigen::VectorXd m_acceleration_limits;
  Eigen::VectorXd m_effort_limits;
  Eigen::VectorXd m_upper_limits;
  Eigen::VectorXd m_lower_limits;

  Eigen::VectorXd m_torque;
  Eigen::VectorXd m_wrench_of_t_in_b;
  Eigen::Vector6d m_wrench_0;

  std::string m_base_frame;
  std::string m_tool_frame;
  std::string m_sensor_frame;

  rosdyn::ChainPtr m_chain_bt;
  rosdyn::ChainPtr m_chain_bs;

  hardware_interface::PosVelEffJointInterface* m_hw;

  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_target_sub;
  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_effort_sub;
  std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>> m_wrench_sub;

  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;

  std::vector< std::string > m_joint_names;
  unsigned int m_nax;


  void setTargetCallback(const sensor_msgs::JointStateConstPtr& msg);
  void setEffortCallback(const sensor_msgs::JointStateConstPtr& msg);
  void setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg);


};


}  // end namespace cnr_control

