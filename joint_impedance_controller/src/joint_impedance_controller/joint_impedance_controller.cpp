#include <joint_impedance_controller/joint_impedance_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_control::JointImpedanceController, controller_interface::ControllerBase);


namespace cnr_control
{


bool JointImpedanceController::init(hardware_interface::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

  ROS_INFO("[ %s ] init controller",  m_controller_nh.getNamespace().c_str());
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  
  m_controller_nh.setCallbackQueue(&m_queue);
  
  try
  {
    
    m_target_ok=false;
    m_effort_ok=false;

    std::string joint_target = "joint_target";
    std::string external_torques = "external_torques";
    std::string external_wrench = "external_wrench";

    if (!m_controller_nh.getParam("joint_target_topic", joint_target))
    {
      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/'joint_target' does not exist. Default value 'joint_target' superimposed.");
      joint_target = "joint_target";
    }


    if (!controller_nh.getParam("controlled_joint",m_joint_names))
    {
      ROS_DEBUG("/controlled_joint not specified, using all");
      m_joint_names=m_hw->getNames();
    }
    m_nax=m_joint_names.size();


    ROS_INFO("subscribing %s",joint_target.c_str());
    m_target_sub.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh,joint_target,1));

    if (!m_controller_nh.getParam("use_wrench", m_use_wrench))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'use_wrench' does not exist. Default value false.");
      m_use_wrench = false;
    }

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;
    std::string robot_description;
    if (!m_root_nh.getParam("/robot_description", robot_description))
    {
      ROS_FATAL_STREAM("Parameter '/robot_description' does not exist");
      return false;
    }

    urdf::Model urdf_model;
    urdf_model.initParam("robot_description");

    if (!m_controller_nh.getParam("base_frame",m_base_frame))
    {
      ROS_ERROR("base_link not defined");
      return 0;
    }
    if (!m_controller_nh.getParam("tool_frame",m_tool_frame))
    {
      ROS_ERROR("tool_link not defined");
      return 0;
    }

    m_chain_bt = rosdyn::createChain(urdf_model,m_base_frame,m_tool_frame,gravity);
    m_chain_bt->setInputJointsName(m_joint_names);

    if (m_use_wrench)
    {
      if (!m_controller_nh.getParam("sensor_frame",m_sensor_frame))
      {
        ROS_ERROR("sensor_frame not defined");
        return 0;
      }
      m_chain_bs = rosdyn::createChain(urdf_model,m_base_frame,m_sensor_frame,gravity);
      m_chain_bs->setInputJointsName(m_joint_names);
      m_wrench_sub.reset(new ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>(m_controller_nh,external_wrench,1));
      m_wrench_sub->setAdvancedCallback(boost::bind(&cnr_control::JointImpedanceController::setWrenchCallback,this,_1));
    }
    else
    {
      if (!m_controller_nh.getParam("external_torques_topic", external_torques ))
      {
        ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/'external_torques' does not exist. Default value 'external_torques' superimposed.");
        external_torques = "external_torques";
      }
      m_effort_sub.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh,external_torques,1));
      m_effort_sub->setAdvancedCallback(boost::bind(&cnr_control::JointImpedanceController::setEffortCallback,this,_1));
    }

    
    m_joint_handles.resize(m_nax);
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      m_joint_handles.at(iAx)=m_hw->getHandle(m_joint_names.at(iAx));
    }

    m_target.resize(m_nax);
    m_Dtarget.resize(m_nax);
    m_x.resize(m_nax);
    m_Dx.resize(m_nax);
    m_DDx.resize(m_nax);
    
    m_Jinv.resize(m_nax);
    m_damping.resize(m_nax);
    m_damping_dafault.resize(m_nax);
    m_k.resize(m_nax);
    m_torque_deadband.resize(m_nax);
    m_torque.resize(m_nax);
    
    m_target.setZero();
    m_Dtarget.setZero();
    m_x.setZero();
    m_Dx.setZero();
    m_DDx.setZero();
    m_torque.setZero();

    m_velocity_limits.resize(m_nax);
    m_acceleration_limits.resize(m_nax);
    m_upper_limits.resize(m_nax);
    m_lower_limits.resize(m_nax);



    for (unsigned int iAx=0; iAx<m_nax; iAx++)
    {
      m_upper_limits(iAx) = urdf_model.getJoint(m_joint_names.at(iAx))->limits->upper;
      m_lower_limits(iAx) = urdf_model.getJoint(m_joint_names.at(iAx))->limits->lower;


      if ((m_upper_limits(iAx)==0) && (m_lower_limits(iAx)==0))
      {
        m_upper_limits(iAx)=std::numeric_limits<double>::infinity();
        m_lower_limits(iAx)=-std::numeric_limits<double>::infinity();
        ROS_INFO("upper and lower limits are both equal to 0, set +/- infinity");
      }

      bool has_velocity_limits;
      if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/has_velocity_limits",has_velocity_limits))
        has_velocity_limits=false;
      bool has_acceleration_limits;
      if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/has_acceleration_limits",has_acceleration_limits))
        has_acceleration_limits=false;

      m_velocity_limits(iAx)= urdf_model.getJoint(m_joint_names.at(iAx))->limits->velocity;

      if (has_velocity_limits)
      {
        double vel;
        if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_velocity",vel))
        {
          ROS_ERROR_STREAM("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_velocity is not defined");
          return false;
        }
        if (vel<m_velocity_limits(iAx))
          m_velocity_limits(iAx)=vel;
      }

      if (has_acceleration_limits)
      {
        double acc;
        if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_acceleration",acc))
        {
          ROS_ERROR_STREAM("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_acceleration is not defined");
          return false;
        }
        m_acceleration_limits(iAx)=acc;
      }
      else
        m_acceleration_limits(iAx)=10*m_velocity_limits(iAx);
    }
    
    std::vector<double> inertia, damping, stiffness, torque_deadband;
    if (!m_controller_nh.getParam("inertia", inertia))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'inertia' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("damping", damping))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'damping' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("stiffness", stiffness))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'stiffness' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("torque_deadband", torque_deadband))
    {
      ROS_DEBUG_STREAM(m_controller_nh.getNamespace()+"/'torque_deadband' does not exist");
      ROS_DEBUG("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      torque_deadband.resize(m_nax,0);
    }
    
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_INFO("inertia value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_Jinv(iAx)=0.0;
      }
      else
        m_Jinv(iAx)=1.0/inertia.at(iAx);
      
      if (damping.at(iAx)<=0)
      {
        ROS_INFO("damping value of Joint %d is not positive, setting equalt to 10/inertia",iAx);
        m_damping(iAx)=10.0*m_Jinv(iAx);
        m_damping_dafault(iAx)=10.0*m_Jinv(iAx);
      }
      else
      {
        m_damping(iAx)=damping.at(iAx);
        m_damping_dafault(iAx)=damping.at(iAx);
      }
      
      
      if (stiffness.at(iAx)<0)
      {
        ROS_INFO("maximum fitness value of Joint %d is negative, setting equal to 0",iAx);
        m_k(iAx)=0.0;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
      }

      if (torque_deadband.at(iAx)<=0)
      {
        ROS_INFO("torque_deadband value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_torque_deadband(iAx)=0.0;
      }
      else
        m_torque_deadband(iAx)=torque_deadband.at(iAx);
    }
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    return false;
  }
  ROS_INFO("[ %s ] init OK controller",  m_controller_nh.getNamespace().c_str());

  return true;
  
}

void JointImpedanceController::starting(const ros::Time& time)
{
  ROS_INFO("[ %s ] Starting controller",  m_controller_nh.getNamespace().c_str());

  m_queue.callAvailable();
  m_is_configured=m_target_ok && m_effort_ok;
  if (m_is_configured)
  {
    ROS_DEBUG("Joint Impedance Controller Configured");
  }
  for (unsigned int iAx=0;iAx<m_nax;iAx++)
  {
    m_x(iAx)=m_joint_handles.at(iAx).getPosition();
    m_Dx(iAx)=m_joint_handles.at(iAx).getVelocity();

    ROS_DEBUG("iAx=%u, x=%f, Dx=%f",iAx,m_x(iAx),m_Dx(iAx));
  }
  ROS_INFO("[ %s ] Started controller",  m_controller_nh.getNamespace().c_str());
}

void JointImpedanceController::stopping(const ros::Time& time)
{

  for (unsigned int iAx=0;iAx<m_nax;iAx++)
  {
    m_joint_handles.at(iAx).setCommand(m_x(iAx),0.0,0.0);
  }
}

void JointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  

  if (m_is_configured)
  {
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      if ( m_torque(iAx) > m_torque_deadband(iAx) )
        m_torque(iAx)-=m_torque_deadband(iAx);
      else if ( m_torque(iAx) < -m_torque_deadband(iAx) )
        m_torque(iAx)+=m_torque_deadband(iAx);
      else
        m_torque(iAx)=0.0;
    }


    m_DDx = m_Jinv.cwiseProduct( m_k.cwiseProduct(m_target-m_x) + m_damping.cwiseProduct(m_Dtarget-m_Dx) + m_torque );
    Eigen::VectorXd saturated_acc=m_DDx;
    double ratio_acc=1;
    for (unsigned int idx=0; idx<m_nax; idx++)
      ratio_acc=std::max(ratio_acc,std::abs(m_DDx(idx))/m_acceleration_limits(idx));
    saturated_acc/=ratio_acc;

    for (unsigned int idx=0; idx<m_nax; idx++)
    {
      //Computing breaking distance
      double t_break=std::abs(m_Dx(idx))/m_acceleration_limits(idx);
      double breaking_distance=0.5*m_acceleration_limits(idx)*std::pow(t_break,2.0);

      if (m_x(idx) > (m_upper_limits(idx)-breaking_distance))
      {
        if (m_Dx(idx)>0)
        {
          ROS_WARN_THROTTLE(2,"Breaking, maximum limit approaching on joint %s",m_joint_names.at(idx).c_str());
          saturated_acc(idx)=-m_acceleration_limits(idx);
        }
      }

      if (m_x(idx) < (m_lower_limits(idx) + breaking_distance))
      {
        if (m_Dx(idx) < 0)
        {
          ROS_WARN_THROTTLE(2,"Breaking, minimum limit approaching on joint %s",m_joint_names.at(idx).c_str());
          saturated_acc(idx)=m_acceleration_limits(idx);
        }
      }
    }
    m_DDx=saturated_acc;

    m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
    m_Dx += m_DDx * period.toSec();

    for (unsigned int idx=0;idx<m_nax;idx++)
      m_x(idx)=std::max(m_lower_limits(idx),std::min(m_upper_limits(idx),m_x(idx)));


    for (unsigned int iAx=0;iAx<m_nax;iAx++)
      m_joint_handles.at(iAx).setCommand(m_x(iAx),m_Dx(iAx),0.0);
  }
  else
  {
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
      m_joint_handles.at(iAx).setCommand(m_x(iAx),0.0,0.0);
    m_is_configured=m_target_ok && m_effort_ok;
    if (m_is_configured)
    {
      ROS_DEBUG("configured");
    }
  }
  
}

void JointImpedanceController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(m_joint_names,tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
    {
      ROS_ERROR_THROTTLE(2,"joints not found");
      m_target_ok=false;
      return;
    }
    m_target_ok=true;
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      m_target(iAx)=tmp_msg.position.at(iAx);
      m_Dtarget(iAx)=tmp_msg.velocity.at(iAx);
    }
  }
  catch(...)
  {
    ROS_ERROR_THROTTLE(2,"something wrong in target callback");
    m_target_ok=false;
  }
}
void JointImpedanceController::setEffortCallback(const sensor_msgs::JointStateConstPtr& msg)
{

  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(m_joint_names,tmp_msg.name,tmp_msg.effort))
    {
      ROS_ERROR_THROTTLE(2,"joints not found");
      m_effort_ok=false;
      return;
    }
    m_effort_ok=true;
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      m_torque(iAx)=tmp_msg.effort.at(iAx);
    }
  }
  catch(...)
  {
    ROS_ERROR_THROTTLE(2,"something wrong in target callback");
    m_effort_ok=false;
  }
}

void JointImpedanceController::setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

  if (msg->header.frame_id.compare(m_sensor_frame))
  {
    ROS_ERROR_THROTTLE(2,"sensor frame is %s, it should be %s",msg->header.frame_id.c_str(),m_sensor_frame.c_str());
    return;
  }

  Eigen::Vector6d wrench_of_sensor_in_sensor;
  wrench_of_sensor_in_sensor(0)=msg->wrench.force.x;
  wrench_of_sensor_in_sensor(1)=msg->wrench.force.y;
  wrench_of_sensor_in_sensor(2)=msg->wrench.force.z;
  wrench_of_sensor_in_sensor(3)=msg->wrench.torque.x;
  wrench_of_sensor_in_sensor(4)=msg->wrench.torque.y;
  wrench_of_sensor_in_sensor(5)=msg->wrench.torque.z;

  Eigen::Affine3d T_base_tool=m_chain_bt->getTransformation(m_x);
  Eigen::MatrixXd jacobian_of_tool_in_base = m_chain_bt->getJacobian(m_x);
  Eigen::Affine3d T_base_sensor=m_chain_bs->getTransformation(m_x);
  Eigen::Affine3d T_tool_sensor= T_base_tool.inverse()*T_base_sensor;

  Eigen::Vector6d wrench_of_tool_in_tool = rosdyn::spatialDualTranformation(wrench_of_sensor_in_sensor,T_tool_sensor);
  Eigen::Vector6d wrench_of_tool_in_base = rosdyn::spatialRotation(wrench_of_tool_in_tool,T_base_tool.linear());

  m_torque=jacobian_of_tool_in_base.transpose()*wrench_of_tool_in_base;
  m_effort_ok=true;
}

}  // end namespace cnr_control
