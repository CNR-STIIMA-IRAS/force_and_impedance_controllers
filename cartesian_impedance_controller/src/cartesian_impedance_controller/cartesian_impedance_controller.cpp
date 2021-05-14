#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_control::CartImpedanceController, controller_interface::ControllerBase)



namespace cnr_control
{


bool CartImpedanceController::init(hardware_interface::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;

  m_controller_nh.setCallbackQueue(&m_queue);


  if (!m_controller_nh.getParam("base_frame",m_base_frame))
  {
    ROS_ERROR("%s/base_frame not defined", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("tool_frame",m_tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("sensor_frame",m_sensor_frame))
  {
    ROS_ERROR("%s/sensor_frame not defined", m_controller_nh.getNamespace().c_str());
    return false;
  }

  if (!m_controller_nh.getParam("base_is_reference", m_base_is_reference))
  {
    ROS_INFO("Using a base reference Cartesian impedance as default");
    m_base_is_reference=true;
  }

  if (m_base_is_reference)
    ROS_INFO("Using Cartesian impedance on base %s", m_base_frame.c_str());

  urdf::Model urdf_model;
  if (!urdf_model.initParam("/robot_description"))
  {
    ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
    return false;
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  m_chain_bt = rosdyn::createChain(urdf_model,m_base_frame,m_tool_frame,gravity);
  m_chain_bs = rosdyn::createChain(urdf_model,m_base_frame,m_sensor_frame,gravity);

  try
  {
    std::string joint_target = "joint_target_topic";
    std::string external_wrench = "external_wrench";
    std::string scaling_in_topic;
    std::string scaling_out_topic;
    if (!m_controller_nh.getParam("joint_target_topic", joint_target))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/joint_target_topic does not exist. Default value 'joint_target_topic' superimposed");
      joint_target = "joint_target_topic";
    }

    if (!m_controller_nh.getParam("external_wrench_topic", external_wrench ))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/external_wrench does not exist. Default value 'external_wrench' superimposed");
      external_wrench = "external_wrench";
    }


    bool zeroing_sensor_at_startup;
    if (!m_controller_nh.getParam("zeroing_sensor_at_startup", zeroing_sensor_at_startup))
    {
      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/'zeroing_sensor_at_startup' does not exist. Default value true.");
    }
    if(zeroing_sensor_at_startup)
        m_init_wrench = true;
    else
        m_init_wrench = false;



    if (!controller_nh.getParam("controlled_joint",m_joint_names))
    {
      ROS_INFO("/controlled_joint not specified, using all");
      m_joint_names=m_hw->getNames();
    }
    m_nAx=m_joint_names.size();
    // note m_joint_names can be in different order with to the chain joints.
    m_chain_bs->setInputJointsName(m_joint_names);
    m_chain_bt->setInputJointsName(m_joint_names);

    m_joint_handles.resize(m_nAx);
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
      m_joint_handles.at(iAx)=m_hw->getHandle(m_joint_names.at(iAx));

    m_chain_bt->setInputJointsName(m_joint_names);
    m_chain_bs->setInputJointsName(m_joint_names);
    m_DDq_deadband.resize(m_nAx);
    m_target.resize(m_nAx);
    m_Dtarget.resize(m_nAx);
    m_x.resize(m_nAx);
    m_Dx.resize(m_nAx);
    m_DDx.resize(m_nAx);
    m_velocity_limits.resize(m_nAx);
    m_effort_limits.resize(m_nAx);
    m_wrench_of_tool_in_base_with_deadband.resize(6);
    m_wrench_of_sensor_in_sensor.resize(6);

    m_Jinv.resize(6);
    m_Jinv_init.resize(6);
    m_damping.resize(6);
    m_damping_init.resize(6);
    m_k.resize(6);
    m_k_init.resize(6);

    m_target.setZero();
    m_Dtarget.setZero();
    m_x.setZero();
    m_Dx.setZero();
    m_DDx.setZero();
    m_wrench_of_tool_in_base_with_deadband.setZero();
    m_wrench_of_sensor_in_sensor.setZero();

    m_velocity_limits.resize(m_nAx);
    m_acceleration_limits.resize(m_nAx);
    m_upper_limits.resize(m_nAx);
    m_lower_limits.resize(m_nAx);



    for (unsigned int iAx=0; iAx<m_nAx; iAx++)
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


    std::vector<double> inertia, damping, stiffness, wrench_deadband;
    if (!m_controller_nh.getParam("inertia", inertia))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/inertia does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (inertia.size()!=6)
    {
      ROS_ERROR("inertia should be have six values");
      return false;
    }

    if (!m_controller_nh.getParam("stiffness", stiffness))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/stiffness does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (stiffness.size()!=6)
    {
      ROS_ERROR("%s/stiffness should be have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (m_controller_nh.hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      if (!m_controller_nh.getParam("damping_ratio", damping_ratio))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/damping_ratio is not a vector of doubles");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }

      if (damping_ratio.size()!=6)
      {
        ROS_ERROR("damping should be have six values");
        return false;
      }

      damping.resize(6,0);
      for (unsigned int iAx=0;iAx<6;iAx++)
      {
        if (stiffness.at(iAx)<=0)
        {
          ROS_ERROR("damping ratio can be specified only for positive stiffness values (stiffness of Joint %s is not positive)",m_joint_names.at(iAx).c_str());
          return false;
        }
        damping.at(iAx)=2*damping_ratio.at(iAx)*std::sqrt(stiffness.at(iAx)*inertia.at(iAx));
      }
    }
    else if (!m_controller_nh.getParam("damping", damping))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/damping does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (damping.size()!=6)
    {
      ROS_ERROR("%s/damping should be have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("wrench_deadband", wrench_deadband))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/wrench_deadband does not exist, set to zero");
      wrench_deadband.resize(6,0);
    }

    if (wrench_deadband.size()!=6)
    {
      ROS_ERROR("%s/wrench_deadband should be have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }


    for (unsigned int iAx=0;iAx<6;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_ERROR("inertia value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_Jinv(iAx)=1.0/inertia.at(iAx);
        m_Jinv_init(iAx)=1.0/inertia.at(iAx);
      }

      if (damping.at(iAx)<=0)
      {
        ROS_ERROR("damping value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_damping(iAx)=damping.at(iAx);
        m_damping_init(iAx)=damping.at(iAx);
      }

      if (stiffness.at(iAx)<0)
      {
        ROS_ERROR("stiffness value of Joint %s is negative",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
        m_k_init(iAx)=stiffness.at(iAx);
      }

      if (wrench_deadband.at(iAx)<0)
      {
        ROS_INFO("wrench_deadband %d is negative, set zero",iAx);
        m_wrench_deadband(iAx)=0.0;
      }
      else
        m_wrench_deadband(iAx)=wrench_deadband.at(iAx);

    }

    ROS_DEBUG("Controller '%s' controls the following joints:",m_controller_nh.getNamespace().c_str());
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      ROS_DEBUG(" - %s",m_joint_names.at(iAx).c_str());
      ROS_DEBUG("position limits = [%f, %f]",m_lower_limits(iAx),m_upper_limits(iAx));
      ROS_DEBUG("velocity limits = [%f, %f]",-m_velocity_limits(iAx),m_velocity_limits(iAx));
      ROS_DEBUG("acceleration limits = [%f, %f]",-m_acceleration_limits(iAx),m_acceleration_limits(iAx));
    }

    m_target_sub.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh,joint_target,1));
    m_target_sub->setAdvancedCallback(boost::bind(&cnr_control::CartImpedanceController::setTargetCallback,this,_1));

    m_wrench_sub.reset(new ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>(m_controller_nh,external_wrench,1));
    m_wrench_sub->setAdvancedCallback(boost::bind(&cnr_control::CartImpedanceController::setWrenchCallback,this,_1));


    ROS_DEBUG("Subscribing to %s",joint_target.c_str());
    ROS_DEBUG("Subscribing to %s",external_wrench.c_str());
    ROS_DEBUG("Subscribing to %s",scaling_in_topic.c_str());
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    return false;
  }

  ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());

  return true;
}



void CartImpedanceController::starting(const ros::Time& time)
{
  for (unsigned int iAx=0;iAx<m_nAx;iAx++)
  {
    m_x(iAx)=m_joint_handles.at(iAx).getPosition();
    m_Dx(iAx)=m_joint_handles.at(iAx).getVelocity();
    m_joint_handles.at(iAx).setCommand(m_x(iAx),m_Dx(iAx),0.0);
  }
  m_target=m_x;
  m_Dtarget=m_Dx;

  m_queue.callAvailable();

  ROS_INFO("Controller '%s' well started",m_controller_nh.getNamespace().c_str());
  m_is_configured = (m_target_ok && m_effort_ok);
  if (m_is_configured)
    ROS_DEBUG("configured");
}

void CartImpedanceController::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s ] Stopping controller", m_controller_nh.getNamespace().c_str());
}



void CartImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{
  try
  {
    m_queue.callAvailable();
  }
  catch (std::exception& e)
  {
    ROS_ERROR_THROTTLE(1,"Something wrong in the callback: %s",e.what());
  }

  m_is_configured = (m_target_ok && m_effort_ok);

  Eigen::Affine3d T_base_targetpose          = m_chain_bt->getTransformation(m_target);
  Eigen::Vector6d cart_target_vel_of_t_in_b  = m_chain_bt->getJacobian(m_target)*m_Dtarget;

  Eigen::Affine3d T_b_t = m_chain_bt->getTransformation(m_x);
  Eigen::Matrix6Xd J_of_t_in_b  = m_chain_bt->getJacobian(m_x);
  Eigen::Vector6d cart_vel_of_t_in_b  = J_of_t_in_b*m_Dx;
  Eigen::Vector6d cart_acc_nl_of_t_in_b  = m_chain_bt->getDTwistNonLinearPartTool(m_x,m_Dx); // DJ*Dq
  Eigen::Vector6d cart_acc_of_t_in_b;
  Eigen::VectorXd cartesian_error_actual_target_in_b;
  rosdyn::getFrameDistance(T_base_targetpose, T_b_t , cartesian_error_actual_target_in_b);


  Eigen::Vector6d cart_vel;
  Eigen::Vector6d cart_acc;
  Eigen::Vector6d cart_err;
  if (m_base_is_reference)
  {
    cart_acc_of_t_in_b = m_Jinv.cwiseProduct(
                           m_k.cwiseProduct(cartesian_error_actual_target_in_b) +
                           m_damping.cwiseProduct(cart_target_vel_of_t_in_b-cart_vel_of_t_in_b) +
                           m_wrench_of_tool_in_base_with_deadband );

    cart_vel = cart_vel_of_t_in_b;
    cart_acc = cart_acc_of_t_in_b;
    cart_err = cartesian_error_actual_target_in_b;
  }
  else
  {
    Eigen::Vector6d cartesian_error_actual_target_in_t  = rosdyn::spatialRotation(cartesian_error_actual_target_in_b    ,T_b_t.linear().inverse());
    Eigen::Vector6d cart_target_vel_of_t_in_t           = rosdyn::spatialRotation(cart_target_vel_of_t_in_b             ,T_b_t.linear().inverse());
    Eigen::Vector6d cart_vel_of_t_in_t                  = rosdyn::spatialRotation(cart_vel_of_t_in_b                    ,T_b_t.linear().inverse());
    Eigen::Vector6d wrench_of_tool_in_t_with_deadband   = rosdyn::spatialRotation(m_wrench_of_tool_in_base_with_deadband,T_b_t.linear().inverse());
    Eigen::Vector6d cart_acc_of_t_in_t =  m_Jinv.cwiseProduct(
                                            m_k.cwiseProduct(cartesian_error_actual_target_in_t) +
                                            m_damping.cwiseProduct(cart_target_vel_of_t_in_t-cart_vel_of_t_in_t) +
                                            wrench_of_tool_in_t_with_deadband );

    cart_acc_of_t_in_b = rosdyn::spatialRotation(cart_acc_of_t_in_t,T_b_t.linear());

    cart_vel = cart_vel_of_t_in_t;
    cart_acc = cart_acc_of_t_in_t;
    cart_err = cartesian_error_actual_target_in_b;

  }


  //Singularities
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_t_in_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues()(svd.cols()-1)==0)
    ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
  else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
    ROS_WARN_THROTTLE(1,"SINGULARITY POINT");

  m_DDx = svd.solve(cart_acc_of_t_in_b-cart_acc_nl_of_t_in_b);

  Eigen::VectorXd saturated_acc=m_DDx;
  double ratio_acc=1;
  for (unsigned int idx=0; idx<m_nAx; idx++)
  ratio_acc=std::max(ratio_acc,std::abs(m_DDx(idx))/m_acceleration_limits(idx));
  saturated_acc/=ratio_acc;

  for (unsigned int idx=0; idx<m_nAx; idx++)
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

  for (unsigned int idx=0;idx<m_nAx;idx++)
    m_x(idx)=std::max(m_lower_limits(idx),std::min(m_upper_limits(idx),m_x(idx)));

  for (unsigned int iAx=0;iAx<m_nAx;iAx++)
  {
    m_joint_handles.at(iAx).setCommand(m_x(iAx),m_Dx(iAx),0.0);
  }
}



void CartImpedanceController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(m_joint_names,tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
    {
      ROS_ERROR("joints not found");
      m_target_ok=false;
      return;
    }
    if (!m_target_ok)
      ROS_DEBUG("First target message received");

    m_target_ok=true;
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      m_target(iAx)=tmp_msg.position.at(iAx);
      m_Dtarget(iAx)=tmp_msg.velocity.at(iAx);
    }

  }
  catch(...)
  {
    ROS_ERROR("Something wrong in target callback");
    m_target_ok=false;
  }
}

void CartImpedanceController::setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

  if (msg->header.frame_id.compare(m_sensor_frame))
  {
    ROS_WARN_THROTTLE(1,"sensor frame is %s, it should be %s",msg->header.frame_id.c_str(),m_sensor_frame.c_str());
    return;
  }

  try
  {

    if(m_init_wrench)
    {
      m_wrench_0(0)=msg->wrench.force.x;
      m_wrench_0(1)=msg->wrench.force.y;
      m_wrench_0(2)=msg->wrench.force.z;
      m_wrench_0(3)=msg->wrench.torque.x;
      m_wrench_0(4)=msg->wrench.torque.y;
      m_wrench_0(5)=msg->wrench.torque.z;

      m_init_wrench = false;
    }

    m_wrench_of_sensor_in_sensor(0) = msg->wrench.force.x  - m_wrench_0(0);
    m_wrench_of_sensor_in_sensor(1) = msg->wrench.force.y  - m_wrench_0(1);
    m_wrench_of_sensor_in_sensor(2) = msg->wrench.force.z  - m_wrench_0(2);
    m_wrench_of_sensor_in_sensor(3) = msg->wrench.torque.x - m_wrench_0(3);
    m_wrench_of_sensor_in_sensor(4) = msg->wrench.torque.y - m_wrench_0(4);
    m_wrench_of_sensor_in_sensor(5) = msg->wrench.torque.z - m_wrench_0(5);

    Eigen::Affine3d T_base_tool=m_chain_bt->getTransformation(m_x);
    Eigen::MatrixXd jacobian_of_tool_in_base = m_chain_bt->getJacobian(m_x);
    Eigen::Affine3d T_base_sensor=m_chain_bs->getTransformation(m_x);
    Eigen::Affine3d T_tool_sensor= T_base_tool.inverse()*T_base_sensor;

    Eigen::Vector6d wrench_of_tool_in_tool = rosdyn::spatialDualTranformation (m_wrench_of_sensor_in_sensor , T_tool_sensor         );
    Eigen::Vector6d wrench_of_tool_in_base = rosdyn::spatialRotation          (wrench_of_tool_in_tool     , T_base_tool.linear()  );

    for (unsigned int idx=0;idx<6;idx++)
    {
      if ( (wrench_of_tool_in_base(idx)>m_wrench_deadband(idx)))
        m_wrench_of_tool_in_base_with_deadband(idx)=wrench_of_tool_in_base(idx)-m_wrench_deadband(idx);
      else if ( (wrench_of_tool_in_base(idx)<-m_wrench_deadband(idx)))
        m_wrench_of_tool_in_base_with_deadband(idx)=wrench_of_tool_in_base(idx)+m_wrench_deadband(idx);
      else
        m_wrench_of_tool_in_base_with_deadband(idx)=0;
    }

    if (!m_effort_ok)
      ROS_INFO("First wrench message received");

    m_effort_ok=true;
  }
  catch(...)
  {
    ROS_ERROR_THROTTLE(1,"Something wrong in wrench callback");
    m_effort_ok=false;
  }
}

}  // end namespace cnr_control
