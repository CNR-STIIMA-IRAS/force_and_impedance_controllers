#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_control::CartImpedanceController, controller_interface::ControllerBase)



namespace cnr_control
{


bool CartImpedanceController::doInit()
{

  this->getControllerNh().setCallbackQueue(&m_queue);

  if (!this->getControllerNh().getParam("base_frame",m_base_frame))
  {
    CNR_ERROR(this->logger(),this->getControllerNh().getNamespace().c_str()<<"/base_frame not defined");
    CNR_RETURN_FALSE(this->logger());
  }
  if (!this->getControllerNh().getParam("tool_frame",m_tool_frame))
  {
    CNR_ERROR(this->logger(),this->getControllerNh().getNamespace().c_str()<<"/tool_frame not defined");
    CNR_RETURN_FALSE(this->logger());
  }
  if (!this->getControllerNh().getParam("sensor_frame",m_sensor_frame))
  {
    CNR_ERROR(this->logger(),this->getControllerNh().getNamespace().c_str()<<"/sensor_frame not defined");
    CNR_RETURN_FALSE(this->logger());
  }

  if (!this->getControllerNh().getParam("base_is_reference", m_base_is_reference))
  {
    ROS_INFO("Using a base reference Cartesian impedance as default");
    m_base_is_reference=true;
  }

  if (m_base_is_reference)
    ROS_INFO("Using Cartesian impedance on base %s", m_base_frame.c_str());

  urdf::Model urdf_model;
  if (!urdf_model.initParam("/robot_description"))
  {
    CNR_ERROR(this->logger(),"Urdf robot_description does not exist"<<this->getControllerNh().getNamespace()<<"/robot_description");
    CNR_RETURN_FALSE(this->logger());
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  m_chain_bt = rosdyn::createChain(urdf_model,m_base_frame,m_tool_frame,gravity);
  m_chain_bs = rosdyn::createChain(urdf_model,m_base_frame,m_sensor_frame,gravity);

  try
  {
    std::string joint_target = "joint_target_topic";
    std::string external_wrench = "external_wrench";

    if (!this->getControllerNh().getParam("joint_target_topic", joint_target))
    {
      ROS_WARN_STREAM(this->getControllerNh().getNamespace()+"/joint_target_topic does not exist. Default value 'joint_target_topic' superimposed");
      joint_target = "joint_target_topic";
    }

    if (!this->getControllerNh().getParam("external_wrench_topic", external_wrench ))
    {
      ROS_WARN_STREAM(this->getControllerNh().getNamespace()+"/external_wrench does not exist. Default value 'external_wrench' superimposed");
      external_wrench = "external_wrench";
    }

    bool zeroing_sensor_at_startup;
    if (!this->getControllerNh().getParam("zeroing_sensor_at_startup", zeroing_sensor_at_startup))
    {
      ROS_INFO_STREAM(this->getControllerNh().getNamespace()+"/'zeroing_sensor_at_startup' does not exist. Default value true.");
    }
    if(zeroing_sensor_at_startup)
        m_init_wrench = true;
    else
        m_init_wrench = false;

    m_nAx=this->jointNames().size();
    // note this->jointNames() can be in different order with to the chain joints.
    m_chain_bs->setInputJointsName(this->jointNames());
    m_chain_bt->setInputJointsName(this->jointNames());

    m_DDq_deadband.resize(m_nAx);
    m_target.resize(m_nAx);
    m_Dtarget.resize(m_nAx);
    m_x.resize(m_nAx);
    m_Dx.resize(m_nAx);
    m_DDx.resize(m_nAx);
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

    std::vector<double> inertia, damping, stiffness, wrench_deadband;
    if (!this->getControllerNh().getParam("inertia", inertia))
    {
      ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/inertia does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", this->getControllerNh().getNamespace().c_str());
      CNR_RETURN_FALSE(this->logger());
    }

    if (inertia.size()!=6)
    {
      CNR_ERROR(this->logger(),"inertia should be have six values");
      CNR_RETURN_FALSE(this->logger());
    }

    if (!this->getControllerNh().getParam("stiffness", stiffness))
    {
      ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/stiffness does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", this->getControllerNh().getNamespace().c_str());
      CNR_RETURN_FALSE(this->logger());
    }

    if (stiffness.size()!=6)
    {
      CNR_ERROR(this->logger(),this->getControllerNh().getNamespace().c_str()<<"/stiffness should be have six values");
      CNR_RETURN_FALSE(this->logger());
    }

    if (this->getControllerNh().hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      if (!this->getControllerNh().getParam("damping_ratio", damping_ratio))
      {
        ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/damping_ratio is not a vector of doubles");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", this->getControllerNh().getNamespace().c_str());
        CNR_RETURN_FALSE(this->logger());
      }

      if (damping_ratio.size()!=6)
      {
        CNR_ERROR(this->logger(),"damping should be have six values");
        CNR_RETURN_FALSE(this->logger());
      }

      damping.resize(6,0);
      for (unsigned int iAx=0;iAx<6;iAx++)
      {
        if (stiffness.at(iAx)<=0)
        {
          CNR_ERROR(this->logger(),"damping ratio can be specified only for positive stiffness values (stiffness of Joint "<<this->jointNames().at(iAx).c_str()<<" is not positive)");
          CNR_RETURN_FALSE(this->logger());
        }
        damping.at(iAx)=2*damping_ratio.at(iAx)*std::sqrt(stiffness.at(iAx)*inertia.at(iAx));
      }
    }
    else if (!this->getControllerNh().getParam("damping", damping))
    {
      ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/damping does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", this->getControllerNh().getNamespace().c_str());
      CNR_RETURN_FALSE(this->logger());
    }

    if (damping.size()!=6)
    {
      CNR_ERROR(this->logger(),this->getControllerNh().getNamespace().c_str()<<"/damping should be have six values");
      CNR_RETURN_FALSE(this->logger());
    }

    if (!this->getControllerNh().getParam("wrench_deadband", wrench_deadband))
    {
      ROS_WARN_STREAM(this->getControllerNh().getNamespace()+"/wrench_deadband does not exist, set to zero");
      wrench_deadband.resize(6,0);
    }

    if (wrench_deadband.size()!=6)
    {
      CNR_ERROR(this->logger(),this->getControllerNh().getNamespace().c_str()<<"/wrench_deadband should be have six values");
      CNR_RETURN_FALSE(this->logger());
    }


    for (unsigned int iAx=0;iAx<6;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        CNR_ERROR(this->logger(),"inertia value of Joint "<<this->jointNames().at(iAx).c_str()<<" is not positive");
        CNR_RETURN_FALSE(this->logger());
      }
      else
      {
        m_Jinv(iAx)=1.0/inertia.at(iAx);
        m_Jinv_init(iAx)=1.0/inertia.at(iAx);
      }

      if (damping.at(iAx)<=0)
      {
        CNR_ERROR(this->logger(),"damping value of Joint "<<this->jointNames().at(iAx).c_str()<<" is not positive");
        CNR_RETURN_FALSE(this->logger());
      }
      else
      {
        m_damping(iAx)=damping.at(iAx);
        m_damping_init(iAx)=damping.at(iAx);
      }

      if (stiffness.at(iAx)<0)
      {
        CNR_ERROR(this->logger(),"stiffness value of Joint "<<this->jointNames().at(iAx).c_str()<<" is negative");
        CNR_RETURN_FALSE(this->logger());
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

    m_target_sub.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(this->getControllerNh(),joint_target,1));
    m_target_sub->setAdvancedCallback(boost::bind(&cnr_control::CartImpedanceController::setTargetCallback,this,_1));

    m_wrench_sub.reset(new ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>(this->getControllerNh(),external_wrench,1));
    m_wrench_sub->setAdvancedCallback(boost::bind(&cnr_control::CartImpedanceController::setWrenchCallback,this,_1));


    ROS_DEBUG("Subscribing to %s",joint_target.c_str());
    ROS_DEBUG("Subscribing to %s",external_wrench.c_str());
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    CNR_RETURN_FALSE(this->logger());
  }

  ROS_INFO("Controller '%s' well initialized",this->getControllerNh().getNamespace().c_str());

  CNR_RETURN_TRUE(this->logger());
}



bool CartImpedanceController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  m_x  = this->getPosition();
  m_Dx = this->getVelocity();
  this->setCommandPosition(m_x);
  this->setCommandVelocity(m_Dx);

  m_target=m_x;
  m_Dtarget=m_Dx;

  m_queue.callAvailable();

  ROS_INFO("Controller '%s' well started",this->getControllerNh().getNamespace().c_str());
  m_is_configured = (m_target_ok && m_effort_ok);
  if (m_is_configured)
    ROS_DEBUG("configured");

  CNR_RETURN_TRUE(this->logger());
}

bool CartImpedanceController::doStopping(const ros::Time& time)
{
  ROS_INFO("[ %s ] Stopping controller", this->getControllerNh().getNamespace().c_str());
  CNR_TRACE_START(this->logger(),"Starting Controller");
  CNR_RETURN_TRUE(this->logger());
}



bool CartImpedanceController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    m_queue.callAvailable();
  }
  catch (std::exception& e)
  {
    CNR_ERROR_THROTTLE(this->logger(),1.0,"Something wrong in the callback: "<<e.what());
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
  m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
  m_Dx += m_DDx * period.toSec();

  this->setCommandPosition(m_x);
  this->setCommandVelocity(m_Dx);

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}



void CartImpedanceController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(this->jointNames(),tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
    {
      CNR_ERROR(this->logger(),"joints not found");
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
    CNR_ERROR(this->logger(),"Something wrong in target callback");
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
    CNR_ERROR_THROTTLE(this->logger(),1.0,"Something wrong in wrench callback");
    m_effort_ok=false;
  }
}

}  // end namespace cnr_control
