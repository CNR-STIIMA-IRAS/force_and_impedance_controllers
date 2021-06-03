#include <joint_impedance_controller/joint_impedance_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr_control::JointImpedanceController, controller_interface::ControllerBase)


namespace cnr_control
{


bool JointImpedanceController::doInit()
{

  CNR_INFO(this->logger(),this->getControllerNh().getNamespace().c_str()<<" init controller");

  this->getControllerNh().setCallbackQueue(&m_queue);

  try
  {

    m_target_ok   = false;
    m_effort_ok   = false;

    std::string joint_target = "joint_target";
    std::string external_torques = "external_torques";
    std::string external_wrench = "external_wrench";

    if (!this->getControllerNh().getParam("joint_target_topic", joint_target))
    {
      ROS_INFO_STREAM(this->getControllerNh().getNamespace()+"/'joint_target' does not exist. Default value 'joint_target' superimposed.");
      joint_target = "joint_target";
    }

    m_nax=this->jointNames().size();


    CNR_INFO(this->logger(),"subscribing "<<joint_target.c_str());
    m_target_sub.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(this->getControllerNh(),joint_target,1));
    m_target_sub->setAdvancedCallback(boost::bind(&cnr_control::JointImpedanceController::setTargetCallback,this,_1));

    if (!this->getControllerNh().getParam("use_wrench", m_use_wrench))
    {
      ROS_WARN_STREAM(this->getControllerNh().getNamespace()+"/'use_wrench' does not exist. Default value false.");
      m_use_wrench = false;
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

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;
    std::string robot_description;
    if (!this->getRootNh().getParam("/robot_description", robot_description))
    {
      ROS_FATAL_STREAM("Parameter '/robot_description' does not exist");
      CNR_RETURN_FALSE(this->logger());
    }

    urdf::Model urdf_model;
    urdf_model.initParam("robot_description");

    if (!this->getControllerNh().getParam("base_frame",m_base_frame))
    {
      CNR_ERROR(this->logger(),"base_link not defined");
      return 0;
    }
    if (!this->getControllerNh().getParam("tool_frame",m_tool_frame))
    {
      CNR_ERROR(this->logger(),"tool_link not defined");
      return 0;
    }

    m_chain_bt = rosdyn::createChain(urdf_model,m_base_frame,m_tool_frame,gravity);
    m_chain_bt->setInputJointsName(this->jointNames());

    if (m_use_wrench)
    {
      if (!this->getControllerNh().getParam("sensor_frame",m_sensor_frame))
      {
        CNR_ERROR(this->logger(),"sensor_frame not defined");
        return 0;
      }
      m_chain_bs = rosdyn::createChain(urdf_model,m_base_frame,m_sensor_frame,gravity);
      m_chain_bs->setInputJointsName(this->jointNames());
      if (!this->getControllerNh().getParam("external_wrench_topic", external_wrench ))
      {
        ROS_INFO_STREAM(this->getControllerNh().getNamespace()+"/'external_wrench' does not exist. Default value 'external_wrench' superimposed.");
        external_wrench = "external_wrench";
      }
      m_wrench_sub.reset(new ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>(this->getControllerNh(),external_wrench,1));
      m_wrench_sub->setAdvancedCallback(boost::bind(&cnr_control::JointImpedanceController::setWrenchCallback,this,_1));
    }
    else
    {
      if (!this->getControllerNh().getParam("external_torques_topic", external_torques ))
      {
        ROS_INFO_STREAM(this->getControllerNh().getNamespace()+"/'external_torques' does not exist. Default value 'external_torques' superimposed.");
        external_torques = "external_torques";
      }
      m_effort_sub.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(this->getControllerNh(),external_torques,1));
      m_effort_sub->setAdvancedCallback(boost::bind(&cnr_control::JointImpedanceController::setEffortCallback,this,_1));
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

    std::vector<double> inertia, damping, stiffness, torque_deadband;
    if (!this->getControllerNh().getParam("inertia", inertia))
    {
      ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/'inertia' does not exist");
      CNR_FATAL(this->logger(),"ERROR DURING INITIALIZATION CONTROLLER ''"<< this->getControllerNh().getNamespace().c_str());
      CNR_RETURN_FALSE(this->logger());
    }
    if (!this->getControllerNh().getParam("stiffness", stiffness))
    {
      ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/'stiffness' does not exist");
      CNR_FATAL(this->logger(),"ERROR DURING INITIALIZATION CONTROLLER ''"<< this->getControllerNh().getNamespace().c_str());
      CNR_RETURN_FALSE(this->logger());
    }

    if(this->getControllerNh().hasParam("damping_ratio"))
    {
        CNR_INFO(this->logger(),"using damping ratio");
        std::vector<double> damping_ratio;
        if (!this->getControllerNh().getParam("damping_ratio", damping_ratio))
        {
          ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/'damping_ratio' does not exist");
          CNR_FATAL(this->logger(),"ERROR DURING INITIALIZATION CONTROLLER ''"<< this->getControllerNh().getNamespace().c_str());
          CNR_RETURN_FALSE(this->logger());
        }

        for (unsigned int iAx=0;iAx<m_nax;iAx++)
        {
          damping.push_back(2*damping_ratio.at(iAx)*sqrt(stiffness.at(iAx)*inertia.at(iAx)));
          CNR_INFO(this->logger(),damping.at(iAx));
        }
    }
    else
    {
      if (!this->getControllerNh().getParam("damping", damping))
      {
        ROS_FATAL_STREAM(this->getControllerNh().getNamespace()+"/'damping' does not exist");
        CNR_FATAL(this->logger(),"ERROR DURING INITIALIZATION CONTROLLER ''"<< this->getControllerNh().getNamespace().c_str());
        CNR_RETURN_FALSE(this->logger());
      }

    }

    if (!this->getControllerNh().getParam("torque_deadband", torque_deadband))
    {
      ROS_DEBUG_STREAM(this->getControllerNh().getNamespace()+"/'torque_deadband' does not exist");
      CNR_DEBUG(this->logger(),"ERROR DURING INITIALIZATION CONTROLLER ''"<< this->getControllerNh().getNamespace().c_str());
      torque_deadband.resize(m_nax,0);
    }

    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        CNR_INFO(this->logger(),"inertia value of Joint %d is not positive, disabling impedance control for this axis"<<iAx);
        m_Jinv(iAx)=0.0;
      }
      else
        m_Jinv(iAx)=1.0/inertia.at(iAx);

      if (damping.at(iAx)<=0)
      {
        CNR_INFO(this->logger(),"damping value of Joint %d is not positive, setting equalt to 10/inertia"<<iAx);
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
        CNR_INFO(this->logger(),"maximum fitness value of Joint %d is negative, setting equal to 0"<<iAx);
        m_k(iAx)=0.0;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
      }

      if (torque_deadband.at(iAx)<=0)
      {
        CNR_INFO(this->logger(),"torque_deadband value of Joint %d is not positive, disabling impedance control for this axis"<<iAx);
        m_torque_deadband(iAx)=0.0;
      }
      else
        m_torque_deadband(iAx)=torque_deadband.at(iAx);
    }
  }
  catch(const  std::exception& e)
  {
    CNR_FATAL(this->logger(),"EXCEPTION: "<< e.what());
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_INFO(this->logger(),this->getControllerNh().getNamespace().c_str()<<" init OK controller" );

  CNR_RETURN_TRUE(this->logger());

}

bool JointImpedanceController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  CNR_INFO(this->logger(),this->getControllerNh().getNamespace().c_str()<<" Starting controller");

  m_queue.callAvailable();
  m_is_configured=m_target_ok && m_effort_ok;
  if (m_is_configured)
  {
    CNR_DEBUG(this->logger(),"Joint Impedance Controller Configured");
  }

    m_x=this->getPosition();
    m_Dx=this->getVelocity();

//    CNR_DEBUG(this->logger(),"iAx=%u, x=%f, Dx=%f",iAx,m_x(iAx),m_Dx(iAx));
  CNR_INFO(this->logger(),this->getControllerNh().getNamespace().c_str()<<" Started controller");
  CNR_RETURN_TRUE(this->logger());
}

bool JointImpedanceController::doStopping(const ros::Time& time)
{

    this->setCommandPosition(m_x);
    CNR_TRACE_START(this->logger(),"Stopping Controller");
    CNR_RETURN_TRUE(this->logger());

}

bool JointImpedanceController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
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
    m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
    m_Dx += m_DDx * period.toSec();

    this->setCommandPosition(m_x);
    this->setCommandVelocity(m_Dx);
  }
  else
  {
    CNR_ERROR_THROTTLE(this->logger(),2.0,"target: "<<m_target_ok <<", effort: "<< m_effort_ok);
    this->setCommandPosition(m_x);
    m_is_configured=m_target_ok && m_effort_ok;
    if (m_is_configured)
    {
      CNR_DEBUG(this->logger(),"configured");
    }
  }

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

void JointImpedanceController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(this->jointNames(),tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
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
    if (!name_sorting::permutationName(this->jointNames(),tmp_msg.name,tmp_msg.effort))
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
    CNR_ERROR_THROTTLE(this->logger(),2.0,"something wrong in target callback");
    m_effort_ok=false;
  }
}

void JointImpedanceController::setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

  if (msg->header.frame_id.compare(m_sensor_frame))
  {
    CNR_ERROR_THROTTLE(this->logger(),2.0,"sensor frame is "<<msg->header.frame_id.c_str()<<", it should be "<<m_sensor_frame.c_str());
    return;
  }

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


  Eigen::Vector6d wrench_of_sensor_in_sensor;
  wrench_of_sensor_in_sensor(0)=msg->wrench.force.x  - m_wrench_0(0);
  wrench_of_sensor_in_sensor(1)=msg->wrench.force.y  - m_wrench_0(1);
  wrench_of_sensor_in_sensor(2)=msg->wrench.force.z  - m_wrench_0(2);
  wrench_of_sensor_in_sensor(3)=msg->wrench.torque.x - m_wrench_0(3);
  wrench_of_sensor_in_sensor(4)=msg->wrench.torque.y - m_wrench_0(4);
  wrench_of_sensor_in_sensor(5)=msg->wrench.torque.z - m_wrench_0(5);

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
