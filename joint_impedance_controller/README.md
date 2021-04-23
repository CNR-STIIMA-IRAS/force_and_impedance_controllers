# Joint impedance controller (plugin name cnr_control/JointImpedanceController)#

Joint impedance controller 


### parameters ###
```yaml
  your_cart_impedance_controller:
    # REQUIRED:
    type                  : "cnr_control/JointImpedanceController"
    inertia               : [ 10, 10, 10, 10, 10, 10]      # in [kg*m^2], dimensions should be equal to controlled joints.
    stiffness             : [1200,800,1200, 120,120,120]   # in [Nm/(rad)], dimensions should be equal to controlled joints.
    damping               : [120,120,120, 120,120,120]     # in [Nm/(rad/s)], dimensions should be equal to controlled joints.
    joint_target_topic    : "/joint_impedance_pos_target"  # setpoint joint topic name
    base_frame            : base_link                      # name of the base link
    tool_frame            : tool0                          # name of the tool link
    sensor_frame          : ftsensor_flange                # name of the sensor link (has to be integral with tool_frame). Needed if use_wrench=true
    use_wrench            : true                           # if true, a wrench topic is subscribed, joint torques are obtained using kinematic chain. if false, a external_torque (in the configuration space) is subscribed.

    # OPTIONAL: 
    base_is_reference     : false                          # true: impedance in base_frame, false: impedance in tool_frame [DEFAULT: true]
    torque_deadband       : [0.1, 0.1, 0.1, 0.0, 0.0, 0.0] # deadband on the joint torque [DEFAULT: zero]
    external_wrench_topic : "/ati/ft/filtered_wrench"      # wrench topic name [DEFAULT: ~/external_wrench]
    external_torques_topic : "extra_torques"               # external torque  topic name [DEFAULT: ~/external_torques]
    controlled_joint      : [ jnt1, jnt2, jnt3 ]           # controlled joint names, [DEFALT: all the joint of the hardware interface]

```
