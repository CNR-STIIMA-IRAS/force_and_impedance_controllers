# Cartesian impedance controller (plugin name cnr_control/CartImpedanceController)#

Cartesian impedance controller in tool or in base reference.
Angular error is angle*axis (see [this page](https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation)]).



### parameters ###
```yaml
  your_cart_impedance_controller:
    # REQUIRED:
    type                  : "cnr_control/CartImpedanceController"
    inertia               : [ 10, 10, 10, 10, 10, 10]      # in [kg, kg, kg, kg*m^2, kg*m^2, kg*m^2]
    stiffness             : [1200,800,1200, 120,120,120]   # in [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]
    damping               : [120,120,120, 120,120,120]     # in [N/(m/s), N/(m/s), N/(m/s), Nm/(rad/s), Nm/rad/s), Nm/rad/s)]
    joint_target_topic    : "/joint_impedance_pos_target"  # setpoint joint topic name
    base_frame            : base_link                      # name of the base link
    tool_frame            : tool0                          # name of the tool link
    sensor_frame          : ftsensor_flange                # name of the sensor link (has to be integral with tool_frame)

    # OPTIONAL: 
    base_is_reference     : false                          # true: impedance in base_frame, false: impedance in tool_frame [DEFAULT: true]
    wrench_deadband       : [0.1, 0.1, 0.1, 0.0, 0.0, 0.0] # deadband on the wrench topic [DEFAULT: zero]
    external_wrench_topic : "/ati/ft/filtered_wrench"      # wrench topic name [DEFAULT: ~/external_wrench]
    controlled_joint      : [ jnt1, jnt2, jnt3 ]           # controlled joint names, [DEFALT: all the joint of the hardware interface]
    damping_ratio         : [1,1,0.8, 1,1,2]               # adimensional damping ratio. NOTE: possibile alternative to speficy damping. If specified, it is used instead of the damping. In this case the stiffness should be positive.

```
