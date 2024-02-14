# Regulated Fuzzy Logic Controller
! [Regulated Fuzzy Logic Controller navigation demo] (media/demo.gif “Regulated Fuzzy Logic Controller navigation demo”)

## overview
Autonomous robots require dependable navigation for effec- tive environmental traversal. This study presents initial findings on the utilization of fuzzy logic within a Nav2 controller, a dominant navigation system in the current ROS landscape. Our devised algorithm, employ- ing a straightforward fuzzy rule set, accurately follows the paths from trajectory planners. Seamlessly integrated with Nav2 and fulfilling all integration criteria, our approach’s simulation outcomes parallel those of controllers like the predictive MPC and Regulated Pure Pursuit.

## Regulated Fuzzy Logic Controller description
In this work, we introduce an approach for developing a controller employing a fuzzy logic algorithm to determine the linear and angular velocities necessary for guiding a robot to its specified destination within a given map. This controller will be implemented within the ROS 2 framework, utilizing *the Same Fuzzy Logic methodology*. 

## Features
- In this study, we have demonstrated the efficacy of the *Regulated Fuzzy Logic Controller* as a reliable, safe, and reasonably efficient solution for performing navigation tasks within the ROS 2 framework. While it performs capably, it is important to note that it does not outperform other state-of-the-art controllers commonly utilized in ROS 2. Consequently, further refinement and enhancement of this approach are warranted.
- We posit that our research can pave the way for an adaptive algorithm where the underlying fuzzy rules can be made accessible for user-specific customization. Fuzzy logic offers the advantage of defining intuitive rules, such as ”intensify turns during sharp curves” or ”halt when in close proximity.” This feature holds the potential to simplify controller configurations, making it more user-friendly, especially for those with limited expertise.
- The third party library [Fuzzy Lite](https://www.fuzzylite.com/) has been used in this work. 

## Configuration
Some fuzzy logic parameters can be tuned to modify the performance of the controller. 

Here an example of a fully-described XML with default parameter values:

```
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]
    use_realtime_priority: false

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
  
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    # regulated_fuzzy_logic_controller parameters
    FollowPath:
      plugin: "regulated_fuzzy_logic_controller::RegulatedFuzzyLogicController"

      INPUT_NL_MED: -3.0
      INPUT_NL_MAX: -2.8
      INPUT_NM_MIN: -2.8
      INPUT_NM_MED: -1.9
      INPUT_NM_MAX: -1.1
      INPUT_N_MIN: -1.1
      INPUT_N_MED: -0.9
      INPUT_N_MAX: -0.6
      INPUT_NS_MIN: -0.6
      INPUT_NS_MED: -0.5
      INPUT_NS_MAX: -0.4
      INPUT_ZN_MIN: -0.4
      INPUT_ZN_MED: -0.25
      INPUT_ZN_MAX: -0.1
      INPUT_Z_MIN: -0.1 
      INPUT_Z_MED: 0.0 
      INPUT_Z_MAX: 0.1
      INPUT_ZP_MIN: 0.1
      INPUT_ZP_MED: 0.25
      INPUT_ZP_MAX: 0.4
      INPUT_PS_MIN: 0.4
      INPUT_PS_MED: 0.5
      INPUT_PS_MAX: 0.6
      INPUT_P_MIN: 0.6
      INPUT_P_MED: 0.9
      INPUT_P_MAX: 1.1
      INPUT_PM_MIN: 1.1
      INPUT_PM_MED: 1.9
      INPUT_PM_MAX: 2.8
      INPUT_PL_MIN: 2.8
      INPUT_PL_MED: 3.0
      INPUT_PL_MAX: 3.15

      OUTPUT_ANG_NL_MIN: -1.82
      OUTPUT_ANG_NL_MED: -1.5
      OUTPUT_ANG_NL_MAX: -1.1
      OUTPUT_ANG_NM_MIN: -1.1
      OUTPUT_ANG_NM_MED: -0.8
      OUTPUT_ANG_NM_MAX: -0.52
      OUTPUT_ANG_N_MIN: -0.52
      OUTPUT_ANG_N_MED: -0.39
      OUTPUT_ANG_N_MAX: -0.28
      OUTPUT_ANG_NS_MIN: -0.28
      OUTPUT_ANG_NS_MED: -0.21
      OUTPUT_ANG_NS_MAX: -0.13
      OUTPUT_ANG_ZN_MIN: -0.13
      OUTPUT_ANG_ZN_MED: -0.08
      OUTPUT_ANG_ZN_MAX: -0.017
      OUTPUT_ANG_Z_MIN: -0.017 
      OUTPUT_ANG_Z_MED: 0.0 
      OUTPUT_ANG_Z_MAX: 0.017
      OUTPUT_ANG_ZP_MIN: 0.017
      OUTPUT_ANG_ZP_MED: 0.08
      OUTPUT_ANG_ZP_MAX: 0.13
      OUTPUT_ANG_PS_MIN: 0.13
      OUTPUT_ANG_PS_MED: 0.21
      OUTPUT_ANG_PS_MAX: 0.28
      OUTPUT_ANG_P_MIN: 0.28
      OUTPUT_ANG_P_MED: 0.39
      OUTPUT_ANG_P_MAX: 0.52
      OUTPUT_ANG_PM_MIN: 0.52
      OUTPUT_ANG_PM_MED: 0.8
      OUTPUT_ANG_PM_MAX: 1.1
      OUTPUT_ANG_PL_MIN: 1.10
      OUTPUT_ANG_PL_MED: 1.5
      OUTPUT_ANG_PL_MAX: 1.82

      OUTPUT_LIN_S_MIN: 0.001
      OUTPUT_LIN_S_MED: 0.035
      OUTPUT_LIN_S_MAX: 0.07
      OUTPUT_LIN_M_MIN: 0.06
      OUTPUT_LIN_M_MED: 0.095
      OUTPUT_LIN_M_MAX: 0.15
      OUTPUT_LIN_L_MIN: 0.1
      OUTPUT_LIN_L_MED: 0.15
      OUTPUT_LIN_L_MAX: 0.2
      OUTPUT_LIN_VL_MIN: 0.19
      OUTPUT_LIN_VL_MED: 0.22
      OUTPUT_LIN_VL_MAX: 0.26
```

## Compile from source
To compile the project, go to the src/ of your ROS 2 workspace and execute
```
git clone https://github.com/pgarcia-dev/regulated_fuzzy_logic_controller
```
and from your workspace path compile all package using colcon:
```
colcon build --symlink-install
```
Note that by executing this command, both the ROS 2 package and the *Fuzzy Lite* library will be compiled. 

## Execution 
Execute the controller running:
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
