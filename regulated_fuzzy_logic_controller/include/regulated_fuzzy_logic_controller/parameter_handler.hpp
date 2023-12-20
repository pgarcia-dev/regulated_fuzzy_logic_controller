// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REGULATED_FUZZY_LOGIC_CONTROLLER__PARAMETER_HANDLER_HPP_
#define REGULATED_FUZZY_LOGIC_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace regulated_fuzzy_logic_controller
{

struct Parameters
{
  double desired_linear_vel; 
  double base_desired_linear_vel; 
  double lookahead_dist; 
  double max_lookahead_dist;
  double min_lookahead_dist;
  double lookahead_time; 
  bool use_interpolation; 

  double INPUT_NL_MIN; //********
  double INPUT_NL_MED = -3.0;
  double INPUT_NL_MAX = -2.8;
  double INPUT_NM_MIN = -2.8;
  double INPUT_NM_MED = -1.9;
  double INPUT_NM_MAX = -1.1;
  double INPUT_N_MIN = -1.1;
  double INPUT_N_MED = -0.9;
  double INPUT_N_MAX = -0.6;
  double INPUT_NS_MIN = -0.6;
  double INPUT_NS_MED = -0.5;
  double INPUT_NS_MAX = -0.4;
  double INPUT_ZN_MIN = -0.4;
  double INPUT_ZN_MED = -0.25;
  double INPUT_ZN_MAX = -0.1;
  double INPUT_Z_MIN = -0.1; 
  double INPUT_Z_MED = 0; //-------------
  double INPUT_Z_MAX = 0.1;
  double INPUT_ZP_MIN = 0.1;
  double INPUT_ZP_MED = 0.25;
  double INPUT_ZP_MAX = 0.4;
  double INPUT_PS_MIN = 0.4;
  double INPUT_PS_MED = 0.5;
  double INPUT_PS_MAX = 0.6;
  double INPUT_P_MIN = 0.6;
  double INPUT_P_MED = 0.9;
  double INPUT_P_MAX = 1.1;
  double INPUT_PM_MIN = 1.1;
  double INPUT_PM_MED = 1.9;
  double INPUT_PM_MAX = 2.8;
  double INPUT_PL_MIN = 2.8;
  double INPUT_PL_MED = 3.0;
  double INPUT_PL_MAX = 3.15;

  double OUTPUT_ANG_NL_MIN = -1.82;
  double OUTPUT_ANG_NL_MED = -1.5;
  double OUTPUT_ANG_NL_MAX = -1.1;
  double OUTPUT_ANG_NM_MIN = -1.1;
  double OUTPUT_ANG_NM_MED = -0.8;
  double OUTPUT_ANG_NM_MAX = -0.52;
  double OUTPUT_ANG_N_MIN = -0.52;
  double OUTPUT_ANG_N_MED = -0.39;
  double OUTPUT_ANG_N_MAX = -0.28;
  double OUTPUT_ANG_NS_MIN = -0.28;
  double OUTPUT_ANG_NS_MED = -0.21;
  double OUTPUT_ANG_NS_MAX = -0.13;
  double OUTPUT_ANG_ZN_MIN = -0.13;
  double OUTPUT_ANG_ZN_MED = -0.08;
  double OUTPUT_ANG_ZN_MAX = -0.017;
  double OUTPUT_ANG_Z_MIN = -0.017; 
  double OUTPUT_ANG_Z_MED = 0; //-------------
  double OUTPUT_ANG_Z_MAX = 0.017;
  double OUTPUT_ANG_ZP_MIN = 0.017;
  double OUTPUT_ANG_ZP_MED = 0.08;
  double OUTPUT_ANG_ZP_MAX = 0.13;
  double OUTPUT_ANG_PS_MIN = 0.13;
  double OUTPUT_ANG_PS_MED = 0.21;
  double OUTPUT_ANG_PS_MAX = 0.28;
  double OUTPUT_ANG_P_MIN = 0.28;
  double OUTPUT_ANG_P_MED = 0.39;
  double OUTPUT_ANG_P_MAX = 0.52;
  double OUTPUT_ANG_PM_MIN = 0.52;
  double OUTPUT_ANG_PM_MED = 0.8;
  double OUTPUT_ANG_PM_MAX = 1.1;
  double OUTPUT_ANG_PL_MIN = 1.1;
  double OUTPUT_ANG_PL_MED = 1.5;
  double OUTPUT_ANG_PL_MAX = 1.82;

  double OUTPUT_LIN_S_MIN = 0;
  double OUTPUT_LIN_S_MED = 0.035;
  double OUTPUT_LIN_S_MAX = 0.07;

  double OUTPUT_LIN_M_MIN = 0.06;
  double OUTPUT_LIN_M_MED = 0.095;
  double OUTPUT_LIN_M_MAX = 0.15;
  double OUTPUT_LIN_L_MIN = 0.1;
  double OUTPUT_LIN_L_MED = 0.15;
  double OUTPUT_LIN_L_MAX = 0.2;
  double OUTPUT_LIN_VL_MIN = 0.19;
  double OUTPUT_LIN_VL_MED = 0.22;
  double OUTPUT_LIN_VL_MAX = 0.26;

  //--- the below ones are not used but kept to avoid breaking other parts of the code
  double rotate_to_heading_angular_vel;
  bool use_velocity_scaled_lookahead_dist;
  double min_approach_linear_velocity;
  double approach_velocity_scaling_dist;
  double max_allowed_time_to_collision_up_to_carrot;
  bool use_regulated_linear_velocity_scaling;
  bool use_cost_regulated_linear_velocity_scaling;
  double cost_scaling_dist;
  double cost_scaling_gain;
  double inflation_cost_scaling_factor;
  double regulated_linear_scaling_min_radius;
  double regulated_linear_scaling_min_speed;
  bool use_fixed_curvature_lookahead;
  double curvature_lookahead_dist;
  bool use_rotate_to_heading;
  double max_angular_accel;
  double rotate_to_heading_min_angle;
  bool allow_reversing;
  double max_robot_pose_search_dist;
  bool use_collision_detection;
  double transform_tolerance;
};

/**
 * @class nav2_regulated_pure_pursuit_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::ParameterHandler
   */
  ParameterHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string & plugin_name,
    rclcpp::Logger & logger, const double costmap_size_x);

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::ParameterHandler
   */
  ~ParameterHandler() = default;

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  Parameters params_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // REGULATED_FUZZY_LOGIC_CONTROLLER__PARAMETER_HANDLER_HPP_
