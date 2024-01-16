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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "regulated_fuzzy_logic_controller/parameter_handler.hpp"

namespace regulated_fuzzy_logic_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string & plugin_name, rclcpp::Logger & logger,
  const double /*costmap_size_x*/)
{
  plugin_name_ = plugin_name;
  logger_ = logger;

  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
/*
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
*/
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NL_MIN", rclcpp::ParameterValue(-3.15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NL_MED", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NL_MAX", rclcpp::ParameterValue(-2.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NM_MIN", rclcpp::ParameterValue(-2.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NM_MED", rclcpp::ParameterValue(-1.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NM_MAX", rclcpp::ParameterValue(-1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_N_MIN", rclcpp::ParameterValue(-1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_N_MED", rclcpp::ParameterValue(-0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_N_MAX", rclcpp::ParameterValue(-0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NS_MIN", rclcpp::ParameterValue(-0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NS_MED", rclcpp::ParameterValue(-0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_NS_MAX", rclcpp::ParameterValue(-0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_ZN_MIN", rclcpp::ParameterValue(-0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_ZN_MED", rclcpp::ParameterValue(-0.25));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_ZN_MAX", rclcpp::ParameterValue(-0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_Z_MIN", rclcpp::ParameterValue(-0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_Z_MED", rclcpp::ParameterValue(0.0)); //----------------------
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_Z_MAX", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_ZP_MIN", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_ZP_MED", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_ZP_MAX", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PS_MIN", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PS_MED", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PS_MAX", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_P_MIN", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_P_MED", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_P_MAX", rclcpp::ParameterValue(1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PM_MIN", rclcpp::ParameterValue(1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PM_MED", rclcpp::ParameterValue(1.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PM_MAX", rclcpp::ParameterValue(2.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PL_MIN", rclcpp::ParameterValue(2.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PL_MED", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".INPUT_PL_MAX", rclcpp::ParameterValue(3.15));

  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NL_MIN", rclcpp::ParameterValue(-1.82));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NL_MED", rclcpp::ParameterValue(-1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NL_MAX", rclcpp::ParameterValue(-1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NM_MIN", rclcpp::ParameterValue(-1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NM_MED", rclcpp::ParameterValue(-0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NM_MAX", rclcpp::ParameterValue(-0.52));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_N_MIN", rclcpp::ParameterValue(-0.52));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_N_MED", rclcpp::ParameterValue(-0.39));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_N_MAX", rclcpp::ParameterValue(-0.28));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NS_MIN", rclcpp::ParameterValue(-0.28));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NS_MED", rclcpp::ParameterValue(-0.21));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_NS_MAX", rclcpp::ParameterValue(-0.13));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_ZN_MIN", rclcpp::ParameterValue(-0.13));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_ZN_MED", rclcpp::ParameterValue(-0.08));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_ZN_MAX", rclcpp::ParameterValue(-0.017));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_Z_MIN", rclcpp::ParameterValue(-0.017));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_Z_MED", rclcpp::ParameterValue(0.0)); //----------------------
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_Z_MAX", rclcpp::ParameterValue(0.017));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_ZP_MIN", rclcpp::ParameterValue(0.017));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_ZP_MED", rclcpp::ParameterValue(0.08));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_ZP_MAX", rclcpp::ParameterValue(0.13));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PS_MIN", rclcpp::ParameterValue(0.13));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PS_MED", rclcpp::ParameterValue(0.21));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PS_MAX", rclcpp::ParameterValue(0.28));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_P_MIN", rclcpp::ParameterValue(0.28));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_P_MED", rclcpp::ParameterValue(0.39));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_P_MAX", rclcpp::ParameterValue(0.52));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PM_MIN", rclcpp::ParameterValue(0.52));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PM_MED", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PM_MAX", rclcpp::ParameterValue(1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PL_MIN", rclcpp::ParameterValue(1.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PL_MED", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".OUTPUT_ANG_PL_MAX", rclcpp::ParameterValue(1.82));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", params_.desired_linear_vel);
  params_.base_desired_linear_vel = params_.desired_linear_vel;
  node->get_parameter(plugin_name_ + ".lookahead_dist", params_.lookahead_dist);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", params_.min_lookahead_dist);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", params_.max_lookahead_dist);
  node->get_parameter(plugin_name_ + ".lookahead_time", params_.lookahead_time);
  node->get_parameter(plugin_name_ + ".use_interpolation", params_.use_interpolation);

  node->get_parameter(plugin_name_ + ".INPUT_NL_MIN", params_.INPUT_NL_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_NL_MED", params_.INPUT_NL_MED);
  node->get_parameter(plugin_name_ + ".INPUT_NL_MAX", params_.INPUT_NL_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_NM_MIN", params_.INPUT_NM_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_NM_MED", params_.INPUT_NM_MED);
  node->get_parameter(plugin_name_ + ".INPUT_NM_MAX", params_.INPUT_NM_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_N_MIN", params_.INPUT_N_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_N_MED", params_.INPUT_N_MED);
  node->get_parameter(plugin_name_ + ".INPUT_N_MAX", params_.INPUT_N_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_NS_MIN", params_.INPUT_NS_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_NS_MED", params_.INPUT_NS_MED);
  node->get_parameter(plugin_name_ + ".INPUT_NS_MAX", params_.INPUT_NS_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_ZN_MIN", params_.INPUT_ZN_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_ZN_MED", params_.INPUT_ZN_MED);
  node->get_parameter(plugin_name_ + ".INPUT_ZN_MAX", params_.INPUT_ZN_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_Z_MIN", params_.INPUT_Z_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_Z_MED", params_.INPUT_Z_MED);
  node->get_parameter(plugin_name_ + ".INPUT_Z_MAX", params_.INPUT_Z_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_ZP_MIN", params_.INPUT_ZP_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_ZP_MED", params_.INPUT_ZP_MED);
  node->get_parameter(plugin_name_ + ".INPUT_ZP_MAX", params_.INPUT_ZP_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_PS_MIN", params_.INPUT_PS_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_PS_MED", params_.INPUT_PS_MED);
  node->get_parameter(plugin_name_ + ".INPUT_PS_MAX", params_.INPUT_PS_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_P_MIN", params_.INPUT_P_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_P_MED", params_.INPUT_P_MED);
  node->get_parameter(plugin_name_ + ".INPUT_P_MAX", params_.INPUT_P_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_PM_MIN", params_.INPUT_PM_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_PM_MED", params_.INPUT_PM_MED);
  node->get_parameter(plugin_name_ + ".INPUT_PM_MAX", params_.INPUT_PM_MAX);
  node->get_parameter(plugin_name_ + ".INPUT_PL_MIN", params_.INPUT_PL_MIN);
  node->get_parameter(plugin_name_ + ".INPUT_PL_MED", params_.INPUT_PL_MED);
  node->get_parameter(plugin_name_ + ".INPUT_PL_MAX", params_.INPUT_PL_MAX);

  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NL_MIN", params_.OUTPUT_ANG_NL_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NL_MED", params_.OUTPUT_ANG_NL_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NL_MAX", params_.OUTPUT_ANG_NL_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NM_MIN", params_.OUTPUT_ANG_NM_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NM_MED", params_.OUTPUT_ANG_NM_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NM_MAX", params_.OUTPUT_ANG_NM_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_N_MIN", params_.OUTPUT_ANG_N_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_N_MED", params_.OUTPUT_ANG_N_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_N_MAX", params_.OUTPUT_ANG_N_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NS_MIN", params_.OUTPUT_ANG_NS_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NS_MED", params_.OUTPUT_ANG_NS_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_NS_MAX", params_.OUTPUT_ANG_NS_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_ZN_MIN", params_.OUTPUT_ANG_ZN_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_ZN_MED", params_.OUTPUT_ANG_ZN_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_ZN_MAX", params_.OUTPUT_ANG_ZN_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_Z_MIN", params_.OUTPUT_ANG_Z_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_Z_MED", params_.OUTPUT_ANG_Z_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_Z_MAX", params_.OUTPUT_ANG_Z_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_ZP_MIN", params_.OUTPUT_ANG_ZP_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_ZP_MED", params_.OUTPUT_ANG_ZP_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_ZP_MAX", params_.OUTPUT_ANG_ZP_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PS_MIN", params_.OUTPUT_ANG_PS_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PS_MED", params_.OUTPUT_ANG_PS_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PS_MAX", params_.OUTPUT_ANG_PS_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_P_MIN", params_.OUTPUT_ANG_P_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_P_MED", params_.OUTPUT_ANG_P_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_P_MAX", params_.OUTPUT_ANG_P_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PM_MIN", params_.OUTPUT_ANG_PM_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PM_MED", params_.OUTPUT_ANG_PM_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PM_MAX", params_.OUTPUT_ANG_PM_MAX);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PL_MIN", params_.OUTPUT_ANG_PL_MIN);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PL_MED", params_.OUTPUT_ANG_PL_MED);
  node->get_parameter(plugin_name_ + ".OUTPUT_ANG_PL_MAX", params_.OUTPUT_ANG_PL_MAX);
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".desired_linear_vel") {
        params_.desired_linear_vel = parameter.as_double();
        params_.base_desired_linear_vel = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        params_.lookahead_dist = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        params_.max_lookahead_dist = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        params_.lookahead_time = parameter.as_double();

      } else if (name == plugin_name_ + ".INPUT_NL_MIN") {
        params_.INPUT_NL_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NL_MED") {
        params_.INPUT_NL_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NL_MAX") {
        params_.INPUT_NL_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NM_MIN") {
        params_.INPUT_NM_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NM_MED") {
        params_.INPUT_NM_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NM_MAX") {
        params_.INPUT_NM_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_N_MIN") {
        params_.INPUT_N_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_N_MED") {
        params_.INPUT_N_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_N_MAX") {
        params_.INPUT_N_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NS_MIN") {
        params_.INPUT_NS_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NS_MED") {
        params_.INPUT_NS_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_NS_MAX") {
        params_.INPUT_NS_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_ZN_MIN") {
        params_.INPUT_ZN_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_ZN_MED") {
        params_.INPUT_ZN_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_ZN_MAX") {
        params_.INPUT_ZN_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_Z_MIN") {
        params_.INPUT_Z_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_Z_MED") { //----------------
        params_.INPUT_Z_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_Z_MAX") {
        params_.INPUT_Z_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_ZP_MIN") {
        params_.INPUT_ZP_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_ZP_MED") {
        params_.INPUT_ZP_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_ZP_MAX") {
        params_.INPUT_ZP_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PS_MIN") {
        params_.INPUT_PS_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PS_MED") {
        params_.INPUT_PS_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PS_MAX") {
        params_.INPUT_PS_MAX = parameter.as_double(); 
      } else if (name == plugin_name_ + ".INPUT_P_MIN") {
        params_.INPUT_P_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_P_MED") {
        params_.INPUT_P_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_P_MAX") {
        params_.INPUT_P_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PM_MIN") {
        params_.INPUT_PM_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PM_MED") {
        params_.INPUT_PM_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PM_MAX") {
        params_.INPUT_PM_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PL_MIN") {
        params_.INPUT_PL_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PL_MED") {
        params_.INPUT_PL_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".INPUT_PL_MAX") {
        params_.INPUT_PL_MAX = parameter.as_double();
      }
     
      else if (name == plugin_name_ + ".OUTPUT_ANG_NL_MIN") {
        params_.OUTPUT_ANG_NL_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NL_MED") {
        params_.OUTPUT_ANG_NL_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NL_MAX") {
        params_.OUTPUT_ANG_NL_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NM_MIN") {
        params_.OUTPUT_ANG_NM_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NM_MED") {
        params_.OUTPUT_ANG_NM_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NM_MAX") {
        params_.OUTPUT_ANG_NM_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_N_MIN") {
        params_.OUTPUT_ANG_N_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_N_MED") {
        params_.OUTPUT_ANG_N_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_N_MAX") {
        params_.OUTPUT_ANG_N_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NS_MIN") {
        params_.OUTPUT_ANG_NS_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NS_MED") {
        params_.OUTPUT_ANG_NS_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_NS_MAX") {
        params_.OUTPUT_ANG_NS_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_ZN_MIN") {
        params_.OUTPUT_ANG_ZN_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_ZN_MED") {
        params_.OUTPUT_ANG_ZN_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_ZN_MAX") {
        params_.OUTPUT_ANG_ZN_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_Z_MIN") {
        params_.OUTPUT_ANG_Z_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_Z_MED") { //----------------
        params_.OUTPUT_ANG_Z_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_Z_MAX") {
        params_.OUTPUT_ANG_Z_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_ZP_MIN") {
        params_.OUTPUT_ANG_ZP_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_ZP_MED") {
        params_.OUTPUT_ANG_ZP_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_ZP_MAX") {
        params_.OUTPUT_ANG_ZP_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PS_MIN") {
        params_.OUTPUT_ANG_PS_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PS_MED") {
        params_.OUTPUT_ANG_PS_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PS_MAX") {
        params_.OUTPUT_ANG_PS_MAX = parameter.as_double(); 
      } else if (name == plugin_name_ + ".OUTPUT_ANG_P_MIN") {
        params_.OUTPUT_ANG_P_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_P_MED") {
        params_.OUTPUT_ANG_P_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_P_MAX") {
        params_.OUTPUT_ANG_P_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PM_MIN") {
        params_.OUTPUT_ANG_PM_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PM_MED") {
        params_.OUTPUT_ANG_PM_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PM_MAX") {
        params_.OUTPUT_ANG_PM_MAX = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PL_MIN") {
        params_.OUTPUT_ANG_PL_MIN = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PL_MED") {
        params_.OUTPUT_ANG_PL_MED = parameter.as_double();
      } else if (name == plugin_name_ + ".OUTPUT_ANG_PL_MAX") {
        params_.OUTPUT_ANG_PL_MAX = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      continue;
    }
  }

  /*

*/

  result.successful = true;
  return result;
}

}  // namespace regulated_fuzzy_logic_controller

