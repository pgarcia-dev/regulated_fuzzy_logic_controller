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

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".INPUT_NL_MIN", rclcpp::ParameterValue(-3.15));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", params_.desired_linear_vel);
  params_.base_desired_linear_vel = params_.desired_linear_vel;
  node->get_parameter(plugin_name_ + ".lookahead_dist", params_.lookahead_dist);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", params_.min_lookahead_dist);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", params_.max_lookahead_dist);
  node->get_parameter(plugin_name_ + ".lookahead_time", params_.lookahead_time);
  node->get_parameter(plugin_name_ + ".use_interpolation", params_.use_interpolation);

  node->get_parameter(plugin_name_ + ".INPUT_NL_MIN", params_.INPUT_NL_MIN);
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
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      continue;
    }
  }

  result.successful = true;
  return result;
}

}  // namespace regulated_fuzzy_logic_controller

