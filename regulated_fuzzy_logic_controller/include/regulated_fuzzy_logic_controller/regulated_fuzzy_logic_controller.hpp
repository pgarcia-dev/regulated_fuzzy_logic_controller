// Copyright (c) 2023 Pablo García. pgarcia.developer@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the “License”);
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an “AS IS” BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REGULATED_FUZZY_LOGIC_CONTROLLER
#define REGULATED_FUZZY_LOGIC_CONTROLLER

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "fl/Headers.h"
#include "regulated_fuzzy_logic_controller/path_handler.hpp"
#include "regulated_fuzzy_logic_controller/collision_checker.hpp"
#include "regulated_fuzzy_logic_controller/parameter_handler.hpp"
#include "regulated_fuzzy_logic_controller/regulation_functions.hpp"

namespace regulated_fuzzy_logic_controller
{

/**
 * @class regulated_fuzzy_logic_controller::RegulatedFuzzyLogicController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedFuzzyLogicController : public nav2_core::Controller
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RegulatedFuzzyLogicController)

  /**
   * @brief Constructor for regulated_fuzzy_logic_controller::RegulatedFuzzyLogicController
   */
  RegulatedFuzzyLogicController() = default;

  /**
   * @brief Destrructor for regulated_fuzzy_logic_controller::RegulatedFuzzyLogicController
   */
  ~RegulatedFuzzyLogicController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /** 
   * @brief Configures the fuzzy logic controller used for regulating the robot's movement.
   * This method sets up the fuzzy logic engine and defines the input and output variables
   * along with their respective fuzzy sets and terms. It is essential for initializing the fuzzy logic controller with the appropriate parameters for the robot's navigation.
  **/
  void configure_fuzzy_controller();

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Regulated as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /** 
   * @brief Calculate the command velocity for the robot based on its current pose and a target pose.
   * @param pose The current pose of the robot.
   * @param carrot_pose The target pose towards which the robot should move. 
   * @return geometry_msgs::msg::TwistStamped The calculated command velocities for linear and angular movement.
  **/
  geometry_msgs::msg::TwistStamped calculateCmdVel(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::PoseStamped carrot_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedFuzzyLogicController")};

  Parameters * params_;
  double control_duration_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  std::unique_ptr<regulated_fuzzy_logic_controller::PathHandler> path_handler_;
  std::unique_ptr<regulated_fuzzy_logic_controller::ParameterHandler> param_handler_;

  std::shared_ptr<fl::Engine> engine_;
  fl::InputVariable * Uao_gtg_;
  fl::OutputVariable * linear_velocity_;
  fl::OutputVariable * angular_velocity_;
};

}  // namespace regulated_fuzzy_logic_controller

#endif  // REGULATED_FUZZY_LOGIC_CONTROLLER
