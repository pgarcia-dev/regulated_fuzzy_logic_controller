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

#ifndef SAME_FUZZY_LOGIC_CONTROLLER
#define SAME_FUZZY_LOGIC_CONTROLLER

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "nav2_core/controller.hpp"
#include "same_fuzzy_logic_controller/path_handler.hpp"
#include "same_fuzzy_logic_controller/collision_checker.hpp"
#include "same_fuzzy_logic_controller/parameter_handler.hpp"
#include "same_fuzzy_logic_controller/regulation_functions.hpp"

#include "fl/Headers.h"

namespace same_fuzzy_logic_controller
{

/**
 * @class same_fuzzy_logic_controller::SameFuzzyLogicController
 * @brief Regulated pure pursuit controller plugin
 */
class SameFuzzyLogicController : public nav2_core::Controller
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SameFuzzyLogicController)

  /**
   * @brief Constructor for same_fuzzy_logic_controller::SameFuzzyLogicController
   */
  SameFuzzyLogicController() = default;

  /**
   * @brief Destrructor for same_fuzzy_logic_controller::SameFuzzyLogicController
   */
  ~SameFuzzyLogicController() override = default;

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
   * Same as above computeVelocityCommands, but with debug results.
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
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, const nav_msgs::msg::Path & path,
    double & linear_vel, double & sign);

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
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("SameFuzzyLogicController")};

  Parameters * params_;
  double goal_dist_tol_;
  double control_duration_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  std::unique_ptr<same_fuzzy_logic_controller::PathHandler> path_handler_;
  std::unique_ptr<same_fuzzy_logic_controller::ParameterHandler> param_handler_;
  std::unique_ptr<same_fuzzy_logic_controller::CollisionChecker> collision_checker_;

  const float INPUT_NL_MIN = -3.15;
  const float INPUT_NL_MED = -3.0;
  const float INPUT_NL_MAX = -2.8;
  const float INPUT_NM_MIN = -2.8;
  const float INPUT_NM_MED = -1.9;
  const float INPUT_NM_MAX = -1.1;
  const float INPUT_N_MIN = -1.1;
  const float INPUT_N_MED = -0.9;
  const float INPUT_N_MAX = -0.6;
  const float INPUT_NS_MIN = -0.6;
  const float INPUT_NS_MED = -0.5;
  const float INPUT_NS_MAX = -0.4;
  const float INPUT_ZN_MIN = -0.4;
  const float INPUT_ZN_MED = -0.25;
  const float INPUT_ZN_MAX = -0.1;
  const float INPUT_Z_MIN = -0.1; 
  const float INPUT_Z_MED = 0; //-------------
  const float INPUT_Z_MAX = 0.1;
  const float INPUT_ZP_MIN = 0.1;
  const float INPUT_ZP_MED = 0.25;
  const float INPUT_ZP_MAX = 0.4;
  const float INPUT_PS_MIN = 0.4;
  const float INPUT_PS_MED = 0.5;
  const float INPUT_PS_MAX = 0.6;
  const float INPUT_P_MIN = 0.6;
  const float INPUT_P_MED = 0.9;
  const float INPUT_P_MAX = 1.1;
  const float INPUT_PM_MIN = 1.1;
  const float INPUT_PM_MED = 1.9;
  const float INPUT_PM_MAX = 2.8;
  const float INPUT_PL_MIN = 2.8;
  const float INPUT_PL_MED = 3.0;
  const float INPUT_PL_MAX = 3.15;

  const float OUTPUT_ANG_NL_MIN = -1;
  const float OUTPUT_ANG_NL_MED = -0.8;
  const float OUTPUT_ANG_NL_MAX = -0.6;
  const float OUTPUT_ANG_NM_MIN = -0.6;
  const float OUTPUT_ANG_NM_MED = -0.45;
  const float OUTPUT_ANG_NM_MAX = -0.31;
  const float OUTPUT_ANG_N_MIN = -0.31;
  const float OUTPUT_ANG_N_MED = -0.25;
  const float OUTPUT_ANG_N_MAX = -0.19;
  const float OUTPUT_ANG_NS_MIN = -0.19;
  const float OUTPUT_ANG_NS_MED = -0.15;
  const float OUTPUT_ANG_NS_MAX = -0.1;
  const float OUTPUT_ANG_ZN_MIN = -0.1;
  const float OUTPUT_ANG_ZN_MED = -0.05;
  const float OUTPUT_ANG_ZN_MAX = -0.025;
  const float OUTPUT_ANG_Z_MIN = -0.025; 
  const float OUTPUT_ANG_Z_MED = 0; //-------------
  const float OUTPUT_ANG_Z_MAX = 0.025;
  const float OUTPUT_ANG_ZP_MIN = 0.025;
  const float OUTPUT_ANG_ZP_MED = 0.05;
  const float OUTPUT_ANG_ZP_MAX = 0.1;
  const float OUTPUT_ANG_PS_MIN = 0.1;
  const float OUTPUT_ANG_PS_MED = 0.15;
  const float OUTPUT_ANG_PS_MAX = 0.19;
  const float OUTPUT_ANG_P_MIN = 0.19;
  const float OUTPUT_ANG_P_MED = 0.25;
  const float OUTPUT_ANG_P_MAX = 0.31;
  const float OUTPUT_ANG_PM_MIN = 0.31;
  const float OUTPUT_ANG_PM_MED = 0.45;
  const float OUTPUT_ANG_PM_MAX = 0.6;
  const float OUTPUT_ANG_PL_MIN = 0.6;
  const float OUTPUT_ANG_PL_MED = 0.8;
  const float OUTPUT_ANG_PL_MAX = 1;


  const float OUTPUT_LIN_S_MIN = 0;
  const float OUTPUT_LIN_S_MED = 0.035;//0.0025;//0.11///////////////////////////////////////
  const float OUTPUT_LIN_S_MAX = 0.07;//0.005;/////////////////////////////////////////////0.22;

  const float OUTPUT_LIN_M_MIN = 0.06;
  const float OUTPUT_LIN_M_MED = 0.095;
  const float OUTPUT_LIN_M_MAX = 0.15;

  static constexpr float increment = 0.25; //TODO: as rosparam
  static constexpr float OUTPUT_LIN_L_MIN = 0.1 + increment;
  static constexpr float OUTPUT_LIN_L_MED = 0.15 + increment;
  static constexpr float OUTPUT_LIN_L_MAX = 0.2 + increment;
  static constexpr float OUTPUT_LIN_VL_MIN = 0.019 + increment; 
  static constexpr float OUTPUT_LIN_VL_MED = 0.22 + increment; 
  static constexpr float OUTPUT_LIN_VL_MAX = 0.26 + increment;

  std::shared_ptr<fl::Engine> engine_;
  fl::InputVariable * Uao_gtg_;
  fl::OutputVariable * linear_velocity_;
  fl::OutputVariable * angular_velocity_;
};

}  // namespace same_fuzzy_logic_controller

#endif  // SAME_FUZZY_LOGIC_CONTROLLER
