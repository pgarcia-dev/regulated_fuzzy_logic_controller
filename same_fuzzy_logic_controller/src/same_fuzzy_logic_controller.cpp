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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "same_fuzzy_logic_controller/same_fuzzy_logic_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "fl/Headers.h"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT
using namespace fl;

namespace same_fuzzy_logic_controller
{

void SameFuzzyLogicController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(node, plugin_name_, logger_, costmap_->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>( tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
}

void SameFuzzyLogicController::cleanup()
{
  RCLCPP_INFO(logger_,"Cleaning up controller: %s of type regulated_pure_pursuit_controller::SameFuzzyLogicController",plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
}

void SameFuzzyLogicController::activate()
{
  RCLCPP_INFO(logger_,"Activating controller: %s of type regulated_pure_pursuit_controller::SameFuzzyLogicController",plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
}

void SameFuzzyLogicController::deactivate()
{
  RCLCPP_INFO(logger_,"Deactivating controller: %s of type regulated_pure_pursuit_controller::SameFuzzyLogicController",plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> SameFuzzyLogicController::createCarrotMsg(const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double SameFuzzyLogicController::getLookAheadDistance(
const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = params_->lookahead_dist;
  if (params_->use_velocity_scaled_lookahead_dist) {
    lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
    lookahead_dist = std::clamp(
      lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
  }

  return lookahead_dist;
}

double calculateCurvature(geometry_msgs::msg::Point lookahead_point)
{
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

void SameFuzzyLogicController::setPlan(const nav_msgs::msg::Path & path)  //==========================================
{
  path_handler_->setPlan(path);
}

geometry_msgs::msg::TwistStamped SameFuzzyLogicController::computeVelocityCommands( //===========================================
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*speed*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
    // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(pose, 5);//=======5 aprox
  //global_path_pub_->publish(transformed_plan);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = 0.4;//0.3;//0.6; //getLookAheadDistance(speed);////////////////////////--------------------------------------------

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double dx2 = carrot_pose.pose.position.x;
  double dy2 = carrot_pose.pose.position.y;
  double angle_to_path = atan2(dy2, dx2); 
 // RCLCPP_INFO(logger_, "***2 heading: %f- next paths's position - x: %f , y: %f", angle_to_path, dx2, dy2);

  static const float INPUT_NL_MIN = -3.15;
  static const float INPUT_NL_MED = -3.0;
  static const float INPUT_NL_MAX = -2.8;
  static const float INPUT_NM_MIN = -2.8;
  static const float INPUT_NM_MED = -1.9;
  static const float INPUT_NM_MAX = -1.1;
  static const float INPUT_N_MIN = -1.1;
  static const float INPUT_N_MED = -0.9;
  static const float INPUT_N_MAX = -0.6;
  static const float INPUT_NS_MIN = -0.6;
  static const float INPUT_NS_MED = -0.5;
  static const float INPUT_NS_MAX = -0.4;
  static const float INPUT_ZN_MIN = -0.4;
  static const float INPUT_ZN_MED = -0.25;
  static const float INPUT_ZN_MAX = -0.1;
  static const float INPUT_Z_MIN = -0.1; 
  static const float INPUT_Z_MED = 0; //-------------
  static const float INPUT_Z_MAX = 0.1;
  static const float INPUT_ZP_MIN = 0.1;
  static const float INPUT_ZP_MED = 0.25;
  static const float INPUT_ZP_MAX = 0.4;
  static const float INPUT_PS_MIN = 0.4;
  static const float INPUT_PS_MED = 0.5;
  static const float INPUT_PS_MAX = 0.6;
  static const float INPUT_P_MIN = 0.6;
  static const float INPUT_P_MED = 0.9;
  static const float INPUT_P_MAX = 1.1;
  static const float INPUT_PM_MIN = 1.1;
  static const float INPUT_PM_MED = 1.9;
  static const float INPUT_PM_MAX = 2.8;
  static const float INPUT_PL_MIN = 2.8;
  static const float INPUT_PL_MED = 3.0;
  static const float INPUT_PL_MAX = 3.15;

  static const float OUTPUT_ANG_NL_MIN = -1;
  static const float OUTPUT_ANG_NL_MED = -0.8;
  static const float OUTPUT_ANG_NL_MAX = -0.6;
  static const float OUTPUT_ANG_NM_MIN = -0.6;
  static const float OUTPUT_ANG_NM_MED = -0.45;
  static const float OUTPUT_ANG_NM_MAX = -0.31;
  static const float OUTPUT_ANG_N_MIN = -0.31;
  static const float OUTPUT_ANG_N_MED = -0.25;
  static const float OUTPUT_ANG_N_MAX = -0.19;
  static const float OUTPUT_ANG_NS_MIN = -0.19;
  static const float OUTPUT_ANG_NS_MED = -0.15;
  static const float OUTPUT_ANG_NS_MAX = -0.1;
  static const float OUTPUT_ANG_ZN_MIN = -0.1;
  static const float OUTPUT_ANG_ZN_MED = -0.05;
  static const float OUTPUT_ANG_ZN_MAX = -0.025;
  static const float OUTPUT_ANG_Z_MIN = -0.025; 
  static const float OUTPUT_ANG_Z_MED = 0; //-------------
  static const float OUTPUT_ANG_Z_MAX = 0.025;
  static const float OUTPUT_ANG_ZP_MIN = 0.025;
  static const float OUTPUT_ANG_ZP_MED = 0.05;
  static const float OUTPUT_ANG_ZP_MAX = 0.1;
  static const float OUTPUT_ANG_PS_MIN = 0.1;
  static const float OUTPUT_ANG_PS_MED = 0.15;
  static const float OUTPUT_ANG_PS_MAX = 0.19;
  static const float OUTPUT_ANG_P_MIN = 0.19;
  static const float OUTPUT_ANG_P_MED = 0.25;
  static const float OUTPUT_ANG_P_MAX = 0.31;
  static const float OUTPUT_ANG_PM_MIN = 0.31;
  static const float OUTPUT_ANG_PM_MED = 0.45;
  static const float OUTPUT_ANG_PM_MAX = 0.6;
  static const float OUTPUT_ANG_PL_MIN = 0.6;
  static const float OUTPUT_ANG_PL_MED = 0.8;
  static const float OUTPUT_ANG_PL_MAX = 1;






  static const float OUTPUT_LIN_S_MIN = 0;
  static const float OUTPUT_LIN_S_MED = 0.035;//0.0025;//0.11///////////////////////////////////////
  static const float OUTPUT_LIN_S_MAX = 0.07;//0.005;/////////////////////////////////////////////0.22;


  float increment = 0.25; //TODO: as rosparam
  static const float OUTPUT_LIN_M_MIN = 0.06;
  static const float OUTPUT_LIN_M_MED = 0.095;
  static const float OUTPUT_LIN_M_MAX = 0.15;



  static const float OUTPUT_LIN_L_MIN = 0.1 + increment;
  static const float OUTPUT_LIN_L_MED = 0.15 + increment;
  static const float OUTPUT_LIN_L_MAX = 0.2 + increment;



  static const float OUTPUT_LIN_VL_MIN = 0.019 + increment; 
  static const float OUTPUT_LIN_VL_MED = 0.22 + increment; 
  static const float OUTPUT_LIN_VL_MAX = 0.26 + increment;

  Engine *engine = new Engine;
  engine->setName("SFLC");
  engine->setDescription("");

  InputVariable* Uao_gtg = new InputVariable;
  Uao_gtg->setName("Uao_gtg");
  Uao_gtg->setDescription("");
  Uao_gtg->setEnabled(true);
  Uao_gtg->setRange(-3.8,3.8); 
  Uao_gtg->setLockValueInRange(false);
  Uao_gtg->addTerm(new Triangle("NL", INPUT_NL_MIN, INPUT_NL_MED, INPUT_NL_MAX));
  Uao_gtg->addTerm(new Triangle("NM", INPUT_NM_MIN, INPUT_NM_MED, INPUT_NM_MAX));
  Uao_gtg->addTerm(new Triangle("N",  INPUT_N_MIN,  INPUT_N_MED,  INPUT_N_MAX));
  Uao_gtg->addTerm(new Triangle("NS", INPUT_NS_MIN, INPUT_NS_MED, INPUT_NS_MAX));
  Uao_gtg->addTerm(new Triangle("ZN", INPUT_ZN_MIN, INPUT_ZN_MED, INPUT_ZN_MAX));
  Uao_gtg->addTerm(new Triangle("Z",  INPUT_Z_MIN,  INPUT_Z_MED,  INPUT_Z_MAX)); //----------
  Uao_gtg->addTerm(new Triangle("ZP", INPUT_ZP_MIN, INPUT_ZP_MED, INPUT_ZP_MAX));
  Uao_gtg->addTerm(new Triangle("PS", INPUT_PS_MIN, INPUT_PS_MED, INPUT_PS_MAX));
  Uao_gtg->addTerm(new Triangle("P",  INPUT_P_MIN,  INPUT_P_MED,  INPUT_P_MAX));
  Uao_gtg->addTerm(new Triangle("PM", INPUT_PM_MIN, INPUT_PM_MED, INPUT_PM_MAX));
  Uao_gtg->addTerm(new Triangle("PL", INPUT_PL_MIN, INPUT_PL_MED, INPUT_PL_MAX));
  engine->addInputVariable(Uao_gtg);

  OutputVariable* linear_velocity = new OutputVariable;
  linear_velocity->setName("linear_velocity");
  linear_velocity->setDescription("");
  linear_velocity->setEnabled(true);
  linear_velocity->setRange(0, 0.7);
  linear_velocity->setLockValueInRange(false);
  linear_velocity->setAggregation(new Maximum);
  linear_velocity->setDefuzzifier(new LargestOfMaximum);
  linear_velocity->setDefaultValue(fl::nan);
  linear_velocity->setLockPreviousValue(false);
  linear_velocity->addTerm(new Triangle("S",  OUTPUT_LIN_S_MIN, OUTPUT_LIN_S_MED, OUTPUT_LIN_S_MAX));
  linear_velocity->addTerm(new Triangle("M",  OUTPUT_LIN_M_MIN, OUTPUT_LIN_M_MED, OUTPUT_LIN_M_MAX));
  linear_velocity->addTerm(new Triangle("L",  OUTPUT_LIN_L_MIN, OUTPUT_LIN_L_MED, OUTPUT_LIN_L_MAX));
  linear_velocity->addTerm(new Triangle("VL", OUTPUT_LIN_VL_MIN, OUTPUT_LIN_VL_MED, OUTPUT_LIN_VL_MAX));
  engine->addOutputVariable(linear_velocity);

  OutputVariable* angular_velocity = new OutputVariable;
  angular_velocity->setName("angular_velocity");
  angular_velocity->setDescription("");
  angular_velocity->setEnabled(true);
  angular_velocity->setRange(-1,1);
  angular_velocity->setLockValueInRange(false);
  angular_velocity->setAggregation(new Maximum);
  angular_velocity->setDefuzzifier(new LargestOfMaximum);
  angular_velocity->setDefaultValue(fl::nan);
  angular_velocity->setLockPreviousValue(false);
  angular_velocity->addTerm(new Triangle("NL", OUTPUT_ANG_NL_MIN, OUTPUT_ANG_NL_MED, OUTPUT_ANG_NL_MAX));
  angular_velocity->addTerm(new Triangle("NM", OUTPUT_ANG_NM_MIN, OUTPUT_ANG_NM_MED, OUTPUT_ANG_NM_MAX));
  angular_velocity->addTerm(new Triangle("N",  OUTPUT_ANG_N_MIN,  OUTPUT_ANG_N_MED,  OUTPUT_ANG_N_MAX));
  angular_velocity->addTerm(new Triangle("NS", OUTPUT_ANG_NS_MIN, OUTPUT_ANG_NS_MED, OUTPUT_ANG_NS_MAX));
  angular_velocity->addTerm(new Triangle("ZN", OUTPUT_ANG_ZN_MIN, OUTPUT_ANG_ZN_MED, OUTPUT_ANG_ZN_MAX));
  angular_velocity->addTerm(new Triangle("Z",  OUTPUT_ANG_Z_MIN,  OUTPUT_ANG_Z_MED,  OUTPUT_ANG_Z_MAX)); //----------
  angular_velocity->addTerm(new Triangle("ZP", OUTPUT_ANG_ZP_MIN, OUTPUT_ANG_ZP_MED, OUTPUT_ANG_ZP_MAX));
  angular_velocity->addTerm(new Triangle("PS", OUTPUT_ANG_PS_MIN, OUTPUT_ANG_PS_MED, OUTPUT_ANG_PS_MAX));
  angular_velocity->addTerm(new Triangle("P",  OUTPUT_ANG_P_MIN,  OUTPUT_ANG_P_MED,  OUTPUT_ANG_P_MAX));
  angular_velocity->addTerm(new Triangle("PM", OUTPUT_ANG_PM_MIN, OUTPUT_ANG_PM_MED, OUTPUT_ANG_PM_MAX));
  angular_velocity->addTerm(new Triangle("PL", OUTPUT_ANG_PL_MIN, OUTPUT_ANG_PL_MED, OUTPUT_ANG_PL_MAX));
  engine->addOutputVariable(angular_velocity);

  RuleBlock* ruleBlock = new RuleBlock;
  ruleBlock->setName("");
  ruleBlock->setDescription("");
  ruleBlock->setEnabled(true);
  //ruleBlock->setConjunction(fl::null);
  //ruleBlock->setDisjunction(fl::null);
  ruleBlock->setEnabled(true);
  ruleBlock->setConjunction(new Minimum);
  ruleBlock->setDisjunction(new Maximum);
  ruleBlock->setImplication(new Minimum);
  //  ruleBlock->setImplication(new AlgebraicProduct);
  ruleBlock->setActivation(new General);

  
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is S and angular_velocity is NL", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is S and angular_velocity is NM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is S and angular_velocity is N", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is M and angular_velocity is NS", engine));

  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZN then linear_velocity is L and angular_velocity is ZN", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is VL and angular_velocity is Z", engine));/////////////
  ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is L and angular_velocity is ZP", engine));

  ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is PS", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is S and angular_velocity is P", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is S and angular_velocity is PM", engine));
  ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is S and angular_velocity is PL", engine));

  engine->addRuleBlock(ruleBlock);

  engine->addRuleBlock(ruleBlock);
  Uao_gtg->setValue(angle_to_path);
  engine->process();

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;  
  cmd_vel.twist.linear.x = linear_velocity->getValue();
  cmd_vel.twist.angular.z = angular_velocity->getValue();
 // RCLCPP_INFO(logger_, "*** robot's position - x: %f , y: %f", pose.pose.position.x, pose.pose.position.y);
  RCLCPP_INFO(logger_, "=== input angle_to_path:%f, output linear:%f, output angular: %f  ",angle_to_path, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

 // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (/*params_->use_collision_detection &&*/ collision_checker_->isCollisionImminent(pose, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, carrot_dist))
  {
    //throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
    RCLCPP_INFO(logger_, "=== COLLISSION");
  }
  

  return cmd_vel;






/*

  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist);
  global_path_pub_->publish(transformed_plan);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (params_->allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(
      params_->curvature_lookahead_dist,
      transformed_plan);
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (params_->allow_reversing) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->desired_linear_vel;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      regulation_curvature, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * lookahead_curvature;
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    throw nav2_core::NoValidControl("SameFuzzyLogicController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
  */
}

bool SameFuzzyLogicController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return params_->use_rotate_to_heading &&
         fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool SameFuzzyLogicController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void SameFuzzyLogicController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * params_->rotate_to_heading_angular_vel;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::Point SameFuzzyLogicController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped SameFuzzyLogicController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (params_->use_interpolation && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

void SameFuzzyLogicController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel, cost_vel = linear_vel;

  // limit the linear velocity by curvature
  if (params_->use_regulated_linear_velocity_scaling) {
    curvature_vel = heuristics::curvatureConstraint(
      linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
  }

  // limit the linear velocity by proximity to obstacles
  if (params_->use_cost_regulated_linear_velocity_scaling) {
    cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
  }

  // Use the lowest of the 2 constraints, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);

  // Apply constraint to reduce speed on approach to the final goal pose
  linear_vel = heuristics::approachVelocityConstraint(
    linear_vel, path, params_->min_approach_linear_velocity,
    params_->approach_velocity_scaling_dist);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
  linear_vel = sign * linear_vel;
}

void SameFuzzyLogicController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    params_->desired_linear_vel = params_->base_desired_linear_vel;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      params_->desired_linear_vel = params_->base_desired_linear_vel * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      params_->desired_linear_vel = speed_limit;
    }
  }
}

double SameFuzzyLogicController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}
} // namespace same_fuzzy_logic_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  same_fuzzy_logic_controller::SameFuzzyLogicController,
  nav2_core::Controller)
