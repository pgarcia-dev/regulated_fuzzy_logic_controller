// Copyright 2023 Intelligent Robotics Lab
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
// limitations under the License.#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class LineFollowerTester : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LineFollowerTester)
  
  LineFollowerTester()
  : Node("line_follower_tester"),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
  {
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&LineFollowerTester::path_callback, this, _1));
    timer_ = create_wall_timer(50ms, std::bind(&LineFollowerTester::control_cycle, this));
  }

private:
  void path_callback(nav_msgs::msg::Path::UniquePtr msg)
  {
    current_path_ = std::move(msg);
  }


  geometry_msgs::msg::PoseStamped get_nearest_path_point(
    const geometry_msgs::msg::Vector3 & robot_pos,
    const std::vector<geometry_msgs::msg::PoseStamped> & path)
  {
    auto result = std::min_element
    (path.begin(), path.end(),
    [robot_pos] (
      const geometry_msgs::msg::PoseStamped & p1,
      const geometry_msgs::msg::PoseStamped & p2) {
        double dist_x_1 = p1.pose.position.x - robot_pos.x;
        double dist_y_1 = p1.pose.position.y - robot_pos.y;
        double dist_1 = sqrt(dist_x_1 * dist_x_1 + dist_y_1 * dist_y_1);

        double dist_x_2 = p2.pose.position.x - robot_pos.x;
        double dist_y_2 = p2.pose.position.y - robot_pos.y;
        double dist_2 = sqrt(dist_x_2 * dist_x_2 + dist_y_2 * dist_y_2);

        return dist_1 < dist_2;
      });

    return *result;
  }

  void control_cycle()
  {
    if (current_path_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Path not received yet");
      return;
    }

    geometry_msgs::msg::TransformStamped pathframe2robot_msg;
    try {
      pathframe2robot_msg = tf_buffer_.lookupTransform(
        current_path_->header.frame_id, "base_footprint",
        tf2::TimePointZero);

    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
      return;
    }

    auto near_path_point = get_nearest_path_point(pathframe2robot_msg.transform.translation, current_path_->poses);

    double dist_x = near_path_point.pose.position.x - pathframe2robot_msg.transform.translation.x;
    double dist_y = near_path_point.pose.position.y - pathframe2robot_msg.transform.translation.y;

    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);

    RCLCPP_INFO(get_logger(), "Minimun: %lf", dist);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  
  nav_msgs::msg::Path::UniquePtr current_path_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto lft_node = LineFollowerTester::make_shared();

  rclcpp::spin(lft_node);

  rclcpp::shutdown();

  return 0;
}
