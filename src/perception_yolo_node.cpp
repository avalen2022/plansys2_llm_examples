// Copyright 2026 Intelligent Robotics Lab
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

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

class PerceptionYoloNode : public rclcpp::Node
{
public:
  PerceptionYoloNode()
  : Node("perception_yolo_node"),
    last_detection_time_(this->now())
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/perception_events", 10);

    declare_parameter<double>("detection_cooldown", 2.0);
    declare_parameter<double>("min_confidence", 0.4);
    declare_parameter<std::vector<std::string>>("detect_classes",
      std::vector<std::string>{"book"});

    cooldown_sec_ = get_parameter("detection_cooldown").as_double();
    min_confidence_ = get_parameter("min_confidence").as_double();
    detect_classes_ = get_parameter("detect_classes").as_string_array();

    load_waypoints();

    detection_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
      "/yolo/detections", 10,
      std::bind(&PerceptionYoloNode::on_detections, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        has_odom_ = true;
      });

    RCLCPP_INFO(get_logger(),
      "Perception YOLO bridge ready. Watching for classes: [%s], "
      "cooldown: %.1fs, min_confidence: %.2f",
      fmt_classes().c_str(), cooldown_sec_, min_confidence_);
  }

private:
  struct Pose2D { double x; double y; };

  void load_waypoints()
  {
    try {
      declare_parameter<std::vector<std::string>>("waypoints");
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

    std::vector<std::string> wp_names;
    get_parameter_or("waypoints", wp_names, std::vector<std::string>{});

    for (const auto & wp : wp_names) {
      try {
        declare_parameter<std::vector<double>>("waypoint_coords." + wp);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

      std::vector<double> coords;
      if (get_parameter_or("waypoint_coords." + wp, coords, {})) {
        if (coords.size() >= 2) {
          waypoints_[wp] = Pose2D{coords[0], coords[1]};
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints for location mapping",
      waypoints_.size());
  }

  std::string find_nearest_waypoint(double x, double y) const
  {
    std::string closest;
    double min_dist = std::numeric_limits<double>::max();

    for (const auto & [name, pose] : waypoints_) {
      double dx = x - pose.x;
      double dy = y - pose.y;
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        closest = name;
      }
    }
    return closest.empty() ? "unknown" : closest;
  }

  bool is_target_class(const std::string & class_name) const
  {
    for (const auto & c : detect_classes_) {
      if (class_name == c) {return true;}
    }
    return false;
  }

  void on_detections(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    if (!has_odom_) {return;}

    auto now = this->now();
    if ((now - last_detection_time_).seconds() < cooldown_sec_) {
      return;
    }

    for (const auto & det : msg->detections) {
      if (det.score < min_confidence_) {continue;}
      if (!is_target_class(det.class_name)) {continue;}

      std::string location = find_nearest_waypoint(robot_x_, robot_y_);

      nlohmann::json event;
      event["observation"] = "object_detected";
      event["object"] = det.class_name;
      event["location"] = location;
      event["coords"] = {{"x", robot_x_}, {"y", robot_y_}};
      event["detail"] = det.class_name + " detected by camera at " + location
        + " (confidence: " + std::to_string(det.score).substr(0, 4) + ")";

      std_msgs::msg::String out;
      out.data = event.dump();
      publisher_->publish(out);

      RCLCPP_INFO(get_logger(), "[PERCEPTION] %s at %s (conf: %.2f)",
        det.class_name.c_str(), location.c_str(), det.score);

      last_detection_time_ = now;
      return;
    }
  }

  std::string fmt_classes() const
  {
    std::string s;
    for (size_t i = 0; i < detect_classes_.size(); ++i) {
      if (i > 0) {s += ", ";}
      s += detect_classes_[i];
    }
    return s;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::map<std::string, Pose2D> waypoints_;
  std::vector<std::string> detect_classes_;

  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  bool has_odom_ = false;

  double cooldown_sec_;
  double min_confidence_;
  rclcpp::Time last_detection_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionYoloNode>());
  rclcpp::shutdown();
  return 0;
}
