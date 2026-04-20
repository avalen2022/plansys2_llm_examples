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

#include <string>
#include <iostream>

#include "plan_bookstore/CheckBookPresent.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plan_bookstore
{

CheckBookPresent::CheckBookPresent(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
}

void
CheckBookPresent::halt()
{
}

BT::NodeStatus
CheckBookPresent::tick()
{
  if (!initialized_) {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    (void)config().blackboard->get("node", node);

    if (node) {
      try {
        node->declare_parameter<std::string>("displaced_book", "");
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

      node->get_parameter("displaced_book", displaced_book_);

      std::string color = displaced_book_.substr(0, displaced_book_.find('_'));
      displaced_from_ = "shelf_" + color;
    }
    initialized_ = true;
  }

  // pick_book(?r, ?b, ?l) → arg1 = book, arg2 = location
  std::string book_name;
  std::string location;
  (void)config().blackboard->get("arg1", book_name);
  (void)config().blackboard->get("arg2", location);

  // FAIL only when asked to pick the displaced book from its original shelf.
  if (book_name == displaced_book_ && location == displaced_from_) {
    std::string msg = "CheckBookPresent: " + book_name +
      " NOT FOUND at " + location + ". Book is missing.";
    config().blackboard->set("out_msg", msg);
    std::cout << "[CheckBookPresent] FAILURE: " << book_name
              << " is NOT at " << location << " (displaced book)"
              << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string msg = "CheckBookPresent: " + book_name + " found at " + location + ".";
  config().blackboard->set("out_msg", msg);
  std::cout << "[CheckBookPresent] OK: " << book_name
            << " found at " << location << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace plan_bookstore

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan_bookstore::CheckBookPresent>("CheckBookPresent");
}
