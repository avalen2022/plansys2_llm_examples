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
  // On first tick, read the displaced book from ROS parameter
  if (!initialized_) {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    (void)config().blackboard->get("node", node);

    if (node) {
      try {
        node->declare_parameter<std::string>("displaced_book", "");
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

      node->get_parameter("displaced_book", displaced_book_);

      // Derive the original shelf from the book name (e.g. red_book → shelf_red)
      // The book is NOT at this location. It's only missing here.
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

  // Only fail if the book is displaced AND we're at the original shelf
  // where it's NOT. At any other location (e.g. middle_path after replan),
  // the book is physically present.
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
