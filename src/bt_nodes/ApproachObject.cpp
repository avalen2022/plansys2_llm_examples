#include <string>
#include <iostream>

#include "plan_bookstore/ApproachObject.hpp"

#include "behaviortree_cpp/behavior_tree.h"

namespace plan_bookstore
{

ApproachObject::ApproachObject(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
ApproachObject::halt()
{
  std::cout << "ApproachObject halt" << std::endl;
}

BT::NodeStatus
ApproachObject::tick()
{
  std::cout << "ApproachObject tick " << counter_ << std::endl;

  if (counter_++ < 5) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plan_bookstore

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan_bookstore::ApproachObject>("ApproachObject");
}