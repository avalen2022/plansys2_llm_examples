#include <string>
#include <iostream>

#include "plan_bookstore/PlaceBook.hpp"

#include "behaviortree_cpp/behavior_tree.h"

namespace plan_bookstore
{

PlaceBook::PlaceBook(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
PlaceBook::halt()
{
  std::cout << "PlaceBook halt" << std::endl;
}

BT::NodeStatus
PlaceBook::tick()
{
  std::cout << "PlaceBook tick " << counter_ << std::endl;

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
  factory.registerNodeType<plan_bookstore::PlaceBook>("PlaceBook");
}