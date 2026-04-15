#ifndef PLAN_BOOKSTORE__PLACEBOOK_HPP_
#define PLAN_BOOKSTORE__PLACEBOOK_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace plan_bookstore
{

class PlaceBook : public BT::ActionNodeBase
{
public:
  explicit PlaceBook(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
};

}  // namespace plan_bookstore

#endif  // PLAN_BOOKSTORE__PLACEBOOK_HPP_