#ifndef PLAN_BOOKSTORE__APPROACHOBJECT_HPP_
#define PLAN_BOOKSTORE__APPROACHOBJECT_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace plan_bookstore
{

class ApproachObject : public BT::ActionNodeBase
{
public:
  explicit ApproachObject(
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

#endif  // plan_bookstore__APPROACHOBJECT_HPP_