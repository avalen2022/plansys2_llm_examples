#ifndef PLAN_BOOKSTORE__CHECKBOOKPRESENT_HPP_
#define PLAN_BOOKSTORE__CHECKBOOKPRESENT_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace plan_bookstore
{

// BT node that simulates checking whether a book is physically present
// at the robot's current location before attempting to pick it up.
//
// In the real system, this would query a perception sensor.
// In simulation, it checks against the "displaced_book" parameter:
// if the book being picked matches the displaced book, it returns FAILURE
// (simulating "book not found here").
class CheckBookPresent : public BT::ActionNodeBase
{
public:
  explicit CheckBookPresent(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  std::string displaced_book_;
  std::string displaced_from_;   // Original shelf where the book is NOT
  bool initialized_{false};
};

}  // namespace plan_bookstore

#endif  // PLAN_BOOKSTORE__CHECKBOOKPRESENT_HPP_
