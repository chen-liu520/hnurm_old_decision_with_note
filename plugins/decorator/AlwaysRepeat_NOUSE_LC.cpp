#include "AlwaysRepeat.hpp"
namespace hnurm_behavior_trees {


  AlwaysRepeat::AlwaysRepeat(
    const std::string & name,
    const BT::NodeConfiguration & conf
    ):BT::DecoratorNode(name, conf)
    {}
    
  BT::NodeStatus AlwaysRepeat::tick() {
    const BT::NodeStatus child_status = child_node_->executeTick();
    return BT::NodeStatus::RUNNING;
  }
}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hnurm_behavior_trees::AlwaysRepeat>("AlwaysRepeat");
}