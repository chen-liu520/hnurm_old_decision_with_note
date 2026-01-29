#include "RootControl.hpp"
namespace hnurm_behavior_trees
{
    RootControl::RootControl(const std::string &name)
        : BT::ControlNode::ControlNode(name, {})
    {
    }

    RootControl::RootControl(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::ControlNode(name, config)
    {
    }

    BT::NodeStatus RootControl::tick()
    {
        const unsigned children_count = children_nodes_.size();

        if (children_count != 3)
        {
            throw BT::BehaviorTreeException("RootControl Node '" + name() + "' must only have 3 children.");
        }
        TreeNode *child_node = children_nodes_[current_child_idx_];
        const BT::NodeStatus child_status = child_node->executeTick();
        if (current_child_idx_ == 0)
        {
            current_child_idx_ = 1;
            return child_status;
        }
        else if (current_child_idx_ == 1)
        {
            if (child_status == BT::NodeStatus::SUCCESS)
            {
                current_child_idx_ = 2;
            }
            else
            {
                current_child_idx_ = 0;
            }
        }
        else if (current_child_idx_ == 2)
        {
            if (child_status == BT::NodeStatus::SUCCESS)
            {
                current_child_idx_ = 0;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

}
