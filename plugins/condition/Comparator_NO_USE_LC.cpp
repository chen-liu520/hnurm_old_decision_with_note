#include <Comparator.hpp>
std::string equal_, lower_, greater_;

namespace hnurm_behavior_trees
{
    Comparator::Comparator(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          comp_status_(false),
          comp_type_("nothing")
    {

    } // Add a semicolon here

    BT::NodeStatus Comparator::tick()
    {
        float compare_A, compare_B;
        getInput("compare_A", compare_A);
        getInput("compare_B", compare_B);
        std::cout << "compare_A =========================== " << compare_A << std::endl;
        std::cout << "compare_B =========================== " << compare_B << std::endl;

        getInput("comparison_type", comp_type_);
        if (comp_type_ == equal_)
        {
            if (compare_A == compare_B)
            {
                std::cout << "A  is equal to B  !!! " << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        else if (comp_type_ == lower_)
        {
            if (compare_A < compare_B)
            {
                std::cout << "A  is lower than B  !!! " << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        else if (comp_type_ == greater_)
        {
            if (compare_A > compare_B)
            {
                std::cout << "A  is greater than B  !!! " << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        else
            return BT::NodeStatus::FAILURE;
    }
}
