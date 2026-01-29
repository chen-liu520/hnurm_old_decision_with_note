#include <AttackOutpost.hpp>
namespace hnurm_behavior_trees
{
    using std::placeholders::_1;
    AttackOutpost::AttackOutpost(
        const std::string & name,
        const BT::NodeConfiguration &conf)
    : BT::SyncActionNode(name,conf),
    detect_status(false)
    {
        node_ =  config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_,node_->get_node_base_interface());
        auto qos = rclcpp::SystemDefaultsQoS();
        outpost_search_pub  = node_->create_publisher<hnurm_interfaces::msg::VisionSendData>("/left/vision_send_data",10);
 
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        sub_search_status  = node_->create_subscription<hnurm_interfaces::msg::Target>(
            "/left/target",
            qos,
            std::bind(&AttackOutpost::callback_search_outpost ,this,_1),
            sub_option);
        send.header.frame_id = "serial_send";
        //     send.header.stamp   时间戳获取回调函数里面的
        send.control_id      = -1.000;  
        send.target_type.data =hnurm_interfaces::msg::TargetType::NONE;
        send.target_state.data = hnurm_interfaces::msg::TargetState::CONVERGING;
        send.pitch = 20.0;
        send.yaw = -20.0;
        search_queue[0] =send;
        search_queue[1] = send;
        search_queue[1].yaw = 20.0;

        


    } 
    BT::NodeStatus AttackOutpost::tick()
    {  
        // bool outpost_attack_status;
        // getInput("outpost_attack_status",outpost_attack_status);
        callback_group_executor_.spin_some();
      return BT::NodeStatus::SUCCESS;

    }
    int counter = 0;
    bool inverter = false;
    void AttackOutpost::callback_search_outpost (const hnurm_interfaces::msg::Target msg)
    {   

        send.header.stamp = msg.header.stamp;
        if(msg.id == "outpost")
        {
            detect_status = true;
            RCLCPP_INFO(node_->get_logger(),"left detect the outpost");   //交还控制权给视觉
        }
        else 
        {   
            RCLCPP_INFO(node_->get_logger(),"left is searching the outpost");   //交还控制权给视觉
            // if(counter==30) counter=0;
            if(inverter)
            {   
                inverter = false;
                detect_status = false;
                outpost_search_pub->publish(search_queue[0]);
                // std::cout<<"!!!!!!!!"<<counter<<std::endl;
                
            }
            else{

                inverter = true;
                detect_status = false;
                outpost_search_pub->publish(search_queue[1]);
                // std::cout<<"!!!!!!!!"<<counter<<std::endl;
                
            }

        }
    }

}




#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hnurm_behavior_trees::AttackOutpost>("AttackOutpost");
}
