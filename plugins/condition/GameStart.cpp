#include <string>
#include "/home/robot/桌面/Livo2/NAVIGATION_MASTER/hnunavigation_-ros2/hnurm_decision/include/GameStart.hpp"

namespace hnurm_behavior_trees {
    GameStart::GameStart(
                            const std::string& condition_name,
                            const BT::NodeConfiguration &conf
                        ):
    BT::ConditionNode(condition_name, conf), 
    is_game_start(false), 
    referee_topic("/vision_recv_data"),

    Node("GameStart")
    {
        // 创建一个新的ROS 2节点实例
        node_ = std::make_shared<rclcpp::Node>("GameStartNode");

        // 创建互斥类型的回调组
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);

        // 将回调组添加到单线程执行器
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface()); 

        // 创建订阅选项并设置回调组
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        
        // 创建订阅者，订阅游戏状态数据主题
        referee_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            referee_topic,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&GameStart::is_game_start_callback,this,std::placeholders::_1),
            sub_option);
        
    }



    BT::NodeStatus GameStart::tick()
    {
        callback_group_executor_.spin_some();
        if(is_game_start)    
        {   
            RCLCPP_INFO(this->get_logger(), "GameStart !!!!");
            return BT::NodeStatus::SUCCESS;    
        }
        else 
        {   
            RCLCPP_INFO(this->get_logger(), "GameStart Signal didn't receive!");
            return BT::NodeStatus::FAILURE ;   
        }
        

    }
    void GameStart::is_game_start_callback(hnurm_interfaces::msg::VisionRecvData::SharedPtr gamestaus)
    {
      float gamestart = -1.0;     //串口传上来的数据为float
    //   gamestart = gamestaus->referee.GAME_PROGRESS;
    //  if(gamestart == 4)
    //     {
    //         is_game_start = true;
    //     }
    // }
    gamestart = gamestaus->game_progress;
    if(gamestart > 3.5)
        {
            is_game_start = true;
        }
    }
        
}



#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_behavior_trees::GameStart>("GameStart");
}