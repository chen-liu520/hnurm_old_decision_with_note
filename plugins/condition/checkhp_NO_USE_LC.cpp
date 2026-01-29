#include <string>
#include <checkhp.hpp>



namespace hnurm_behavior_trees {
    checkhp::checkhp(
        const std::string& condition_name,
        const BT::NodeConfiguration &conf)
    :BT::ConditionNode(condition_name, conf),
    hp_topic_("/left/vision_recv_data"),
    min_hp_(250.0),  //defalt 200.0
    is_hp_low(false)
    {   
        
        

        getInput("hp_topic",hp_topic_);



        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_,node_->get_node_base_interface());
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        hp_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            hp_topic_,
            rclcpp::ServicesQoS(),
            std::bind(&checkhp::check_hp_callback,this,std::placeholders::_1),
            sub_option);
    } // Add a semicolon here
        
    
    BT::NodeStatus checkhp::tick()
    {   
        getInput("hp_threshold",min_hp_);
        callback_group_executor_.spin_some();
        if(is_hp_low)
        {   
            
            RCLCPP_INFO(node_->get_logger(), "HP is low! ahead to the boold supply point!");
            return BT::NodeStatus::SUCCESS;      //返回FAILURE终止当前的流程，回到补血点,即便是时间没到一分钟也可以回
        }
        else 
        {   RCLCPP_INFO(node_->get_logger(), "HP is OK!!!");
            return BT::NodeStatus::FAILURE ;   //继续执行当前任务
        }

    }

    void checkhp::check_hp_callback(hnurm_interfaces::msg::VisionRecvData::SharedPtr hp)
    {   std::cout<<"!!!!!!!!!!!!!!!!"<<hp<<std::endl;
        if(hp->current_hp<=min_hp_){
            is_hp_low = true ;
        }
        else {
            is_hp_low = false;
        }

    }
}

 

