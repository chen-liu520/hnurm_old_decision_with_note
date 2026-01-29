
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <behaviortree_cpp_v3/condition_node.h>
#include <hnurm_interfaces/msg/vision_send_data.hpp>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>




namespace hnurm_behavior_trees {

    class GameStart : public BT::ConditionNode ,public rclcpp::Node{
    public:
    /**
     * @brief A constructor for nav2_behavior_tree::GameStart
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GameStart(
                const std::string & condition_name,  // 节点在行为树中的名字
                const BT::NodeConfiguration & conf   // BT节点配置
             );

    GameStart() = delete;    // 删除默认构造函数

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;  // 重写tick函数，用于执行节点的逻辑

    /**
     * @brief Creates list of BT ports，用于表示节点的输入输出端口
     * @return BT::PortsList Containing node-specific ports，当前返回空列表，表示该节点没有输入或输出端口
     */
    static BT::PortsList providedPorts()
    {
        return {
        };
    }

    private:
        /**
         * @brief 监听游戏状态消息的回调函数，用于处理游戏状态消息，处理 VisionRecvData 类型的消息，判断游戏是否开始
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        void is_game_start_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr game_state_); //

        // ROS 2 节点实例的共享指针
        rclcpp::Node::SharedPtr node_;
        // 回调组，用于管理回调函数的执行
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        // 单线程执行器，用于执行回调函数
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        // 订阅游戏状态数据的订阅者
        rclcpp::Subscription< hnurm_interfaces::msg::VisionRecvData>::SharedPtr referee_sub_;
        // 游戏状态数据的主题名称（默认： "/")
        std::string referee_topic;

        bool is_game_start;

    };

} // namespace hnurm_behavior_trees