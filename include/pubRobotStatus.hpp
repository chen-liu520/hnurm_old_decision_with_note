

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "hnurm_interfaces/msg/vision_send_data.hpp"
#include "hnurm_interfaces/msg/target.hpp"
#include <hnurm_interfaces/msg/zone_end_point2_d.hpp>
#include <hnurm_interfaces/msg/special_area.hpp>
#include <hnurm_interfaces/msg/area.hpp>
#include <hnurm_interfaces/msg/type.hpp>
#include <hnurm_interfaces/msg/self_color.hpp>
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/bool.hpp>
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

#include "deque"
#include "mutex"
#include <atomic>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
// #include "hnurm_decision_interfaces/msg/robot_status.hpp"

// namespace hnurm_behavior_trees{

    /**
     * @brief The pubRobotStatus behavior is used to switch the controller
     * that will be used by the controller server. It subscribes to a topic "controller_selector"
     * to get the decision about what controller must be used. It is usually used before of
     * the FollowPath. The selected_controller output port is passed to controller_id
     * input port of the FollowPath
     */
    class pubRobotStatus : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::pubRobotStatus
         *
         * @param xml_tag_name Name for the XML tag for this node
         * @param conf  BT node configuration
         */
        pubRobotStatus(
            const std::string &xml_tag_name,
            const BT::NodeConfiguration &conf);
        ~pubRobotStatus() override;
        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing basic ports along with node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<bool>("cruise_status", "status to control entry of cruise branch"),
                BT::OutputPort<bool>("hp_status", "status to control entry of add hp branch"),
                BT::OutputPort<bool>("pursuit_status", "status to control entry of pursuit branch"),
                BT::OutputPort<bool>("do_clear", "clear costmatp")},    
                BT::OutputPort<bool>("navigate_to_pose_status", "navigate_to_pose_status"),

                BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_goal", "output target goal"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("cruise_goal", "target cruise goal"),   
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_goal", "supply goal"),     
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose_override", "goal_pose_override to replace goal_pose publish in rviz2"),               
                BT::OutputPort<std::string>("current_controller", "current_controller"),
                

                BT::OutputPort<bool>("pursuit_mode1", "switch pursuit mode"),
                BT::OutputPort<bool>("pursuit_mode2", "switch pursuit mode"),
                BT::OutputPort<bool>("is_hp_ok", "check if hp is ok"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("pursuit_goal", "target pursui goal"),
                BT::OutputPort<bool>("is_in_special_area", "is_in_special_area "),
                BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goal_poses", "goal poses to compute path through"),
                
        }

        // 现在的x,y坐标，getCenter函数计算特殊区域的中心点坐标返回类型
        struct current_global_pose
        {
            float pose_x;
            float pose_y;
        };

        // curent robot position in map frame
        current_global_pose current_pose_{0.0, 0.0};

        // goal_pose_override，或者说当前导航目标点（goal_pose_callback）
        current_global_pose goal_pose{0.0, 0.0};


        // 机器人当前的姿态信息
        struct current_rpy
        {
            float roll;
            float pitch;
            float yaw;
        };
        // 待删除，已经替换成Seria_Data.recv_data了：存储裁判系统数据（当前生命值、游戏进度、姿态）
        struct current_referee_status 
        {
            float current_hp;
            float game_progress;
            current_rpy rpy;
        };
        current_referee_status referee_status_{0.0, 0.0, {0.0f, 0.0f, 0.0f}};


        // 存储发送给其他系统的数据（目标状态、目标距离）
        struct Send_data
        {
            bool target_state;
            float target_distance;
        };
        Send_data send_data{false, 0.0};

        // 追逐类型枚举：接近目标或远离目标
        enum class PursuitType
        {
            Approach,
            Away
        };

        // 追逐类型的原子变量（线程安全），初始化为接近模式
        std::atomic<PursuitType> pursuit_type = PursuitType::Approach;

        // 目标姿态
        geometry_msgs::msg::PoseStamped target_pose_;

        // 控制权优先级枚举
        enum class ControlAuthority
        {
            Idle,       // 空闲状态
            MainCamera, // 主相机（最高优先级）
            BackCamera  // 后视相机（低优先级）
        };
        // 全局控制权状态
        std::atomic<ControlAuthority> current_authority = ControlAuthority::Idle;

        // 存储参考偏航角控制数据（参考偏航角、是否处于控制状态）
        struct referen_yaw_control
        {
            double reference_yaw_;
            bool on_control;
        };
        // 全局变量存储基准Yaw角度
        referen_yaw_control back_camera_control = {0.0, false};
        
        
        // 接收裁判系统数据的结构体
         struct Seria_Data
        {
            hnurm_interfaces::msg::VisionRecvData data;
            bool is_init = false;
        };
        // 全局变量存储接收到的裁判系统数据  
        Seria_Data recv_data;



        // 原子类型：多线程安全，比互斥锁效率高。表示控制状态
        std::atomic<bool> on_control = false; 
           
        // 裁判系统读取互斥锁，匹配referee_status_变量，好像不用了！！！
        std::mutex referee_status_mutex;
        // 裁判系统读取互斥锁，匹配recv_data变量
        std::mutex recv_data_mutex;

        // 标记是否已获取初始偏航角 （recv_callback）
        bool get_init_yaw_ = false;

        // 存储初始偏航角
        float init_yaw_;

        // 后端目标的偏置值， 调整后端摄像头或传感器的目标跟踪位置
        float back_target_bias = 0.0;

       
        
        // initialize from default.yaml,which execute in the initial_timer_callback
        // 全局初始化标志位，确保在导航开始前获取到初始位置
        bool is_global_init = false; 

        // 存储目标对象的ID（可能是敌方机器人或特定目标的标识符）
        std::string target_id_;






        // tfs start//////////////////////////////////////////////////////////////////////////////////////

        geometry_msgs::msg::TransformStamped transform_m2bf; // transform map to base_footprint，提取欧拉角用于方向控制
        geometry_msgs::msg::PoseStamped supply_goal; // 加血点
        geometry_msgs::msg::PoseStamped goal_; // 云台手发点的目标点

        bool goal_pose_updated_ = false; // goal_pose_callback()【接受到云台手发送的目标点，值为true】、update_poses()、timer_callback()
        bool pub_goal_pose_ = false; // timer_callback(),标记是否发布导航目标点
        bool goal_pose_reached_ = false; // 未使用

        double goal_pose_timeout_ = 100.0; // 似乎timeout参数没什么作用，但是先留着
        rclcpp::Time goal_pose_timeout_start_time_; // 初始化了，goal_pose_callback()，但是每检查超时

        // tfs end//////////////////////////////////////////////////////////////////////////////////////








        // special areas start////////////////////////////////////////////////////////////////////////
        std::vector<hnurm_interfaces::msg::SpecialArea> spa_s; // 未使用
        std::vector<std::string> special_areas_names_; // 读取名字，初始化哈系表

        struct Areas_with_center
        {
            hnurm_interfaces::msg::SpecialArea spa;   // 多边形数据
            current_global_pose center;               // 区域中心坐标
            std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> spa_paths_ends_; // 区域内穿越路径端点列表
        }; // areas_with_center;

        std::unordered_map<std::string, Areas_with_center> special_areas_; // 哈希表，以区域名字为键，区域信息为值
        std::string closest_area_;                                         // 存储当前离机器人最近的特殊区域名称
        bool is_in_special_area = false;                                   // 标记机器人是否在特殊区域内
        // special areas end////////////////////////////////////////////////////////////////////////









        // cmd_vel remapper
        geometry_msgs::msg::Twist vel_pub_; // 存储要发布的速度命令 （在特殊区域内修改偏航角控制）
        bool cmd_excuting_ = false;         // 标记命令是否正在执行
        int direction_selector_ = 0;        // 方向选择器，未使用
        Eigen::Vector2d target_;            // 存储目标位置的二维向量表示，未使用

        double cmd_yaw_;                    // 存储命令偏航角值，未使用
        double cmd_yaw_2_;
        struct PID                          // 用于创建偏航角PID控制器
        {
            double Kp = 0.5; // 比例系数
            double Ki = 0.0; // 积分系数
            double Kd = 0.0; // 微分系数
            double integral = 0.0;
            double prev_error = 0.0;
            rclcpp::Time prev_time;
        };
        PID yaw_pid_;                       // 偏航角PID控制器
        PID yaw_pid_inverse_;               // 反向偏航角PID控制器
        double theta_target;                // 未使用

        // align to the MPPI
        bool first_callback = true;         // 未使用
        rclcpp::Time last_time_;            // 未使用

    private:
        /**
         * @brief Function to perform some user-defined operation on tick
         */
        BT::NodeStatus tick() override;

        /**
         * @brief callback function for the controller_selector topic
         *
         * @param msg the message with the id of the controller_selector
         */

        // callback start ////////////////////////////////////////////////////////////////////
        //  接收视觉系统发送的机器人静态数据和状态信息
        void recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);

        // 处理后视相机的目标数据（当前代码已注释）
        void back_target_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // 接收视觉系统发送的目标状态信息，控制机器人行为模式切换
        void send_callback(const hnurm_interfaces::msg::VisionSendData::SharedPtr msg);

        // 接收里程计数据（当前代码为空实现）
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // 处理视觉系统检测到的目标信息，控制机器人追击行为
        void target_callback(const hnurm_interfaces::msg::Target::SharedPtr msg);

        // 接收外部目标点指令(云台手发送)，更新机器人导航目标
        void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // 接收全局定位系统发布的机器人位置信息
        void global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

        // 系统初始化函数，加载机器人配置参数
        void initializaion();

        // 定时器回调，实现周期性任务：更新并发布最近的特殊区域信息
        void timer_callback();

        // TF变换监听器回调（当前代码为空实现）
        void tf_listener_callback();

        // 关闭所有行为状态：设置黑板输出端口 pursuit_status=false 和 cruise_status=false
        void all_off();

        // 检查机器人是否到达目标点
        bool goal_checker(current_global_pose cruise_goal, current_global_pose current_pose, double goal_tolerance);
        // 更新目标点信息
        void goal_updater(current_global_pose goal);

        // 重映射速度命令，实现特殊区域内的速度控制
        void remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        // 处理全局路径规划结果
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

        // callback end /////////////////////////////////////////////////////////////////////








        // spa 特殊区域相关 start////////////////////////////////////////////////////////////////////////////////////

        // 函数：getAreas 获取特殊区域信息
        hnurm_interfaces::msg::SpecialArea getAreas(const std::string &area_name_);

        // 函数：getPathEnds 获取特殊区域的路径端点
        std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> getPathEnds(const std::string &area_name_);

        // 函数：getCenter 计算特殊区域的中心点坐标
        current_global_pose getCenter(const std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> &end_points);

        // 函数：calculate_distance 计算当前位置与目标位置之间的距离的平方
        double calculate_distance(current_global_pose current_pose, current_global_pose target_pose);

        // 函数：getClosestArea 获取离机器人最近的特殊区域名称
        std::string getClosestArea();

        // 函数：visualizeClosestArea 可视化离机器人最近的特殊区域，绘制特殊区域的路径线，发布标记到RViz可视化
        void visualizeClosestArea();

        // 函数：isinSpecialArea 判断机器人是否在特殊区域内
        bool isinSpecialArea(current_global_pose current_pose, std::string area);

        // spa 特殊区域相关 end////////////////////////////////////////////////////////////////////////////////////








        // 路径更新与目标管理 start /////////////////////////////////////////////////////////////////////////////////

        // 函数：update_poses 更新机器人的导航目标点，根据当前状态（补给、追击、巡航）更新目标点
        void update_poses(); 

        //
        rclcpp::Time path_through_end_time_; // 路径通过任务的结束时间
        double reset_duration_ = 2.0;        // 重置路径任务的超时时间（默认2秒）

        std::mutex goal_poses_mutex;         // 目标点的互斥锁，保护goal_poses_的访问
        std::vector<geometry_msgs::msg::PoseStamped> goal_poses_; // 存储多个导航目标点的列表
        bool is_executing_ = false;          // 标记是否正在执行导航任务
        std::mutex diff_yaw_mutex;           // 互斥锁，保护yaw_diff变量的访问
        double yaw_diff;                     // 机器人当前方向与路径方向的偏航角差

        // 计算两个二维向量之间的夹角
        double calculateAngle(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2);

        // 将巡航目标转换为PoseStamped消息
        geometry_msgs::msg::PoseStamped update_goal(current_global_pose cruise_goal);

        // 函数：find_Away_point 计算远离目标的安全位置
        current_global_pose find_Away_point(float yaw, float distance);

        // 路径更新与目标管理 end /////////////////////////////////////////////////////////////////////////////////
        







        // cruise 巡航 start ////////////////////////////////////////////////////////////////////////////////////
        struct current_cruise_goal
        {
            int idx;                // 当前巡航目标索引
            bool former_is_reached; // 标记前一个目标是否已到达
        };

        current_cruise_goal cruise_iterator{0, false}; // 巡航目标迭代器，初始值为{0, false}
        bool is_waiting_ = false;                      // 标记机器人是否在巡航点等待
        double wait_duration_ = 2.0;                   // 巡航点间隔等待持续时间（到达一个点后等待几秒）（默认2秒）
        rclcpp::Time wait_start_time_;                 // 等待开始时间
        double timeout_ = 10.0;                        // 巡航超时时间（默认10秒）
        rclcpp::Time timeout_start_time_;              // 超时开始时间

        // 查找离机器人当前位置最近的Buff点
        current_global_pose getClosestBuff();

        // cruise 巡航 end //////////////////////////////////////////////////////////////////////////////////////







        // subscribers
        rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_sub_; // recv_callback
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr back_target_sub_;         // back_target_callback
        rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr send_sub_; // send_callback 订阅视觉系统发布的目标状态信息

        rclcpp::Subscription<hnurm_interfaces::msg::Target>::SharedPtr target_sub_; // target_callback 订阅视觉系统检测到的具体目标信息
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;         // odom_callback 订阅里程计数据

        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr global_pose_sub_; // 订阅全局【定位】系统发布的机器人位置

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_override_sub_; // 云台手发送的目标点订阅
        rclcpp::Subscription<hnurm_interfaces::msg::SpecialArea>::SharedPtr special_areas_sub_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // 订阅速度命令
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;          // 订阅全局路径规划结果
        // publihser
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pursuit_pub_;           // 发布追击目标点（当前代码已注释）
        rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr decision_pub_;    // 发布决策结果
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;       // 发布可视化标记（发布用于RViz可视化的标记，如机器人与最近特殊区域的连接线）
        rclcpp::Publisher<hnurm_interfaces::msg::SpecialArea>::SharedPtr special_areas_pub_;  // 发布特殊区域信息
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr in_special_area_pub_;               // 发布是否在特殊区域内的状态
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_remap_pub_;           // 发布重映射后的速度命令

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;                                     // 常规回调组
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::CallbackGroup::SharedPtr callback_group_for_vision_data;                      // 视觉数据专用回调组
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_for_vision_data;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr tf_listener_timer_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // declare subscriptions
        std::string recv_topic_;
        std::string back_target_;
        std::string send_topic_;
        std::string target_topic_;
        std::string global_position_;                // 全局定位信息主题，可能是重定位
        std::string pursuit_armor_id_;
        std::string decision_send_topic_;
        std::string odom_topic_;
        float hp_threshold;                         // 生命值阈值，用于判断是否需要补给
        std::string goal_pose_override_;            // 云台手发送的目标点主题
        std::string area_pub_topic_;

        int decision_selector_;                     // 决策选择器，用于选择不同的决策策略

        // command velocity remapper
        std::string cmd_vel_remap_topic_;
        std::string plan_path_topic_;

        std::vector<current_global_pose> cruise_goals_;   // 存储巡航模式下的目标点序列。在cruise_模式下，机器人会按顺序访问这些目标点
        std::vector<current_global_pose> buff_points_;    // 存储Buff点（补给点）的坐标集合
        current_global_pose enermy_outpost_;              // 敌人前哨战点坐标
        current_global_pose attack_outpost_point_;        // 攻击前哨战点坐标

        // FSM global status有限状态机
        bool pursuit_ = false;            
        bool cruise_ = false;
        bool supply_ = false;
        bool attack_outpost_ = false;
        bool stay_ = false;

        std::mutex transform_mutex;                        // 保护TF变换数据的线程安全访问
        std::thread executor_thread_;                      // 执行器线程，用于处理回调函数

        std::mutex mtx_;                                   // 通用互斥锁，保护共享资源的访问
        std::thread wait_thread_;                          // 等待线程，用于处理目标点到达后的等待逻辑

        std::deque<float> bias_window_; // slide window
        const int WINDOW_SIZE = 10;
    };
//} // namespace hnurm_behavior_trees
