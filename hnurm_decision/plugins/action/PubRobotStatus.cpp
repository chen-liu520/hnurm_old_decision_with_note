#include "PubRobotStatus.hpp"

using namespace std::chrono_literals;

namespace hnurm_behavior_trees2{
    using std::placeholders::_1;

    PubRobotStatus::PubRobotStatus(const std::string &name, const BT::NodeConfiguration &conf) : BT::SyncActionNode(name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 创建回调组和执行器 start /////////////////////////////////////////////////////////////////////////////////////////////////////

        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        callback_group_for_vision_data = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_for_vision_data.add_callback_group(callback_group_for_vision_data, node_->get_node_base_interface());

        // 创建回调组和执行器 end /////////////////////////////////////////////////////////////////////////////////////////////////////

        // 参数声明与获取 start /////////////////////////////////////////////////////////////////////////////////////////////////////
        //************所有的默认最后都使用yaml文件里的*********************/
        recv_topic_ = node_->declare_parameter("recv_topic", "/vision_recv_data");
        //back_target_ = node_->declare_parameter("back_target", "/back_target");

        send_topic_ = node_->declare_parameter("send_topic", "/vision_send_topic");
        target_topic_ = node_->declare_parameter("target_topic", "/target");
        global_position_ = node_->declare_parameter("global_position_", "/global_costmap/published_footprint");
        pursuit_armor_id_ = node_->declare_parameter("pursuit_armor_id", "all");

        decision_send_topic_ = node_->declare_parameter("decision_send_topic", "/decision/vision_send_topic");
        //odom_topic_ = node_->declare_parameter("odom_topic", "/Odometry_transformed");
        hp_threshold = node_->declare_parameter("hp_threshold", 100.0);
        goal_pose_override_ = node_->declare_parameter("goal_pose_override", "/goal_pose_override");
        area_pub_topic_ = node_->declare_parameter("area_pub_topic", "/special_areas");
        cmd_vel_remap_topic_ = node_->declare_parameter("cmd_vel_remap_topic", "/cmd_vel_remap");
        yaw_pid_.Kp = node_->declare_parameter("PID_follower.KP", 1.0);
        yaw_pid_.Ki = node_->declare_parameter("PID_follower.KI", 0.0);
        yaw_pid_.Kd = node_->declare_parameter("PID_follower.KD", 0.0);
        yaw_pid_.integral = node_->declare_parameter("PID_follower.integral", 0.0);
        yaw_pid_inverse_ = yaw_pid_;

        decision_selector_ = node_->declare_parameter("decision_selector", 1);

        node_->declare_parameter<std::vector<std::string>>("self_blue.goal_points", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_blue.supply", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_blue.buff_points", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_blue.enermy_outpost", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_blue.attack_outpost_point", std::vector<std::string>());

        node_->declare_parameter<std::vector<std::string>>("self_red.goal_points", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_red.supply", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_red.buff_points", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_red.enermy_outpost", std::vector<std::string>());
        node_->declare_parameter<std::vector<std::string>>("self_red.attack_outpost_point", std::vector<std::string>());
        attack_outpost_ = node_->declare_parameter("attack_outpost", false);

        node_->declare_parameter<std::vector<std::string>>("special_areas_names", std::vector<std::string>());
        // 参数声明与获取 end /////////////////////////////////////////////////////////////////////////////////////////////////////

        // suscriptions
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;

        rclcpp::SubscriptionOptions sub_option_for_vision_data;
        sub_option_for_vision_data.callback_group = callback_group_for_vision_data;

        recv_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            recv_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::recv_callback, this, _1),
            sub_option_for_vision_data);
        /*
        back_target_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            back_target_,
            rclcpp::SensorDataQoS(),
            std::bind(&pubRobotStatus::back_target_callback, this, _1),
            sub_option);
        */

        send_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionSendData>(
            send_topic_, // 【是上位机发给下位机的，包含目标状态、几号机器人、距离】
            rclcpp::ServicesQoS(),
            std::bind(&PubRobotStatus::send_callback, this, _1),
            sub_option);

        target_sub_ = node_->create_subscription<hnurm_interfaces::msg::Target>(
            target_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::target_callback, this, _1),
            sub_option);

        global_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
            global_position_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::global_pose_callback, this, _1),
            sub_option);
        /*
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::odom_callback, this, _1),
            sub_option_for_vision_data);
        */

        goal_pose_override_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_pose_override_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::goal_pose_callback, this, _1),
            sub_option);
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::remap_cmd_vel_callback, this, _1),
            sub_option_for_vision_data);
        path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "path", // 来自ros_brige
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::path_callback, this, _1),
            sub_option);
        // publisher
        timer_ = node_->create_wall_timer(10ms, std::bind(&PubRobotStatus::timer_callback, this), callback_group_);
        //tf_listener_timer_ = node_->create_wall_timer(1000ms, std::bind(&PubRobotStatus::tf_listener_callback, this), callback_group_);
        decision_pub_ = node_->create_publisher<hnurm_interfaces::msg::VisionSendData>("/decision/vision_send_data", rclcpp::SensorDataQoS()); // 未使用

        pursuit_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);            // 10指的是消息队列大小,未使用
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("closest_area", 10);       // visualizeClosestArea()
        special_areas_pub_ = node_->create_publisher<hnurm_interfaces::msg::SpecialArea>(area_pub_topic_, 10); // timer
        in_special_area_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/is_in_special_area", 10);        // global_pose_callback
        cmd_vel_remap_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_remap_topic_, 10);     // remap_cmd_vel_callback

        // 需要对tfbuffer 和listener 进行初始化，不然会段错误
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    PubRobotStatus::~PubRobotStatus()
    {
        if (executor_thread_.joinable())
        {
            executor_thread_.join(); // 这个析构函数的主要作用是确保线程资源的正确回收
        }
        // 如果不调用 join()，线程可能在对象销毁后继续运行，导致以下问题：
        // 访问已释放的对象内存，引发未定义行为
        // 线程资源（如栈空间）无法被操作系统回收，造成内存泄漏
    }

    /* name: initializaion
     * @brief 把yaml文件所有参数加载到类成员变量中，打印检查
     */
    void PubRobotStatus::initializaion()
    {
        if (!recv_data.is_init)
        {
            RCLCPP_INFO(node_->get_logger(), "didn't get sentry static datas,waiting for initialization!");
            return;
        }
        else if (!is_global_init)
        {
            std::string self_color_ = "self_blue"; // default self_color as blue
            /*
              COLOR_NONE=0   RED=1   BLUE=2
            */
            if (recv_data.data.self_color.data == 1)
            {
                self_color_ = "self_red";
            }
            RCLCPP_INFO(node_->get_logger(), "Load Self Color:%s", self_color_.c_str());
            auto goal_points = node_->get_parameter(self_color_ + ".goal_points").as_string_array();           // 巡航点列表_字符串
            auto supply_point = node_->get_parameter(self_color_ + ".supply").as_string_array();               // 补给点 家_字符串
            auto bps_ = node_->get_parameter(self_color_ + ".buff_points").as_string_array();                  // 增益点列表_字符串
            auto outpost_ = node_->get_parameter(self_color_ + ".enermy_outpost").as_string_array();           // 地方前哨站坐标
            auto attack_point = node_->get_parameter(self_color_ + ".attack_outpost_point").as_string_array(); // 攻击前哨站坐标
            special_areas_names_ = node_->get_parameter("special_areas_names").as_string_array();              // 特殊区域
            // 填充巡航点列表
            for (const auto &point : goal_points)
            {
                GlobalPose gp;
                size_t comma_pos = point.find(',');
                gp.pose_x = std::stod(point.substr(0, comma_pos));
                gp.pose_y = std::stod(point.substr(comma_pos + 1));
                cruise_goals_.push_back(gp);
            }

            // 填充增益点列表
            for (const auto &point : bps_)
            {
                GlobalPose gp;
                size_t comma_pos = point.find(',');
                gp.pose_x = std::stod(point.substr(0, comma_pos));
                gp.pose_y = std::stod(point.substr(comma_pos + 1));
                buff_points_.push_back(gp);
            }

            // 打印巡航点列表_检查
            for (const auto &goal : cruise_goals_)
            {
                RCLCPP_INFO(node_->get_logger(), "Loaded goal: (%.2f, %.2f)",
                            goal.pose_x, goal.pose_y);
            }

            // 打印增益点列表_检查
            for (const auto &goal : buff_points_)
            {
                RCLCPP_INFO(node_->get_logger(), "Loaded buff points: (%.2f, %.2f)",
                            goal.pose_x, goal.pose_y);
            }

            // 填充补给点并检查
            GlobalPose sp_;
            for (const auto &point : supply_point)
            {
                size_t comma_pos = point.find(',');
                sp_.pose_x = std::stod(point.substr(0, comma_pos));
                sp_.pose_y = std::stod(point.substr(comma_pos + 1));
                supply_goal.header.frame_id = "map"; // 设置坐标系 ID
                supply_goal.pose.position.x = sp_.pose_x;
                supply_goal.pose.position.y = sp_.pose_y;
                supply_goal.pose.position.z = 0.0;
                supply_goal.pose.orientation.x = 0;
                supply_goal.pose.orientation.y = 0;
                supply_goal.pose.orientation.z = 0;
                supply_goal.pose.orientation.w = 1;
                // print supply point
                RCLCPP_INFO(node_->get_logger(), "Loaded supply point: (%.2f, %.2f)",
                            sp_.pose_x, sp_.pose_y);
            }

            // decode enermy_outpost point

            for (const auto &point : outpost_)
            {
                size_t comma_pos = point.find(',');
                enermy_outpost_.pose_x = std::stod(point.substr(0, comma_pos));
                enermy_outpost_.pose_y = std::stod(point.substr(comma_pos + 1));

                RCLCPP_INFO(node_->get_logger(), "Loaded enermy outpost: (%.2f, %.2f)",
                            enermy_outpost_.pose_x, enermy_outpost_.pose_y);
            }

            for (const auto &point : attack_point)
            {
                size_t comma_pos = point.find(',');
                attack_outpost_point_.pose_x = std::stod(point.substr(0, comma_pos));
                attack_outpost_point_.pose_y = std::stod(point.substr(comma_pos + 1));

                RCLCPP_INFO(node_->get_logger(), "Loaded attack_outpost_point_: (%.2f, %.2f)",
                            attack_outpost_point_.pose_x, attack_outpost_point_.pose_y);
            }

            // special_areas_names_这里只是读区域名称（R1）
            for (const auto &name : special_areas_names_)
            {
                // 调用工具函数对数据结构进行填充
                special_areas_[name].spa = getAreas(name);
                special_areas_[name].spa_paths_ends_ = getPathEnds(name);
                special_areas_[name].center = getCenter(special_areas_[name].spa.points);
                std::cout << "area " << name << " x=" << special_areas_[name].center.pose_x << " y=" << special_areas_[name].center.pose_y << std::endl;
            }

            RCLCPP_INFO(node_->get_logger(), "initialized,decision is running");
            is_global_init = true;
        }
    }






    /**************FSM TO CONTROL BT********************/

    BT::NodeStatus PubRobotStatus::tick()
    {
        callback_group_executor_for_vision_data.spin_some();
        if (!is_global_init)
        {
            initializaion();
            setOutput("navigate_to_pose_status", false);
            setOutput("cruise_status", false);
            setOutput("is_in_special_area", false);
            setOutput("do_clear", false);
            return BT::NodeStatus::FAILURE; // must execute BT after initialized
        }

        callback_group_executor_.spin_some();
        supply_goal.header.stamp = node_->now(); // update supply goal time stamp

        std::lock_guard<std::mutex> lock(goal_poses_mutex);
        auto copy_goals = goal_poses_;

        if (supply_)
        {
            std::cout << "Supply Mode on, ahead to the supply !" << std::endl;
            setOutput("cruise_status", false);
            setOutput("navigate_to_pose_status", false);
            setOutput("pursuit_status", false);
            setOutput("hp_status", true); // main status
            setOutput("supply_goal", supply_goal);
        }
        else if (pursuit_)
        {

            setOutput("pursuit_status", true); // main status
            setOutput("navigate_to_pose_status", false);
            setOutput("cruise_status", false);
            setOutput("hp_status", false);
            setOutput("target_goal", target_pose_);
            if (stay_)
                setOutput("pursuit_status", false); // todo: pursuit away mode
        }
        else if (cruise_)
        {
            std::cout << "Cruise Mode on !" << std::endl;
            setOutput("navigate_to_pose_status", false);
            setOutput("pursuit_status", false);
            setOutput("hp_status", false);
            setOutput("cruise_status", true); // main status
            if (recv_data.data.current_enemy_outpost_hp != 0 && attack_outpost_)
            {
                setOutput("cruise_goal", update_goal(attack_outpost_point_));
            }
            else
                setOutput("cruise_goal", update_goal(cruise_goals_[cruise_iterator.idx]));
        }
        // }

        // 在特殊区域内，设置控制器为FollowPath
        if (is_in_special_area)
        {
            setOutput("is_in_special_area", true);
            // setOutput("do_clear",true);
            setOutput("current_controller", "FollowPath");
            // setOutput("goal_poses",goal_poses_);
            // std::cout<<"current_controller: FollowPath"<<std::endl;
        }
        // 不在特殊区域内，设置控制器为Omni
        else
        {
            goal_poses_.clear();
            setOutput("is_in_special_area", false);
            // setOutput("do_clear",false);
            setOutput("current_controller", "Omni");
            // std::cout<<"current_controller: Omni"<<std::endl;
        }

        return BT::NodeStatus::SUCCESS; // 每次tick直接返回success，作为一定频率更新的global status publisher
    }

    /****************FSM TO CONTROL BT************************/






    /***************************************工具函数 start **********************************************/

    /**************Special Area start****************/
    /*  name: getAreas
        @brief: 获取区域边界，类型，通行类型，填充结构体special_areas_的自定义消息类型
        @return: 返回第一个成员变量，在initialization()中调用
    */
    hnurm_interfaces::msg::SpecialArea PubRobotStatus::getAreas(const std::string &area_name_)
    {
        /*
        type: "both"
        area: "across"
        path_ends: ["9.04,4.61","11.03,4.55",] # path_ends:穿越路径点
        polygon_ends:                          # polygon_ends:多边形边界
                ["7.97,4.16","8.61,3.50","9.30,4.21","11.29,4.19","11.32,4.98","8.68,5.10","7.97,4.16",]
        */
        node_->declare_parameter<std::vector<std::string>>(area_name_ + ".polygon_ends", std::vector<std::string>());
        std::string test_type = node_->declare_parameter<std::string>(area_name_ + ".type", "both"); // 默认值：both
        std::string test_area = node_->declare_parameter<std::string>(area_name_ + ".area", "red");  // 默认值：red

        auto polygon_ends_ = node_->get_parameter(area_name_ + ".polygon_ends").as_string_array();

        // decode polygon ends
        hnurm_interfaces::msg::SpecialArea spa_;
        /*
        SpecialArea:包含：
        points（多边形边界点）
        area（区域类型，（across可跨越,3））
        type（通行类型，2：双向通行,both）
        */
        for (const auto &point : polygon_ends_)
        {
            hnurm_interfaces::msg::ZoneEndPoint2D zep; // float64 x;float64 y;
            size_t comma_pos = point.find(',');
            zep.x = std::stod(point.substr(0, comma_pos));
            zep.y = std::stod(point.substr(comma_pos + 1));
            spa_.points.push_back(zep);
        }
        if (test_type == "both")
        {
            spa_.type.data = 2;
        }
        else if (test_type == "single")
        {
            spa_.type.data = 1;
        }
        else
        {
            spa_.type.data = 0;
        }

        // 可以查阅接口消息数据类型:全为across 表示都是可以跨越地形
        if (test_area == "across")
        {
            spa_.area.data = 3;
        }
        else if (test_area == "mid")
        {
            spa_.area.data = 2;
        }
        else if (test_area == "red")
        {
            spa_.area.data = 1;
        }
        else
        {
            spa_.area.data = 0;
        }

        return spa_;
    }

    /*  name: getPathEnds
        @brief: 获取穿越路径点
        @return: 返回第二个成员变量，在initialization()中调用
    */
    std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> PubRobotStatus::getPathEnds(const std::string &area_name_)
    {
        node_->declare_parameter<std::vector<std::string>>(area_name_ + ".path_ends", std::vector<std::string>());
        auto test_path = node_->get_parameter(area_name_ + ".path_ends").as_string_array();

        std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> path_ends_;
        // decode path ends
        for (const auto &point : test_path)
        {
            hnurm_interfaces::msg::ZoneEndPoint2D pep;
            size_t comma_pos = point.find(',');
            pep.x = std::stod(point.substr(0, comma_pos));
            pep.y = std::stod(point.substr(comma_pos + 1));
            path_ends_.push_back(pep);
        }
        return path_ends_;
    }

    /*  name: getCenter
        @brief: 计算特殊区域的几何中心点，这个点用于估计区域和现在坐标的距离，用于抵达特殊区域
        @return: 返回第三个成员变量，在initialization()中调用
    */
    GlobalPose PubRobotStatus::getCenter(const std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> &points)
    {
        std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> end_points = points;
        double total_y_ = 0.0;
        double total_x_ = 0.0;
        GlobalPose center_;
        if (!end_points.empty())
            end_points.pop_back();
        else
        {
            center_.pose_x = 0.0;
            center_.pose_y = 0.0;
            return center_;
        }
        for (const auto &ep : end_points)
        {
            total_y_ += ep.y;
            total_x_ += ep.x;
        }
        center_.pose_x = total_x_ / end_points.size();
        center_.pose_y = total_y_ / end_points.size();
        return center_;
    }

    /****************Special Area end**********************/

    /*  name: calculate_distance
            @brief: 计算现在坐标和目标坐标的距离平方
            @return: 返回这个距离
        */
    double PubRobotStatus::calculate_distance(GlobalPose current_pose, GlobalPose target_pose)
    {
        //target_pose来自target_callback()
        // return std::sqrt(std::pow(current_pose.pose_x - target_pose.pose_x, 2) + std::pow(current_pose.pose_y - target_pose.pose_y, 2));
        double dx = current_pose.pose_x - target_pose.pose_x;
        double dy = current_pose.pose_y - target_pose.pose_y;
        return dx * dx + dy * dy;
    }

    /*  name: getClosestArea
        @brief: 计算现在位姿和特殊区域的几何中心点的距离平方，返回距离最小的特殊区域的【名称】
        @return: 返回这个名称
    */
    std::string PubRobotStatus::getClosestArea()
    {
        if (special_areas_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "special areas is empty");
            return "";
        }
        std::string cspa_;
        double min_dist_ = std::numeric_limits<double>::max(); // init as max
        for (const auto &spa_ : special_areas_)
        {
            // RCLCPP_ERROR(node_->get_logger(), "spa: %s",spa_.first.c_str());
            double dist_ = calculate_distance(current_pose_, spa_.second.center);
            if (dist_ < min_dist_)
            {
                min_dist_ = dist_;
                cspa_ = spa_.first;
            }
        }
        return cspa_;
    }

    void PubRobotStatus::visualizeClosestArea()
    {
        //   if (special_areas_.empty()) {
        //     return;
        // }
        // std::cout<<"closest area: "<<closest_area_<<std::endl;
        GlobalPose closest_center = special_areas_[closest_area_].center;
        visualization_msgs::msg::MarkerArray marker_array;

        // delete old marker
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header.frame_id = "map";
        delete_marker.ns = "closest_area_link";
        marker_array.markers.push_back(delete_marker);

        // def Marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "closest_area_link";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // set ends
        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = current_pose_.pose_x;
        start_point.y = current_pose_.pose_y;
        start_point.z = 0.0;
        end_point.x = closest_center.pose_x;
        end_point.y = closest_center.pose_y;
        end_point.z = 0.0;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        // set appreance
        marker.scale.x = 0.1; // 线宽
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);

        // path marker
        marker.points.clear();
        marker.id = 20;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        start_point.x = special_areas_[closest_area_].spa_paths_ends_[0].x;
        start_point.y = special_areas_[closest_area_].spa_paths_ends_[0].y;
        start_point.z = 0.0;
        end_point.x = special_areas_[closest_area_].spa_paths_ends_[1].x;
        end_point.y = special_areas_[closest_area_].spa_paths_ends_[1].y;
        end_point.z = 0.0;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        marker.scale.x = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);

        // publish marer array
        marker_pub_->publish(marker_array);
    }

    bool PubRobotStatus::isinSpecialArea(GlobalPose current_pose, std::string area)
    {
        bool inside = false;
        hnurm_interfaces::msg::SpecialArea temp_spa_ = special_areas_[area].spa;
        const size_t n = temp_spa_.points.size();
        // new ray method
        if (n < 3)
            return false;
        for (size_t i = 0, j = n - 1; i < n; j = i++)
        {
            const float xi = temp_spa_.points[i].x, yi = temp_spa_.points[i].y;
            const float xj = temp_spa_.points[j].x, yj = temp_spa_.points[j].y;
            const bool y_in_range = (yi > current_pose.pose_y) != (yj > current_pose.pose_y);
            if (y_in_range)
            {
                const float dx = xj - xi, dy = yj - yi;
                if (dy == 0)
                    continue; // avoid divide zero
                const float t = (current_pose.pose_y - yi) / dy;
                const float x_intersect = xi + t * dx;
                if (current_pose.pose_x <= x_intersect)
                {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    void PubRobotStatus::update_poses()
    {
        GlobalPose temp_goal_pose;
        std::lock_guard<std::mutex> lock(goal_poses_mutex);
        if (supply_)
        {
            // 补给模式：设置补给点为目标
            GlobalPose temp_pose_;
            temp_pose_.pose_x = supply_goal.pose.position.x;
            temp_pose_.pose_y = supply_goal.pose.position.y;
            goal_updater(temp_pose_);
        }
        else if (pursuit_)
        {
            // 追击模式：设置目标为敌方位置
            GlobalPose temp_pose_;
            temp_pose_.pose_x = target_pose_.pose.position.x;
            temp_pose_.pose_y = target_pose_.pose.position.y;
            goal_updater(temp_pose_);
        }
        else if (cruise_)
        {
            if (attack_outpost_ && recv_data.data.current_enemy_outpost_hp != 0)
            {
                // 如果attack_outpost_指令（状态）为true并却对方前哨血量不为0，把目标改为对方前哨
                goal_updater(attack_outpost_point_);
            }
            else
                goal_updater(cruise_goals_[cruise_iterator.idx]);
        }

        // generate poses path through

        /*****************非特殊区域****************************/
        if (!is_in_special_area && !goal_poses_.empty())
        {
            const auto &current_goal = goal_poses_.front(); // 获取目标点队列中的第一个目标点（当前执行目标）
            GlobalPose temp_pose_;
            temp_pose_.pose_x = current_goal.pose.position.x;
            temp_pose_.pose_y = current_goal.pose.position.y;
            /*
            is_executing_ : 是否正在执行导航
            goal_pose_updated_ ： 是否更新了目标点
            3. 新目标与旧目标的X坐标不同
            4. 新目标与旧目标的Y坐标相同
            总条件：导航任务正在执行时，目标点发生了X方向的变化
            当目标点发生显著变化时，强制取消当前任务，避免机器人执行错误路径
            */
            if (is_executing_ && goal_pose_updated_ && !(temp_goal_pose.pose_x == current_goal.pose.position.x) && (temp_goal_pose.pose_y == current_goal.pose.position.y)) // force cancel if pre goal pose unequal to the current goal pose
            {
                goal_poses_.clear();
                is_executing_ = false;
                return;
            }
            /*机器人已到达当前目标点（误差范围内）**/
            if (goal_checker(temp_pose_, current_pose_, 0.1)) // final point reached ,reset
            {
                goal_poses_.clear();
                is_executing_ = false;
                return;
            }
        }
        // 当机器人不在执行导航任务且处于特殊区域内时执行后续逻辑
        // "隧道"模型：将特殊区域视为需要"穿过"的隧道
        else if (!is_executing_ && is_in_special_area)
        {
            std::vector<double> distance_to_current_pose_; // // 机器人到路径端点的距离
            std::vector<double> distance_to_goal_pose_;    // 目标点到路径端点的距离
            std::vector<GlobalPose> cal_path_length_;
            for (const auto &path_ends_ : special_areas_[closest_area_].spa_paths_ends_)
            {
                GlobalPose temp_ends_;
                temp_ends_.pose_x = path_ends_.x;
                temp_ends_.pose_y = path_ends_.y;
                // double dist_ = calculate_distance(current_pose_,temp_ends_);

                distance_to_current_pose_.push_back(calculate_distance(current_pose_, temp_ends_));
                distance_to_goal_pose_.push_back(calculate_distance(current_pose_, temp_ends_));

                cal_path_length_.push_back(temp_ends_);
            }
            double path_length_ = calculate_distance(cal_path_length_[0], cal_path_length_[1]);

            std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> temp_paths_ends_ = special_areas_[closest_area_].spa_paths_ends_;
            // check if current inside the tunnel
            //  bool is_in_tunnel_ = false;
            if ((path_length_ > distance_to_current_pose_[0]) && (path_length_ > distance_to_current_pose_[1])) // udpate in the tunnel
            {
                // is_in_tunnel_ = true;
                if (distance_to_goal_pose_[0] < distance_to_goal_pose_[1])
                {
                    GlobalPose temp_ends_;
                    temp_ends_.pose_x = temp_paths_ends_[0].x;
                    temp_ends_.pose_y = temp_paths_ends_[0].y;
                    goal_poses_.push_back(update_goal(temp_ends_));
                }
                else
                {
                    GlobalPose temp_ends_;
                    temp_ends_.pose_x = temp_paths_ends_[1].x;
                    temp_ends_.pose_y = temp_paths_ends_[1].y;
                    goal_poses_.push_back(update_goal(temp_ends_));
                }
                // if (goal_pose_updated_||cruise_)
                // {
                goal_poses_.push_back(update_goal(goal_override_pose)); // 修复：使用 goal_override_pose
                // }
                temp_goal_pose = goal_override_pose; // 修复：使用 goal_override_pose
                is_executing_ = true;
                return;
            }

            if (distance_to_current_pose_[0] < distance_to_current_pose_[1])
            {
                for (const auto &path_ends_ : temp_paths_ends_)
                {
                    GlobalPose temp_ends_; // 修复：GlobalPose 而非 current_global_pose
                    temp_ends_.pose_x = path_ends_.x;
                    temp_ends_.pose_y = path_ends_.y;
                    goal_poses_.push_back(update_goal(temp_ends_));
                }

                // if (goal_pose_updated_)
                // {
                goal_poses_.push_back(update_goal(goal_override_pose)); // 修复：使用 goal_override_pose
                // }
            }
            else
            {
                std::reverse(temp_paths_ends_.begin(), temp_paths_ends_.end());
                for (const auto &path_ends_ : temp_paths_ends_)
                {
                    GlobalPose temp_ends_; // 修复：GlobalPose 而非 current_global_pose
                    temp_ends_.pose_x = path_ends_.x;
                    temp_ends_.pose_y = path_ends_.y;
                    goal_poses_.push_back(update_goal(temp_ends_));
                }
                // if (goal_pose_updated_)
                // {
                goal_poses_.push_back(update_goal(goal_override_pose)); // 修复：使用 goal_override_pose
                // }
            }
            temp_goal_pose = goal_override_pose; // 修复：使用 goal_override_pose
            is_executing_ = true;
        }
        // 仅当机器人处于特殊区域内且正在执行导航任务时执行后续逻辑
        else if (is_in_special_area && is_executing_)
        {
            // 导航任务正在执行时，目标点发生了X方向的变化，强制取消当前任务
            if (goal_pose_updated_ && !(temp_goal_pose.pose_x == goal_override_pose.pose_x) && (temp_goal_pose.pose_y == goal_override_pose.pose_y)) // 修复：使用 goal_override_pose
            {
                goal_poses_.clear();
                is_executing_ = false;
                return;
            }
            // 防止处理空队列导致的潜在错误
            if (goal_poses_.empty())
            {
                RCLCPP_WARN(node_->get_logger(), "Goal queue is empty!");
                is_executing_ = false;
                return;
            }

            const auto &current_goal = goal_poses_.front();
            GlobalPose temp_pose_;
            temp_pose_.pose_x = current_goal.pose.position.x;
            temp_pose_.pose_y = current_goal.pose.position.y;

            if (goal_checker(temp_pose_, current_pose_, 0.1))
            { // tolerance to path through ends
                goal_poses_.erase(goal_poses_.begin());
                RCLCPP_INFO(node_->get_logger(), "Update goal poses");
            }

            double dis_ = std::hypot(temp_pose_.pose_x - current_pose_.pose_x,
                                     temp_pose_.pose_y - current_pose_.pose_y);
            RCLCPP_INFO(node_->get_logger(), "Target: (%.2f, %.2f), Current: (%.2f, %.2f), Distance: %.2f",
                        temp_pose_.pose_x, temp_pose_.pose_y, current_pose_.pose_x, current_pose_.pose_y, dis_);
            int i = 0;
            auto copy_goals = goal_poses_; // 防止遍历时被修改
            for (const auto &pose : copy_goals)
            {
                RCLCPP_INFO(node_->get_logger(), "Goal %d: (%.2f, %.2f)",
                            i++, pose.pose.position.x, pose.pose.position.y);
            }
        }
    }

    /*  name: calculateAngle
         @brief: 计算两个向量之间的夹角，单位为度
         @return: 返回这个角度
     */
    double PubRobotStatus::calculateAngle(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2)
    {
        double dot = v1.dot(v2);
        double norms = v1.norm() * v2.norm();
        if (norms == 0)
            throw std::runtime_error("zero vector");

        // 计算夹角的弧度值
        double cosTheta = std::clamp(dot / norms, -1.0, 1.0);
        double angleRad = std::acos(cosTheta);

        // 计算叉积确定方向（二维向量的叉积为z分量）
        double cross = v1.x() * v2.y() - v1.y() * v2.x();
        double sign = (cross < 0) ? -1.0 : 1.0; // 叉积为负表示顺时针，角度取负

        // 转换为角度并应用方向符号
        return sign * angleRad * 180.0 / M_PI;
    }

    /*  name: all_off
         @brief: 关闭所有相关的状态输出
         @return: 无
     */
    void PubRobotStatus::all_off()
    {
        setOutput("pursuit_status", false);
        setOutput("cruise_status", false);
    }

     /*  name: goal_checker
         @brief: 检查机器人是否到达目标点
         @return: 如果到达目标点则返回true，否则返回false
     */
    bool PubRobotStatus::goal_checker(GlobalPose cruise_goal, GlobalPose current_pose, double goal_tolerance)
    {
        // hypot计算两点间距离的平方根
        double distance_ = std::hypot(current_pose.pose_x - cruise_goal.pose_x, current_pose.pose_y - cruise_goal.pose_y);
        if (distance_ < goal_tolerance)
        {
            return true;
        }
        return false;
    }


    /* name: update_goal，【不是回调】
     * @brief 更新巡航目标点的位姿信息
     * @param cruise_goal 巡航目标点
     */
    geometry_msgs::msg::PoseStamped PubRobotStatus::update_goal(GlobalPose cruise_goal)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->now(); // 设置时间戳为当前时间
        pose.header.frame_id = "map";     // 设置坐标系 ID
        pose.pose.position.x = cruise_goal.pose_x;
        pose.pose.position.y = cruise_goal.pose_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        // pursuit_pub_->publish(pose);
        return pose;
    }

    /* name: goal_updater
     * @brief 更新云台点覆盖
     * @param goal 云台点覆盖
     */
    void PubRobotStatus::goal_updater(GlobalPose goal)
    {
        goal_override_pose = goal; // 更新当前导航点为参数点
    }

    /* name: find_Away_point
     * @brief：路径规划辅助函数，用于计算机器人从当前位置远离目标点时应到达的安全位置坐标
       @param：1.yaw：float 类型，目标点相对于机器人当前位置的偏航角（单位：弧度）
       @param：2.distance：float 类型，目标点与机器人当前位置的距离（单位：米）
     */
    GlobalPose PubRobotStatus::find_Away_point(float yaw, float distance)
    {
        const float target_x_base = distance * cos(yaw);
        const float target_y_base = distance * sin(yaw);
        // 转换到map坐标系
        const float target_x_map = current_pose_.pose_x + target_x_base;
        const float target_y_map = current_pose_.pose_y + target_y_base;

        // 目标到机器人的方向向量
        const float dx = current_pose_.pose_x - target_x_map;
        const float dy = current_pose_.pose_y - target_y_map;
        const float direction_norm = sqrt(dx * dx + dy * dy);

        // 单位向量
        const float unit_dx = dx / direction_norm;
        const float unit_dy = dy / direction_norm;
        // 期望保持的距离
        const float desired_dist = 3.4;

        // 计算期望点坐标（从目标点向机器人方向后退）
        const float desired_x = target_x_map + unit_dx * (direction_norm - desired_dist);
        const float desired_y = target_y_map + unit_dy * (direction_norm - desired_dist);
        GlobalPose target_pose_{desired_x, desired_y};
        return target_pose_;
    }

    /* name: calculateAverage
     * @brief：用于计算多边形点集的平均坐标，作为机器人当前位置的估计值。将定位系统发布的 footprint（多边形）转换为机器人的中心点坐标
       @param：msg geometry_msgs::msg::PolygonStamped msg - 包含多边形点集的 ROS 消息
     */
    GlobalPose PubRobotStatus::calculateAverage(geometry_msgs::msg::PolygonStamped msg)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        GlobalPose cgp_;
        for (auto const &p : msg.polygon.points)
        {
            x_sum += p.x;
            y_sum += p.y;
        }
        cgp_.pose_x = x_sum / msg.polygon.points.size();
        cgp_.pose_y = y_sum / msg.polygon.points.size();
        return cgp_;
    }

    /* name: getClosestBuff
     * @brief：用于获取离机器人最近的增益点坐标。
       @return：current_global_pose 类型，离机器人最近的增益点坐标
     */
    GlobalPose PubRobotStatus::getClosestBuff()
    {
        GlobalPose closest_point_;                    // 存储最近的增益点
        double min_dist_ = std::numeric_limits<double>::max(); // init as max，双精度浮点数的最大值，确保第一个比较的增益点总是被选中
        for (const auto &bp : buff_points_)
        {
            double dist_ = calculate_distance(current_pose_, bp);
            if (dist_ < min_dist_)
            {
                min_dist_ = dist_;
                closest_point_ = bp;
            }
        }
        return closest_point_;
    }
    /***************************************工具函数 end*****************************************************/






    /***************************************回调函数 start*****************************************************/

    /* name: path_callback
     * @brief 该函数负责处理全局路径规划结果，计算机器人当前方向与路径方向的偏差角。
     * @订阅话题：path:来自ros_bridge
     * @param msg 导航路径消息指针
     */
    void PubRobotStatus::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                "base_footprint",                     // target frame
                "odom",                               // source frame
                rclcpp::Time(0),                      // 获取最新变换
                rclcpp::Duration::from_seconds(1.0)); // 超时时间
            // RCLCPP_ERROR(node_->get_logger(), "Transform found!");
        }
        catch (tf2::TransformException &ex)
        {
            // RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s. Waiting for transform...", ex.what());
            // 失败直接返回
            return;
        }
        /*************************位姿解算，关注yaw**********************************/
        // get roatation quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(transform.transform.rotation, quat);
        // transform into eulerangle
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        /*************************位姿解算，关注yaw***********************************/

        // current_direction：基于偏航角计算的机器人当前朝向向量
        Eigen::Vector2d current_direction(
            std::cos(yaw),
            std::sin(yaw));

        int trace_forward = 100;
        geometry_msgs::msg::Point start = msg->poses[0].pose.position;
        geometry_msgs::msg::Point end = msg->poses[trace_forward].pose.position;

        // target_direction：路径的前进方向向量（使用路径的前100个点计算）
        Eigen::Vector2d target_direction(
            end.x - start.x,
            end.y - start.y);
        std::lock_guard<std::mutex> lock(diff_yaw_mutex); // 互斥锁，头文件里有，保护yaw_diff
        yaw_diff = calculateAngle(current_direction, target_direction);
    }

    /* name: remap_cmd_vel_callback
     * @brief 函数负责在特殊区域内重映射速度命令，调整机器人的偏航角以保持沿特殊区域路径的正确方向。
     * @订阅话题：cmd_vel（速度命令）
     * @param msg geometry_msgs::msg::Twist（线速度和角速度）
     */
    void PubRobotStatus::remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!is_global_init) // 确保系统已完成全局初始化，否则直接返回
            return;
        geometry_msgs::msg::Twist vel_pub_;
        vel_pub_ = *msg; // 复制原始速度命令

        // 计算特殊区域路径的方向向量（两条可能的方向）
        Eigen::Vector2d target_direction(
            special_areas_[closest_area_].spa_paths_ends_[0].x - special_areas_[closest_area_].spa_paths_ends_[1].x,
            special_areas_[closest_area_].spa_paths_ends_[0].y - special_areas_[closest_area_].spa_paths_ends_[1].y);

        Eigen::Vector2d target_direction2(
            special_areas_[closest_area_].spa_paths_ends_[1].x - special_areas_[closest_area_].spa_paths_ends_[0].x,
            special_areas_[closest_area_].spa_paths_ends_[1].y - special_areas_[closest_area_].spa_paths_ends_[0].y);

        // // 计算垂直于路径方向的向量（逆时针）
        Eigen::Vector2d perpendicular_ccw(-target_direction.y(), target_direction.x());
        Eigen::Vector2d perpendicular_ccw_2(-target_direction2.y(), target_direction2.x());

        // target_direction.normalize();
        perpendicular_ccw.normalize();
        // target_direction is calculate in the map frame,so current_direction must transform in the map frame
        // listen to the tf: base_footprint -> map
        try
        {
            transform_m2bf = tf_buffer_->lookupTransform( // 查找从base_footprint->map 的变换
                "map",                                    // 目标坐标系
                "base_footprint",                         // 源坐标系
                rclcpp::Time(0),                          // 时间戳（0表示最新）
                rclcpp::Duration::from_seconds(0.1)       // 超时时间
            );
            // RCLCPP_ERROR(node_->get_logger(), "Transform found!");
        }
        catch (tf2::TransformException &ex)
        {
            // RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s. Waiting for transform...", ex.what());
            transform_m2bf = geometry_msgs::msg::TransformStamped(); // 重置为默认值
            return;                                                  // 如果转换查找失败，则退出该回调
        }
        tf2::Quaternion q(
            transform_m2bf.transform.rotation.x,
            transform_m2bf.transform.rotation.y,
            transform_m2bf.transform.rotation.z,
            transform_m2bf.transform.rotation.w);
        tf2::Matrix3x3 m(q);

        // 提取欧拉角 (ZYX顺序：yaw, pitch, roll)
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // RPY 对应 ZYX 欧拉角

        // 计算机器人当前方向向量
        Eigen::Vector2d current_direction(
            std::cos(yaw),
            std::sin(yaw));
        current_direction.normalize();

        // 只有在特殊区域内才进行速度重映射
        if (is_in_special_area)
        {
            // 计算当前方向与期望垂直方向的偏差
            double error = calculateAngle(perpendicular_ccw, current_direction);
            

            if (yaw_pid_.prev_time.nanoseconds() == 0)
            {
                yaw_pid_.prev_time = node_->now();
                return; // skip the first calculate
            }
            auto now = node_->now();

            // PID控制器计算
            double dt = (now - yaw_pid_.prev_time).seconds();
            if (dt <= 0)
            {
                dt = 0.01;
            }
            double P = yaw_pid_.Kp * error;

            // 积分项（带抗饱和）
            yaw_pid_.integral += error * dt;
            double I = yaw_pid_.Ki * yaw_pid_.integral;
            // 微分项
            double derivative = (error - yaw_pid_.prev_error) / dt;
            double D = yaw_pid_.Kd * derivative;

            // 更新记录值
            yaw_pid_.prev_error = error;
            yaw_pid_.prev_time = now;

            // 计算角速度输出
            double angular_z = P + I + D;

            // vel_pub_.linear.x = 0;//for debug
            // vel_pub_.linear.y = 0;
            // vel_pub_.linear.z = std::clamp(output_cmd_, 0.0, 360.0);
            vel_pub_.linear.z = -recv_data.data.yaw + angular_z;
        }

        cmd_vel_remap_pub_->publish(vel_pub_);
    }

    /* name: goal_pose_callback
     * @brief 函数负责在接收到【云台手】发送的目标位姿时，更新导航目标点，并设置相关标志以指示目标已更新。
     * @订阅话题：goal_pose_override_（云台发送的目标位姿）
     * @param msg geometry_msgs::msg::PoseStamped（目标位姿）
     */
    void PubRobotStatus::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        /*
          goal_updated?
          goal_reached?
          timeout?
          when goal_reached or timeout --> cancel navigate task
        */
        // goal_:云台手发送目标点
        goal_ = *msg;
        goal_.header.stamp = node_->now(); // 设置时间戳为当前时间
        goal_.header.frame_id = "map";     // 设置坐标系 ID
        goal_.pose.position.z = 0.0;
        goal_.pose.orientation.x = 0;
        goal_.pose.orientation.y = 0;
        goal_.pose.orientation.z = 0;
        goal_.pose.orientation.w = 1;
        goal_pose_updated_ = true;
        std::lock_guard<std::mutex> lock(goal_poses_mutex);
        if (!goal_poses_.empty())                // goal_poses_:所有导航点数组。
            goal_poses_.clear();                 // 一旦有云台点，清空所有旧的目标点
        goal_override_pose.pose_x = msg->pose.position.x; // 填充public成员变量
        goal_override_pose.pose_y = msg->pose.position.y;
        goal_pose_timeout_start_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "goal_pose: x=%f ,y=%f ", goal_override_pose.pose_x, goal_override_pose.pose_y);
    }

    /* name: recv_callback
     * @brief 函数负责接收视觉系统发送的数据，更新机器人状态（包括当前血量、姿态等），根据血量和允许发射量判断是否需要补给，并保存完整数据用于后续决策处理。
     * @订阅话题：recv_topic_（默认/vision_recv_data，视觉系统发送的机器人状态数据）
     * @param msg hnurm_interfaces::msg::VisionRecvData（包含当前血量、姿态、允许发射量等视觉系统数据）
     */
    void PubRobotStatus::recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        // referee_status_: 机器人状态
        // 更新当前血量
        referee_status_.current_hp = msg->current_hp;

        // 初始化偏航角（仅第一次接收数据时）
        if (!get_init_yaw_)
        {
            init_yaw_ = msg->yaw;
            get_init_yaw_ = true;
        }
        referee_status_.rpy.roll = msg->roll;
        referee_status_.rpy.pitch = msg->pitch;
        referee_status_.rpy.yaw = msg->yaw;

        // allow_fire_amount：允许发射量
        // 更新五大状态（pursuit_ 、cruise_ 、supply_ 、attack_outpost_、stay_ = false））之一——补给状态 supply
        if ((msg->allow_fire_amount <= 50.0) || (msg->current_hp <= 180.0))
            supply_ = true;
        else if (msg->current_hp > 300.0 && msg->allow_fire_amount > 90)
        {
            supply_ = false; // 开局默认为false，只有当血量第一次降低到阈值以下才会转变为true，后续需要加血到360.0以上才恢复false
        }

        recv_data.data = *msg;
        recv_data.is_init = true;
    }

    /* name: timer_callback
     * @brief：作为机器人控制系统的核心协调模块，负责周期性处理多项关键任务，包括导航目标管理和特殊区域识别与发布
     */
    int counter = 0;
    void PubRobotStatus::timer_callback() // 此处实现全向感知相机的进程分配
    {
        // 检查当前控制权是否属于后视相机   !!!!
        hnurm_interfaces::msg::VisionSendData send;
        rclcpp::Time now = node_->now();

        // if (current_authority.load(std::memory_order_acquire) == ControlAuthority::MainCamera) {
        //     // std::cout<<"main camera obtained control !"<<std::endl;
        //     back_camera_control.on_control = false;   //reset
        //     return;  // 控制权已被抢占，不执行后续操作
        // }
        // if(!send_data.target_state&&back_camera_control.on_control)  //当主相机没有检测到，并且后视相机检测到了
        // {
        //   // RCLCPP_INFO(node_->get_logger(), "back camera obtained control");

        //   send.header.stamp = now;
        //   send.header.frame_id = "serial_send";
        //   send.target_state.data = hnurm_interfaces::msg::TargetState::CONVERGING;    //send converging state
        //   send.yaw = back_camera_control.reference_yaw_ -45.0 + (back_target_bias);   //减小转动的幅度，靠惯性转过去
        //   // if (std::abs(back_camera_control.reference_yaw_ - referee_status_.rpy.yaw)<10.0)
        //   // {
        //   //   back_camera_control.on_control = false;
        //   //   return;
        //   // }
        //   // decision_pub_->publish(send);
        //   }

        /***************Navigate To Pose Override****************/

        if (goal_pose_updated_)
        {
            pub_goal_pose_ = true;
        }
        if (pub_goal_pose_)
        {
            const auto elapsed = node_->now() - goal_pose_timeout_start_time_; // goal_pose_callback()
            if (elapsed.seconds() >= goal_pose_timeout_ || goal_checker(goal_override_pose, current_pose_, 0.25))
            {
                // 到达目标或者超时
                pub_goal_pose_ = false;
                goal_pose_updated_ = false; // 到达了之后就把云台点更新了的状态值为false
                // std::cout<<"goal_pose: x="<<goal_pose.pose_x<<" y="<<goal_pose.pose_y<<std::endl;
            }
        }
        // if(pub_goal_pose_)
        // {
        //   std::cout<<"pub_goal_pose_     on"<<std::endl;
        // }
        // else std::cout<<"pub_goal_pose_   off"<<std::endl;
        /***************Navigate To Pose Override End****************/

        /***************Special Area Pub****************/
        closest_area_ = getClosestArea();
        if (closest_area_ != "")
        {
            special_areas_pub_->publish(special_areas_[closest_area_].spa);
        }
        // hnurm_interfaces::msg::SpecialArea tmp_sp = special_areas_[closest_area_].spa;

        // RCLCPP_INFO(node_->get_logger(), "pub special area filter:%s",closest_area_.c_str());
        // std::cout<<"pub special area fitler:"<<closest_area_<<std::endl;
        /***************Special Area Pub End****************/
    }

    /* name: send_callback
     * @brief：接收上位机发送的目标【状态】和【距离】数据，根据目标状态判断是否开启追击模式或巡逻模式，设置目标距离
     */
    void PubRobotStatus::send_callback(const hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        if (msg->target_state.data >= 1) // 利用是否识别到控制是否追击和巡逻
        {
            pursuit_ = true;
            cruise_ = false;
        }
        else
        {
            pursuit_ = false;
            cruise_ = true;
        }

        // 将视觉系统提供的目标距离信息保存到 target_data 结构体中，用于后续计算目标导航点。
        target_data.target_distance = msg->target_distance; 
        // used for calculate target nav point
        // send_data.target_state = (msg->target_state.data >0 ) ? true:false;    //recv vision data
        // if(send_data.target_state)  //主相机获取控制权
        // {
        //   // RCLCPP_INFO(node_->get_logger(), "main camera obtained control");
        //   current_authority.store(ControlAuthority::MainCamera, std::memory_order_release);
        //   back_camera_control.on_control = false;
        // }
        // else
        // {
        //   //释放主相机控制权
        //   current_authority.store(ControlAuthority::Idle, std::memory_order_release);
        //   // //释放后视相机正在执行控制状态
        //   // back_camera_control.on_control = false;
        // }
    }

    /* name: target_callback
     * @brief：接收目标【装甲板】解算数据，得到【装甲板是谁的和信息头】，根据目标点距离近：stay；再根据装甲板信息判断是不是前哨，是就stay；不是就 pursuing
       @param：msg hnurm_interfaces::msg::Target（目标点数据）
     */
    void PubRobotStatus::target_callback(const hnurm_interfaces::msg::Target::SharedPtr msg)
    {
        RCLCPP_WARN(node_->get_logger(), "DETECTED !!!");
        // 获取机器人朝向（yaw角）
        tf2::Quaternion q(
            transform_m2bf.transform.rotation.x,
            transform_m2bf.transform.rotation.y,
            transform_m2bf.transform.rotation.z,
            transform_m2bf.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (target_data.target_distance < 3.5) // 发布远离点，找远离点被注释
        {
            // current_global_pose pub_pose = find_Away_point(yaw,send_data.target_distance);
            // geometry_msgs::msg::PoseStamped pose;
            // pose.header = msg->header;
            // pose.header.frame_id = "map";
            // pose.pose.position.x =  pub_pose.pose_x;
            // pose.pose.position.y = pub_pose.pose_y;
            // pose.pose.position.z = 0.0;
            // pose.pose.orientation.x = 0;
            // pose.pose.orientation.y = 0;
            // pose.pose.orientation.z = 0;
            // pose.pose.orientation.w = 1;
            // pursuit_pub_ ->publish(pose);
            // pursuit_type.store(PursuitType::Away);
            stay_ = true;
            pursuit_ = false;
            cruise_ = false;
        }
        else if (msg->id == "outpost")
        {
            stay_ = true;
            pursuit_ = false;
            cruise_ = false;
            return;
        }
        else
        {

            // 计算敌方目标点位置
            float pub_x = target_data.target_distance * std::cos(yaw) + current_pose_.pose_x;
            float pub_y = target_data.target_distance * std::sin(yaw) + current_pose_.pose_y;
            target_pose_.header = msg->header;
            target_pose_.header.frame_id = "map";
            target_pose_.pose.position.x = pub_x;
            target_pose_.pose.position.y = pub_y;
            target_pose_.pose.position.z = 0.0;
            target_pose_.pose.orientation.x = 0;
            target_pose_.pose.orientation.y = 0;
            target_pose_.pose.orientation.z = 0;
            target_pose_.pose.orientation.w = 1;
            // pursuit_pub_ ->publish(pose);
            // pursuit_type.store(PursuitType::Approach);
            stay_ = false;
            pursuit_ = true;
            cruise_ = false;
        }
    }

    /* name: global_pose_callback
     * @brief：处理机器人的全局位置信息，更新机器人状态并管理巡航目标点
     * @订阅话题：global_position_（默认：/global_costmap/published_footprint）
     * @param msg geometry_msgs::msg::PolygonStamped（包含多边形点集的 ROS 消息）
     */
    void PubRobotStatus::global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {

        // current_pose_.pose_x = msg->polygon.points[15].x;    //取published_footprint（数组中储存32个）中的一个作为当前位置的估计即可，后续如果有抖动的话再修改别的
        // current_pose_.pose_y = msg->polygon.points[15].y;
        current_pose_ = calculateAverage(*msg);
        //  RCLCPP_INFO(node_->get_logger(),"current_point: x=%f ,y=%f ,cruise_point: x=%f ,y=%f ",current_pose_.pose_x,current_pose_.pose_y,cruise_goals_[cruise_iterator.idx].pose_x,cruise_goals_[cruise_iterator.idx].pose_y);

        // cruise_iterator.former_is_reached = goal_checker(cruise_goals_[cruise_iterator.idx], current_pose_);
        // if (cruise_iterator.former_is_reached) {
        //     cruise_iterator.idx++;
        //     if (static_cast<std::size_t>(cruise_iterator.idx) >= cruise_goals_.size()) {
        //         cruise_iterator.idx = 0;
        //     }
        // }

        /*************************Cruise iterator Begin,巡航目标点管理*********************************** */
        /*          头文件：
                        struct current_cruise_goal
                        {
                            int idx;                // 当前巡航目标索引
                            bool former_is_reached; // 标记前一个目标是否已到达
                        };

                        current_cruise_goal cruise_iterator{0, false}; // 巡航目标迭代器，初始值为{0, false}
                        bool is_waiting_ = false;                      // 标记机器人是否在巡航点等待
                        double wait_duration_ = 2.0;                   // 巡航点间隔等待持续时间（到达一个点后等待几秒）（默认2秒）
                        rclcpp::Time wait_start_time_;                 // 等待开始时间
        */
        const bool is_reached = goal_checker(cruise_goals_[cruise_iterator.idx], current_pose_, 0.25);
        if (is_reached && !is_waiting_)
        {
            // 首次到达目标点，触发等待
            wait_start_time_ = node_->now();
            is_waiting_ = true;
            // RCLCPP_INFO(node_->get_logger(), "cruise goal is reached ,start waiting %.1f seconds...", wait_duration_);
        }
        // //超时异常检测，暂时不使用
        // if (!is_reached && !is_waiting_)
        // {
        //   timeout_start_time_ = node_ ->now();
        //   const auto elapsed_timeout_ = node_->now() - timeout_start_time_;
        //   if(elapsed_timeout_.seconds()>=timeout_)
        //   {
        //     //timeout update goal
        //     cruise_iterator.idx++;
        //     if (static_cast<std::size_t>(cruise_iterator.idx) >= cruise_goals_.size()) {
        //         cruise_iterator.idx = 0;
        //     }
        //     is_waiting_ = false;
        //     RCLCPP_INFO(node_->get_logger(), "time out ,ahead to the [IDX=%d]", cruise_iterator.idx);
        //   }
        // }

        if (is_waiting_)
        {
            const auto elapsed = node_->now() - wait_start_time_;
            if (elapsed.seconds() >= wait_duration_)
            {
                // finished waiting
                cruise_iterator.idx++;
                if (static_cast<std::size_t>(cruise_iterator.idx) >= cruise_goals_.size())
                {
                    cruise_iterator.idx = 0;
                }
                is_waiting_ = false;
                RCLCPP_INFO(node_->get_logger(), "finished waiting ,ahead to the [IDX=%d]", cruise_iterator.idx);
            }
        }
        cruise_iterator.former_is_reached = is_reached && !is_waiting_;
        /*************************Cruise iterator End*********************************** */

        /***********************************特殊区域检测 start**********************************/
        visualizeClosestArea();
        std_msgs::msg::Bool isin_spa;
        if (isinSpecialArea(current_pose_, closest_area_))
        {
            is_in_special_area = true;
            // std::cout<<"in special area: "<<closest_area_<<std::endl;
        }
        else
            is_in_special_area = false;
        update_poses();
        isin_spa.data = is_in_special_area;
        in_special_area_pub_->publish(isin_spa);
        /***********************************特殊区域检测 end**********************************/
    }
    /***************************************回调函数 end*****************************************************/

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_behavior_trees2::PubRobotStatus>("PubRobotStatus");
}