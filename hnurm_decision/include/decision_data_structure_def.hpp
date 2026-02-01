#ifndef decision_data_structure_def_HPP_
#define decision_data_structure_def_HPP_

#include "hnurm_interfaces/msg/vision_recv_data.hpp"

namespace hnurm_behavior_trees2{
    // 机器人map上坐标
    struct GlobalPose{
        float pose_x;
        float pose_y;
    };

    // 机器人位姿，猜测待删改：二维地图上应该是只有yaw的。roll和pitch为0
    struct PoseRPY
    {
        float roll;
        float pitch;
        float yaw;
    };

    // 裁判系统返回状态，主要关注当前血量和游戏进度和yaw角度
    struct RobotRefereeStatus
    {
        float current_hp;
        float game_progress;
        PoseRPY rpy;
    };

    // 敌方目标机器人状态和距离，接收上位机传给下位机话题，msg类型在interfaces里
    struct TargetData
    {
        bool target_state;     // 未知.猜测：0：失去目标 1：检测到， 2：开火
        float target_distance; // 直接拿过来 
    };

    // 不确定，貌似没用，没有实例化对象：存储参考偏航角控制数据（参考偏航角、是否处于控制状态）
    struct referen_yaw_control
    {
        double reference_yaw_;
        bool on_control;
    };

    // 串口数据，主要是为了存储上一帧的数据，用于计算目标距离
    struct SeriaData
    {
        hnurm_interfaces::msg::VisionRecvData data;       
        bool is_init = false;
    };
}

# endif