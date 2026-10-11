#ifndef CHARGE_CONTROLLER_H
#define CHARGE_CONTROLLER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <cmath>
#include <atomic>
#include <limits>
#include <fstream>
#include <tf/transform_broadcaster.h>

// ROS相关
#include <ros/ros.h>
#include "robot_msgs/Voice.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "sensor_msgs/LaserScan.h"
#include "tag_odom_location.h"
#include "robot_msgs/LeavePileChange.h"

#include "common/common_params.h"
#include "common/common_algorithm.h"
#include "collision_detector/charge_obstacle_detection.h"
namespace charge_controller
{
// 定义对桩部分状态机
typedef enum
{
    AIM_INIT,        // 初始化对桩操作
    CHARGE_MAP_INIT, // 充电局部地图构建初始化
    CONTROL_TO_GOAL, // 控制去目标点
    CHARGE_CHECK,    // 检查充电
    AIM_FINISH,      // 最终结束
    AIM_ERROR,
} AimState;

// 定义离桩部分状态机
typedef enum
{
    LEAVE_INIT,     // 初始化离桩
    LEAVE_MAP_INIT, // 初始化离桩地图
    LEAVE_DIS,      // 离桩弹出
    LEAVE_ROT_TO_FRONT, // 离桩后旋转到正前方
    LEAVE_FINISH,   // 完成离桩
    LEAVE_ERROR     // 离桩失败
} LeaveState;

// 旋转后进行直线行走操作
typedef enum
{
    GET_DE_POSE,
    ROT_1,
    GO_STRAIGHT,
    ROT_2,
    BACK
} OdomControlState;

// 旋转后进行直线行走操作
typedef enum
{
    MOVE_TO_TEMP_GOAL,
    ROT_TO_GOAL_DIR,
    MOVE_TO_LINE_GOAL,
    ROT_TO_FINAL_GOAL,
    MOVE_TO_FINAL_GOAL,
    MOVE_SUCCESS
} PIDControlState;

class ChargeController
{
public:
    /**********************************************************
     * Description: 充电控制器含参构造函数
                    初始化相关参数
     * Input:    n: ros节点句柄
     *           default_config: 定位与控制器相关参数
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    ChargeController(ros::NodeHandle n, const LocalPlannerConfig &default_config);

    /**********************************************************
     * Description: 充电控制器析构函数
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    ~ChargeController();

    /**********************************************************
     * Description: 发布零速
     * Input:    无
     * Output:   cmd_vel: 发布的零速度指令
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void PubZeroVelocity(geometry_msgs::Twist &cmd_vel);

    /**********************************************************
     * Description: 检查是否超时
     * Input:    无
     * Output:   无
     * Return:   true: 表示超时
     *           false: 表示未超时
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    bool CheckDelayTime();

    /**********************************************************
     * Description: 执行对桩操作
     * Input:    无
     * Output:   cmd_vel: 发布的速度指令
     * Return:   AimState: 对桩状态
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    AimState RunAimPile(geometry_msgs::Twist &cmd_vel);

    /**********************************************************
     * Description: 执行离桩操作
     * Input:    无
     * Output:   cmd_vel: 发布的速度指令
     * Return:   LeaveState: 离桩状态
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    LeaveState RunLeavePile(geometry_msgs::Twist &cmd_vel);

    /**********************************************************
     * Description: 重置对桩参数
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void ResetAimPileParams();

    /**********************************************************
     * Description: 重置离桩参数
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    void ResetLeavePileParams();
    
    /**********************************************************
     * Description: 初始化充电地图
     * Input:    无
     * Output:   无
     * Return:   true: 初始化充电地图成功
     *           false: 初始化充电地图失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    bool InitChargeMap();

private:
    /**********************************************************
     * Description: 传感器感知障碍数据回调
     * Input:    msg: sensor_msgs::PointCloud2::ConstPtr类型的数据消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2026/01/29        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    void SensorDataCB(const sensor_msgs::PointCloud2::ConstPtr &msg);                                                                                                                                    

    /**********************************************************
     * Description: apriltag识别消息回调
     * Input:    msg: apriltag_ros::AprilTagDetectionArray::ConstPtr类型的消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    void TagsCallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);                                                                                                                       
    
    /**********************************************************
     * Description: 反光板识别消息回调，接收识别到的结果
     * Input:    msg: std_msgs::Float32MultiArray类型的消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    void ReflectorCallBack(const std_msgs::Float32MultiArray msg);                                                                                                                                     

    /**********************************************************
     * Description: 磁导航传感器识别消息回调，接收原始数据
     * Input:    msg: geometry_msgs::PointStamped::ConstPtr类型的消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    void MagneticCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);

      /**********************************************************************
     * Description: 订阅充电是否成功的回调函数
     * Input:     msg: 接收到的充电是否成功的消息
     * Output:    无
     * Return:    无
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/11/11      V1.0      邓同兴           代码初次注释
     **********************************************************************/
    void ChargeResultCallback(const std_msgs::Int32::ConstPtr& msg);

    /**********************************************************
     * Description: 订阅是否为第一次充电的回调函数
     * Input:     msg: 接收到的是否为第一次充电的消息
     * Output:    无
     * Return:    无
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/11/11      V1.0      邓同兴           代码初次注释
     **********************************************************/
    void FirstFlagCallback(const std_msgs::Bool::ConstPtr& msg); // 20251223 dtx
    
    /**********************************************************
     * Description: 检查apriltag码位姿状态是否有效
     * Input:    T_cam_to_left_tag: 左侧apriltag码在相机坐标系下的位姿
     *           T_cam_to_middle_tag: 中间apriltag码在相机坐标系下的位姿
     *           T_cam_to_right_tag: 右侧apriltag码在相机坐标系下的位姿
     *           valid_count: 位姿状态有效的个数
     * Output:   T_baselink_to_tag_reference: apriltag码在基坐标系下的位姿
     * Return:   true: 位姿状态有效
     *           false: 位姿状态无效
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    bool CheckTagPoseValid(Eigen::Matrix4d T_cam_to_left_tag, 
        Eigen::Matrix4d T_cam_to_middle_tag, 
        Eigen::Matrix4d T_cam_to_right_tag, 
        int &valid_count, 
        Eigen::Matrix4d &T_baselink_to_tag_reference); 
    
    /**********************************************************
     * Description: 查找TF树中两个坐标系之间的位姿关系
     * Input:    parent_link: 父坐标系名
     *           child_link: 子坐标系名 
     * Output:   pose: 转换后的位姿
     * Return:   true: 转换成功
     *           false: 转换失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    bool LookupTfPose(std::string parent_link, std::string child_link, RPYPose &pose);
    
    /**********************************************************
     * Description: 获取当前位姿
     * Input:    无
     * Output:   input_data: 更新包含里程计，apriltag，反光板计算得到的当前位姿
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    bool GetLocationInputDate(TagLocation::LocationInput &input_data);
    
    /**********************************************************
     * Description: 可视化TF变换
     * Input:    transform_matrix: 子坐标系在父坐标系下的转换矩阵
     *           parent_link: 父坐标系名
     *           child_link: 子坐标系名
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    void VisualizeTf(Eigen::Matrix4d &transform_matrix, std::string parent_link, std::string child_link);
    
    /**********************************************************
     * Description: 计算移动速度
     * Input:    central_axis_angle: 距离中心的角度误差
     *           distance_to_goal: 离目标点的距离
     * Output:   linear_velocity: 计算得到的线速度
     *           angle_velocity: 计算得到的角速度
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    void ComputeMoveVelocity(double central_axis_angle, double distance_to_goal, 
        double &linear_velocity, double &angle_velocity);
    
    /**********************************************************
     * Description: 移动到目标点
     * Input:    无
     * Output:   cmd_vel: 发布的速度指令
     * Return:   true: 成功运动到目标点
     *           false: 运动到目标点失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    bool MoveToGoal(geometry_msgs::Twist &cmd_vel);
    
    /**********************************************************
     * Description: 使用里程计控制到达目标点
     * Input:    baselink_pose_in_map: 基坐标系在参考坐标系下的位姿
     * Output:   cmd_vel: 发布的速度指令
     * Return:   true: 成功到达目标点
     *           false: 到达目标点失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    bool MoveToGoalWithOdom(RPYPose &baselink_pose_in_map, geometry_msgs::Twist &cmd_vel);
    
    /**********************************************************
     * Description: 使用PID控制到达目标点
     * Input:    T_reference_map_to_baselink: 基坐标系在参考坐标系下的转换矩阵
     * Output:   cmd_vel: 发布的速度指令
     * Return:   true: 成功到达目标点
     *           false: 到达目标点失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    bool MoveToGoalWithPID(Eigen::Matrix4d &T_reference_map_to_baselink, geometry_msgs::Twist &cmd_vel);
    
    /**********************************************************
     * Description: 控制进行原地旋转
     * Input:    theta: 需要旋转的角度(弧度)
     *           w_z: 旋转的角速度
     * Output:   cmd_vel: 发布的速度指令
     * Return:   true: 旋转完成
     *           false: 旋转失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    bool Rot(double theta, double w_z, geometry_msgs::Twist &cmd_vel);
    
    /**********************************************************
     * Description: 控制直线行走
     * Input:    dist: 需要直行的距离
     *           v_x: 直行的线速度
     * Output:   cmd_vel: 发布的速度指令
     * Return:   true: 直行完成
     *           false: 直行失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    bool GoDist(double dist, double v_x, geometry_msgs::Twist &cmd_vel);
    
    /**********************************************************
     * Description: 根据里程计获取基坐标系在参考坐标系下的位姿，控制运动
     * Input:    baselink_pose_in_map: 基坐标系在参考坐标系下的位姿
     * Output:   无
     * Return:   OdomControlState: 控制运动的状态
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    OdomControlState GetDePose(RPYPose baselink_pose_in_map);
    
    
    /**********************************************************
     * Description: 发布语音
     * Input:    name: 语音文件名
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    void PublishVoice(std::string name);

    /**********************************************************
     * Description: 发布倒桩相机状态
     * Input:    state: 1->打开相机 0->关闭相机
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    void PublishCamState(int state);

    /**********************************************************
     * Description: 离桩距离修改服务
     * Input:    req: 离桩距离修改服务请求
     * Output:   res: 离桩距离修改服务响应
     * Return:   true: 服务调用成功
     *           false: 服务调用失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    bool LeavepileChangeReceive(robot_msgs::LeavePileChange::Request &req, 
        robot_msgs::LeavePileChange::Response &res);
    
public:
    ros::Publisher cam_state_pub_;
    ros::Publisher voice_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher path_pub_;

    OdomControlState move_step_;
    PIDControlState pid_step_;
    AimState aimpile_step_;
    LeaveState leavepile_step_;
    std::atomic<bool> first_flag_; // 时间记录
    std::atomic<int32_t> charge_flag_; // 判断充电是否成功标志
    sensor_msgs::PointCloud2::ConstPtr sensor_data_msg_;
    std::string failed_id_;
    // 20251022 lzw 离桩坐标系距离基坐标系的距离
    // 大于leave_pile_avoid_dis时 咖啡车开启避障
    // 距离值传出到power_manage 大于close_relay_dis_时，关闭继电
    // 在离桩完成或离桩失败时置0
    double leavepile_map_to_baselink_dis_; 
    double aimpile_map_to_baselink_dis_;

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_;             // tf树的监听
    tf::TransformBroadcaster broadcaster_; // tf树的广播
    TagLocation::Params locate_params_;

    ros::Subscriber apriltag_sub_;
    ros::Subscriber reflector_sub_;
    ros::Subscriber sensor_data_sub_; // 使用感知障碍替代激光雷达数据进行障碍判断
    ros::Subscriber magnetic_sub_; // 20250325 liuzhiwen 磁导航数据订阅
    ros::Subscriber charge_result_sub_; //20251223 dtx
    ros::ServiceServer leavepile_change_srv_; // 20250510 liuzhiwen 更改离桩距离服务

    // 反光板的值
    float reflector_x_left_;
    float reflector_y_left_;
    float reflector_x_right_;
    float reflector_y_right_;
    float reflector_x_left_door_;
    float reflector_y_left_door_;
    float reflector_x_right_door_;
    float reflector_y_right_door_;

    TagLocation::SensorPose reflector_pose_in_baselink_;
    TagLocation::SensorPose apriltag_pose_in_baselink_;
    TagLocation::SensorPose baselink_pose_in_odom_;

    int time_count_;
    std::mutex m_reflector_pose_;
    std::mutex m_tag_pose_;
    std::mutex m_sensor_data_;
    std::mutex m_tag_pixel_pose_;
    std::mutex m_magnetic_data_; // 20250325 liuzhiwen 磁导航数据互斥量
    std::mutex m_leavepile_change_; // 20250510 liuzhiwen 离桩距离修改

    std::pair<bool, float> magnetic_data_; // 20250325 liuzhiwen 磁导航数据
    std::chrono::steady_clock::time_point time_begin_;
    std::chrono::steady_clock::time_point time_start_;
    std::shared_ptr<TagLocation::TagOdomLocation> tag_locater_;
    std::shared_ptr<CHARGE_ODT::ObstacleDetect> obstacle_detecter_;
    const LocalPlannerConfig &default_config_;

    int sleep_times_;
    bool check_flag_;
    bool go_dis_flag_; // 20241019 liuzhiwen 行走距离标志

    double yaw_delta_total_;
    double current_yaw_;
    double yaw_delta_;
    double current_x_;
    double current_y_;
    double first_rot_angle_;
    double go_straight_dist_;
    double second_rot_angle_;
    
    nav_msgs::Path path_msg_;
    Eigen::Matrix4d T_leavepile_map_to_odom_;
    Eigen::Matrix4d T_baselink_to_tag_reference_;
    bool tag_update_flag_;
    bool reflector_update_flag_;
    std::atomic<bool> use_door_reflector_;
    double leave_pile_dist_;
    std::mutex charge_mutex_;
};
} // namespace charge_controller
#endif
