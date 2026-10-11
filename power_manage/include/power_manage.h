#ifndef SRC_POWER_MANAGE_H
#define SRC_POWER_MANAGE_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "serialserver.h"
#include "serialport.h"
#include "yaml-cpp/yaml.h"
#include "nav_msgs/SetMap.h"
#include "utils.h"
#include "robot_msgs/Peripheral.h"

// ros相关头文件
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h> 
#include <std_msgs/Int32.h> 
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_msgs/PowerCommond.h"
#include "robot_msgs/Voice.h"
#include "robot_msgs/LeavePileChange.h"
#include "geometry_msgs/PointStamped.h"
#include "openssl_utils.h"

// qt相关库
#include <QApplication>
#include <QThread>
#include <QObject>
#include <QTimer>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonParseError>
#include <QJsonDocument>
#include <QDebug>
#include <QString>
#include <map>
#include "robot_msgs/Switch.h"
#include "robot_msgs/ElevatorPoint.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include "robot_msgs/MapManage.h"
#include "robot_msgs/DataBase.h"
#include "robot_msgs/Web.h"
#include <robot_msgs/TaskAction.h>
#include <robot_msgs/TaskService.h>

using namespace serial_server;

class PowerManage : public QThread
{
    Q_OBJECT // Q_OBJECT宏，允许类中使用信号和槽的机制

public: 
    /**********************************************************
     * Description: 电源管理类构造函数，初始化相关成员变量
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/   
    PowerManage();

    /**********************************************************
     * Description: 电源管理类析构函数
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/       
    ~PowerManage();

private:
    /**********************************************************************
     * Description: 基于动作的目标点完成执行函数
     * Input:     state：当前状态
     *            result：无用
     * Output:    无
     * Return:    无
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/12/23      V1.0      dtx           代码初次注释
     **********************************************************************/
    void DoneCb(const actionlib::SimpleClientGoalState& state, const robot_msgs::TaskResultConstPtr& result);
    /**********************************************************************
     * Description: 打印日志
     * Input:     无
     * Output:    无
     * Return:    无
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/12/23      V1.0      dtx           代码初次注释
     **********************************************************************/
    void ActiveCb();
    /**********************************************************************
     * Description: 反馈回调函数
     * Input:     feedback：反馈信息
     * Output:    无
     * Return:    无
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/12/23      V1.0      dtx           代码初次注释
     **********************************************************************/
    void FeedbackCb(const robot_msgs::TaskFeedbackConstPtr& feedback);
    /**********************************************************************
     * Description: 设置充点任务目标，对桩或离桩
     * Input:     str：目标字符串
     * Output:    无
     * Return:    无
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/12/23      V1.0      dtx           代码初次注释
     **********************************************************************/
    void SetChargingGoal(const std::string& str);
   
    /**********************************************************************
     * Description: 异常处理函数
     * Input:     msg：异常出现在对桩过程还是离桩过程
     * Output:    无
     * Return:    异常处理是否成功
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/12/23      V1.0      dtx           代码初次注释
     **********************************************************************/
    bool ExceptionHandle(const std::string &msg);
   
    /**********************************************************************
     * Description: 向action服务端发送暂停任务请求
     * Input:     actionIndex: 任务索引
     * Output:    无
     * Return:    发送请求是否成功
     * Modify Date    Version    Author          Modification
     * -----------------------------------------------
     * 2025/12/23      V1.0      dtx           代码初次注释
     **********************************************************************/
    bool sendTaskCommand(const int &actionIndex);

    /**********************************************************
     * Description: 电源关闭功能
     * Input:    close_time: 相应时间后关机
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void PowerCloseFun(const int close_time);

    /**********************************************************
     * Description: 电源重启功能
     * Input:    restart_time: 相应时间后重启
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void PowerRestartFun(int restart_time);

    /**********************************************************
     * Description: 日志发布
     * Input:    data: 待发布的日志信息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void LogPub(std::string data);

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
     * Description: 发布电机状态(对桩完成时)
     * Input:    motor_control_mode: 电机控制模式 0->电机限流 1->电机失能
     *           set_motor_current_: 是否开启电机限流
     *           set_motor_enable_: 是否开启电机失能
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void PublishStartMotorState(int motor_control_mode, bool set_motor_current_, bool set_motor_enable_);
    
    /**********************************************************
     * Description: 发布电机状态(离桩开始前)
     * Input:    motor_control_mode: 电机控制模式 0->电机限流 1->电机失能
     *           set_motor_current_: 是否开启电机限流
     *           set_motor_enable_: 是否开启电机失能
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/     
    void PublishEndMotorState(int motor_control_mode, bool set_motor_current_, bool set_motor_enable_);
    
    /**********************************************************
     * Description: 控制板信息回调
     * Input:    msg: std_msgs::String类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/     
    void ControlCallBack(const std_msgs::String &msg); 

    /**********************************************************
     * Description: 设备急停信息回调
     * Input:    msg: std_msgs::UInt16类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/     
    void EmergencyCallBack(const std_msgs::UInt16 &msg); 
    
    /**********************************************************
     * Description: 控制外设以及开关机电源开关服务
     * Input:    request: 服务消息请求
     * Output:   response: 服务消息响应
     * Return:   true: 服务调用成功
     *           false: 服务调用失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/     
    bool ManageCmdReceive(robot_msgs::PowerCommond::Request &request, 
        robot_msgs::PowerCommond::Response &response);
    
    /**********************************************************
     * Description: 解析字符串
     * Input:    input_string: 待解析的字符串
     * Output:   无
     * Return:   std::map<std::string, float>: 获取所有字符和对应的值
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/     
    std::map<std::string, float> ParseString(const std::string &input_string);

    /**********************************************************
     * Description: 将十六进制字符串转换为QByteArray字节数组，每两个字符转换为一个字节
     * Input:    str: 十六进制字符串
     * Output:   无
     * Return:   QByteArray: 转换后的字节数组
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    QByteArray Hexstring2QByteArray(QString str);
	
    /**********************************************************
     * Description: 时间任务管理 实现对时 设置任务和关闭电源
     * Input:    无 
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/      
    void TimeTaskManage();

    /**********************************************************
     * Description: 充电重试线程
     * Input:    无 
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void RechargeThread();

    /**********************************************************
     * Description: 发布速度
     * Input:    无 
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void PublishCmdVel(); 
    
    /**********************************************************
     * Description: 电机电流数据回调
     * Input:    msg: std_msgs::Float64MultiArray::Ptr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/      
    void MotorCurrentCallBack(const std_msgs::Float64MultiArray::Ptr &msg);

    /**********************************************************
     * Description: 灯光控制消息回调
     * Input:    msg: std_msgs::Int8类型消息 0->开灯 1->关灯
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void LampCtrl(const std_msgs::Int8 msg);

    /**********************************************************
     * Description: 根据分割副分割字段
     * Input:    str: 需要分割的字符串  split:分割符
     * Output:   rst 分割后的字符串队列
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          gaojunlin    代码初次注释  
     **********************************************************/  
    void StrSplit(std::string str, const char split, std::vector<std::string> &rst);

    /**********************************************************
     * Description: 倒桩相机状态回调
     * Input:    msg: std_msgs::Int8类型消息
     *           0 -> 相机状态正常 
     *           1 -> 倒桩相机掉线0.2s 
     *           2 -> 倒桩相机掉线30s
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/      
    void CamCallBack(const std_msgs::Int8 msg);

    /**********************************************************
     * Description: 对桩起始位置状态回调
     * Input:    msg: std_msgs::String::ConstPtr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void PileStartPoseCallBack(const std_msgs::String::ConstPtr &msg);

    /**********************************************************
     * Description: 订阅电机参数设置消息
     * Input:    msg: std_msgs::String::ConstPtr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void UIMotorConfigCallBack(const std_msgs::String::ConstPtr &msg_p);

    /**********************************************************
     * Description: 离桩后矫正位姿
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void CorrectPose();

    /**********************************************************
     * Description: 托盘IO状态回调
     * Input:    msg: std_msgs::String::ConstPtr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void PushIOCallBack(const std_msgs::String::ConstPtr &msg);

    /**********************************************************
     * Description: 软急停信号回调
     * Input:    msg: std_msgs::Bool::ConstPtr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/12/23        v1.0          dtx    代码初次注释  
     **********************************************************/  
    void MotorJiTingCallBack(const std_msgs::Bool::ConstPtr &msg);
   
    /**********************************************************
     * Description: 关闭IO控制
     * Input:    无 
     * Output:   无
     * Return:   true: 关闭IO控制成功
     *           false: 关闭IO控制失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/      
    bool CloseIO();

    /**********************************************************
     * Description: 打开IO控制
     * Input:    无 
     * Output:   无
     * Return:   true: 打开IO控制成功
     *           false: 打开IO控制失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    bool OpenIO();

    /**********************************************************
     * Description: 定位漂移回调
     * Input:    msg: std_msgs::String::ConstPtr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2026/02/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void MapDriftCallBack(const std_msgs::Bool::ConstPtr &msg);

    /**********************************************************
     * Description: 桩前位置矫正
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2026/02/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void StationPoseCorrect();

    /**********************************************************
     * Description: 查询tf获取位置
     * Input:    parent_link: 父坐标系
     *           child_link: 子坐标系
     * Output:   pose: 查找到的位置
     * Return:   bool: true 查找到对应的tf false 未查找到对应的tf
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2026/02/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    bool LookupTfPose(std::string parent_link, std::string child_link, RPYPose &pose);
    
    /**********************************************************
     * Description: 调用set_map进行位姿矫正
     * Input:    msg_poseinit: 位姿信息
     * Output:   无
     * Return:   bool: true 矫正位姿成功 false 矫正位姿失败并记录日志
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2026/03/20        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    bool InitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg_poseinit);

    /**********************************************************
     * Description: 订阅io_data数据 处理继电器控制反馈状态
     * Input:    msg: std_msgs::String::ConstPtr类型消息
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2026/03/27        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    void IoDataCallBack(const std_msgs::String::ConstPtr &msg);

protected:
    /**********************************************************
     * Description: 倒桩任务刷新线程
     * Input:    无 
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void run();

private slots:
    /**********************************************************
     * Description: 电源数据发布槽函数（旧协议 V1）
     * Input:    无 
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SlotPowerDataPub(); 

    /**********************************************************
     * Description: 电源数据发布槽函数（新协议 V2）
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SlotPowerDataPubNew();

    /**********************************************************
     * Description: 发布日志槽函数
     * Input:    data: 待发布日志数据
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SlotLogSerPub(QString data);

    /**********************************************************
     * Description: 电源板状态发布槽函数
     * Input:    msg_type: 消息类型
     *           msg_data: 消息数据
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SlotPowerStatus(QString msg_type,QString msg_data);

signals:
    /**********************************************************
     * Description: 发送控制命令信号
     * Input:    ser_cmd: 控制命令
     * Output:   无
     * Return:   
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SigSendCmd(int ser_cmd);

    /**********************************************************
     * Description: 电源管理控制命令信号
     * Input:    ser_cmd: 电源管理控制命令
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SigPowerCmd(int ser_cmd);

    /**********************************************************
     * Description: 控制命令发布信号
     * Input:    msg_type: 消息类型
     *           msg_data: 消息数据
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SigPubCmd(QString msg_type, QString msg_data);

    /**********************************************************
     * Description: 充电阈值控制信号
     * Input:    ser_cmd: 电量阈值
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/  
    void SigChargeLimt(int ser_cmd); 

private:
    ros::NodeHandle nh_;
    
    ros::Publisher power_data_pub_;
    ros::Publisher log_pub_;
    ros::Publisher serial_status_pub_;
    ros::Publisher serial_error_pub_;
    ros::Publisher verson_pub_;
    ros::Publisher voice_pub_;
    ros::Publisher locate_status_pub_;
    ros::Publisher set_motor_current_pub_;
    ros::Publisher set_motor_enable_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher magnetic_data_pub_;
    ros::Publisher charge_success_pub_;      // 20251223 添加充电成功发布话题 dtx
    ros::Publisher feedback_pub_; // 20251223 dtx 
	ros::Publisher cmd_sensor_pub_;
    ros::Publisher station_localization_failed_pub_; // 发布桩前定位失败
    ros::Publisher apriltag_detect_pub_; // apriltag识别话题 1 开启识别 0 结束识别

    ros::Subscriber power_cmd_sub_;
    ros::Subscriber fault_poweroff_sub_;
    ros::Subscriber restart_system_sub_;
    ros::Subscriber motor_current_sub_;
    ros::Subscriber lamp_sub_;
    ros::Subscriber control_version_sub_;
    ros::Subscriber emergency_stop_sub_;   // 内部使用，获取电源管理的控制命令
    ros::Subscriber cam_err_sub_; // 20250428 lzw 对桩相机掉线话题订阅
    ros::Subscriber pile_start_pose_sub_;
    ros::Subscriber push_io_sub_; // 20250913 lzw 订阅托盘状态
    ros::Subscriber sub_motor_config_cmd_; //订阅电机配置参数指令
    ros::Subscriber localization_failed_sub_; // 监测定位是否失败
    ros::Subscriber map_sub_;
    ros::Subscriber current_map_sub_;
    ros::Subscriber motor_jiting_sub_; // 20251223 dtx 订阅软急停信号
	ros::Subscriber io_data_sub_;
    ros::ServiceServer power_data_server_;  // 电源管理开关服务器

    ros::ServiceClient db_client_;
    ros::ServiceClient originpoint_client_;
    ros::ServiceClient push_prohibition_client_; // 20250913 lzw 禁止托盘外设
    ros::ServiceClient set_map_client_; // 20260320 lzw 矫正位姿服务客户端
	ros::ServiceClient control_relay_client_; // 20260327 lzw 控制继电器客户端
    ros::ServiceClient task_service_client_; //20251223 dtx
    std::unique_ptr<actionlib::SimpleActionClient<robot_msgs::TaskAction>> ac_client_;// 202512023 dtx
	
    
    QTimer *pPower_pub_timer_; // 电量发布时间
    QString powerboaderinfo_; // 版本信息
    QString power_info_; // 上传状态信息

    SerialServer *pSerialserver_;   
	SerialPort *pser_; // 串口类声明
    
    int battery_low_threshold_;
    int first_charge_success_battery_; // 每次对桩时刚充电成功时的电量值
    int motor_control_mode_;  // lzw 20240808 发布电机状态
    double motor_current_max_threshold_;
    double motor_current_min_threshold_; // lzw 20240829 电流阈值
    std::vector<double> motor_current_data_{0, 0}; //lzw 20240829 接收电机电流数据
    
    std::mutex m_charge_task_;
    std::mutex m_set_param_;
    std::mutex m_motor_current_; 
    
    // 在构造函数中进行初始化
    bool set_param_flag_;
    bool voice_pub_flag_;
    bool charge_success_;
    bool charge_failed_;
    bool deploy_on_pile_;
    bool close_flg_;
    bool pub_start_log_flag_;
    bool pub_free_flag_;
    bool pub_log_flag_;
    bool motor_current_flag_; // 20240829 lzw 添加发布微小速度的标志位 motor_current_flag_
    bool is_first_charge_success_flag_; // 每次对桩成功时刻标志位
    RobotState charge_task_; // 机器人状态 
    int fault_timer_;
    int battery_high_threshold_;
    int power_serial_version_;
    u_short start_log_flag_; //20250113 lzw 设置该标志位 保证在ROBOT_START下切换时 只记录最近一次状态到日志

    // fangyao 20240829 新协议
    enum
    {
        TIME_WAIT,
        TIMING,
        TASK_SET,
        POWEROFF,
        FINISH
    };
    //20251223 dtx 任务状态
    enum class task_state
    {
        none,
        successed,
        aborted
    };

    bool charge_full_{false}; // 电量满
    int timetask_index_{TIME_WAIT};
    bool timetask_active_{false}; // 定时任务设置激活
    int wakeuptime_{0}; // 定时任务休眠时间 单位 秒
    std::string strWakeupTime_; // 唤醒开始时间
    bool redo_flag_{true}; // 重试标志
    bool serial_status_{false};
    bool communication_status_{false};
    
    // 20251223 dtx 
    int count_;
    bool charge_success_flag_{false}; 
    bool pause_{false};
    bool has_exception_{false}; // 是否有异常行为发生
    std_msgs::String aim_pile_status_; 
    std::atomic<double> leavepile_dis_; // 离桩距离
    std::atomic<int> status_;  // 任务状态
    std::atomic<bool> motor_jiting_flag_{false}; // 软急停标志位
    std::string failed_id_;  // 任务失败的id
    std::mutex failed_id_mtx_;

    // lzw add
    std::string log_path_; // 日志文件路径
    std::string car_type_;  //车子类型
    geometry_msgs::Twist vel_; // 20240829 lzw 发布速度
    std::mutex m_recharge_; // 20250928 lzw 重试充电
    std::mutex m_push_io_; 
    double close_relay_dis_; // 关闭继电器距离
    bool pole_direction_; // 电极朝向
    int recharge_num_; // 20250913 lzw 重试充电次数
    int recharge_over_time_; // 20250928 lzw 充电重试线程超时退出时间 单位秒
    int judge_charge_mode_; // 20251022 lzw 判断充电成功模式
    bool rot_flag_;
    int battery_drop_interval_; // 在桩电量却下掉阈值
    int charge_station_type_; // 充电桩类型
    int charge_success_to_failed_wait_time_; // 从充电成功跳转到充电失败的等待时间
    
    Eigen::Matrix4d T_charge_in_origin_; // 充电坐标系在原点坐标系下的位姿
    double x_axis_position_threshold_; // 基坐标在充电坐标系下x轴范围内开始矫正
    double y_axis_position_threshold_; // // 基坐标在充电坐标系下y轴范围内开始矫正
    LocationMode location_mode_enum_; // 桩前定位模式 0 不开启 1 只使用反光板 2 只使用二维码 3 都使用
    tf::TransformListener tf_;

    bool set_motor_current_{true};
    bool set_motor_enable_{true};
    bool is_relay_{true}; // 20250122 lzw 关闭继电器标志位 true: 未关闭 false: 关闭
    bool recharge_over_{false};
    bool control_pallet_{false}; // 20260324 lzw 是否控制托盘 咖啡车控 其他车型不控
    bool cmd_push_flag_{false}; // 20250916 lzw 禁用后power_manage内部下降托盘服务 确保只下降一次
    bool push_prohibition_flag_{false}; // 20250916 lzw 禁用托盘服务标志 确保只禁用一次
    bool push_io_status_{false}; // 20250916 lzw 托盘状态
    bool io_log_flag_{false}; // 20250924 lzw 托盘服务日志 确保日志只写一次
    bool motor_status_{false}; // 20241203 lzw 对桩失败时只发布一次限流
    bool close_relay_status_{false}; // 20250213 lzw 离桩成功或者失败只发一次关闭继电器
    bool finish_station_localization_{false}; // 完成桩前定位
    std::atomic<bool> monitor_usb_cam_{false}; // 20250815 lzw 初始化为false
    std::atomic<bool> station_localization_failed_flag_{true}; // 20260212 lzw 默认桩前定位失败
    std::atomic<bool> localization_failed_flag_{false}; // 20260212 lzw 默认导航定位成功
    std::atomic<bool> apriltag_detect_start_{false};
    std::atomic<task_state> task_state_{task_state::none};//20251223 dtx 任务状态 
	std::atomic<bool> station_relay_status_{false}; // 20260327 lzw 充电桩继电器反馈状态 1 吸合 0 断开
	
    // c_add
    std::string default_map_;
    std::string pile_start_pose_;

    enum new_power_manage_cmd
    {
        PeripheralOn_,
        PeripheralOff_,
        robot_restart,
        motor_poweron,
        motor_poweroff,
        cancel_waring,
        aim_pile,
        leave_pile,
        lamp_close,
        lamp_open,
        open_charge,
        open_pollution,
        close_pollution,
        open_coverLock,
        close_coverLock,
        LiftLampOpen,
        RightLampOpen,
        LampClose,
        LampOpen,
        CleanBrushOn,
        CleanBrushOff,
        WashBrushOn,
        WashBrushOff,
        Battery_Cal,
        QuerryBrushErrCode,//查询故障码
        ClearCleanBrushErr,//清除扫地刷故障码
        ClearWashBrushErr//清除洗地刷故障码
    };

    enum old_power_manage_cmd
    {
        PeripheralOn,
        PeripheralOff,
        StartCharge,
        EndCharge,
        ReadyCharge,
        FinishCharge,
        ReadVersion,
        ReadControlVersion,
        lamp_close_V1,
        lamp_open_V1,
        BatteryCal
    };

    enum power_close_cmd
    {
        PowerClose,
        PowerOpen,
        DetermineTime,
        CancelAllTask,
        PerformAllTasks
    }; // 电源管理控制命令
};

#endif // SRC_POWER_MANAGE_H
