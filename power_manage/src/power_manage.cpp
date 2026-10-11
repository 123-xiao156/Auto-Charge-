#include <nlohmann/json.hpp>
#include "power_manage.h"

char g_charge_close_timer_A = 0x00; // 定义全局变量并初始化
char g_charge_close_timer_B = 0x00; // 定义全局变量并初始化
char g_charge_close_timer_C = 0x00; // 定义全局变量并初始化
char g_charge_close_timer_D = 0x00; // 定义全局变量并初始化

char g_charge_open_timer_A = 0x00; // 定义全局变量并初始化
char g_charge_open_timer_B = 0x00; // 定义全局变量并初始化
char g_charge_open_timer_C = 0x00; // 定义全局变量并初始化
char g_charge_open_timer_D = 0x00; // 定义全局变量并初始化

char g_current_timer_A = 0x00;     // 定义全局变量并初始化
char g_current_timer_B = 0x00;     // 定义全局变量并初始化
char g_current_timer_C = 0x00;     // 定义全局变量并初始化
char g_current_timer_D = 0x00;     // 定义全局变量并初始化

int g_cleanBrush_level = 1;        //扫地刷电机等级分为1/2/3
int g_washBrush_level = 2;         //洗地刷电机等级分为1/2/3


int task_magnetic_state_ = 0;
bool use_magnetic_ = false;

ModbusData g_modbus_data;
BatteryFault g_battery_fault; // 电源错误
using json = nlohmann::json;
PowerManage::PowerManage() : set_param_flag_(true), voice_pub_flag_(true), charge_success_(false), 
charge_failed_(false), deploy_on_pile_(true), close_flg_(false), pub_log_flag_(true), 
pub_start_log_flag_(true), pub_free_flag_(true), motor_current_flag_(false), charge_task_(ROBOT_START),  
fault_timer_(0), battery_high_threshold_(90), power_serial_version_(1), start_log_flag_(1),
is_first_charge_success_flag_(false), location_mode_enum_(NONE), tf_(ros::Duration(10))
{
    // 系统授权
    LicenseManage t_lm;
    if (!t_lm.getLicense())
    {
        std_msgs::String license_manage_fault;
        license_manage_fault.data = "powermanage:B0317";
        while (ros::ok())
        {
            sleep(1);
            serial_error_pub_.publish(license_manage_fault);
            //std::cout << "The PowerManage is not licened!!!" << std::endl;
        }
    }

    // 加载电量相关参数
    ros::NodeHandle n("~");
    n.param("powerHighlimit", battery_high_threshold_, 100); 
    n.param("powerLowlimit", battery_low_threshold_, 20); 
    n.param("power_serial_version", power_serial_version_, 1); 
    n.param("car_type", car_type_, std::string("UC")); 
    n.param("default_map", default_map_, std::string("")); 
    //std::cout << "config 配置地图 default_map_ = " << default_map_ << std::endl;
    
    control_pallet_ = (car_type_ == "CD-01") ? true : false;

    // 加载部署相关参数
    n.param("deploy_on_pile", deploy_on_pile_, true); // 是否部署在桩上 
    n.param("close_relay_dis", close_relay_dis_, 0.5);
    n.param("set_motor_current", set_motor_current_, true); // 是否开启电机限流功能
    n.param("set_motor_enable", set_motor_enable_, true); // 是否开机电机使能控制
    n.param("motor_control_mode", motor_control_mode_, 0); // 电机控制模式
    n.param("use_magnetic", use_magnetic_, false); // 控制磁导传感器是否启用
    n.param("motor_current_max_threshold", motor_current_max_threshold_, 300.0); // 20240829  添加电机电流阈值
    n.param("motor_current_min_threshold", motor_current_min_threshold_, 200.0);
    n.param("recharge_num", recharge_num_, 3);
    n.param("recharge_over_time", recharge_over_time_, 70);
    n.param("judge_charge_mode", judge_charge_mode_, 1);
    n.param("charge_pole_direction", pole_direction_, false); 
    n.param<int>("cleanBrushLevel", g_cleanBrush_level, 1);//扫地刷默认一档
    n.param<int>("washBrushLevel", g_washBrush_level, 2);  
    n.param("charge_rot_flag", rot_flag_, false);  
    n.param("battery_drop_interval", battery_drop_interval_, 10);
    n.param("x_axis_position_threshold", x_axis_position_threshold_, 1.0);
    n.param("y_axis_position_threshold", y_axis_position_threshold_, 0.35);
    n.param("control_pallet", control_pallet_, false); // 直接作为参数 是否控制托盘
    n.param<int>("charge_success_to_failed_wait_time", charge_success_to_failed_wait_time_, 3);
    
    int location_mode = 0;
    std::vector<double> charge_map_in_origin_pose;
    n.param("location_mode", location_mode, 0);
    n.getParam("charge_map_in_origin_pose", charge_map_in_origin_pose);
    
    location_mode_enum_ = static_cast<LocationMode>(location_mode);
    if (location_mode_enum_ < NONE || location_mode_enum_ > REFLECTOR_APRILTAG_MODE) 
    {
        ROS_WARN("Invalid location_mode %d, using default NONE", location_mode);
        location_mode_enum_ = NONE;
    }

    RPYPose charge_pose; // 桩前坐标系在建图原点坐标系下的位姿 
    charge_pose.x = charge_map_in_origin_pose[0];
    charge_pose.y = charge_map_in_origin_pose[1];
    charge_pose.z = charge_map_in_origin_pose[2];
    charge_pose.roll = charge_map_in_origin_pose[3];
    charge_pose.pitch = charge_map_in_origin_pose[4];
    charge_pose.yaw = charge_map_in_origin_pose[5];
    T_charge_in_origin_ = RPYPoseToMatrix4d(charge_pose);

    serial_error_pub_ = nh_.advertise<std_msgs::String>("/ugv_fault", 5, true); // 发布串口错误码
    power_data_pub_ = nh_.advertise<std_msgs::String>("/power_data", 10, true); // 电源相关数据
    magnetic_data_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/magnetic_data", 10, true); // 磁导相关数据
    log_pub_ = nh_.advertise<std_msgs::String>("/journal", 10, true); // 发布日志
    serial_status_pub_ = nh_.advertise<std_msgs::Bool>("/power_serial_status", 10, true); // 发布串口状态
    locate_status_pub_ = nh_.advertise<std_msgs::Bool>("/only_use_odom", 10, true); // 发布串口状态
    verson_pub_ = nh_.advertise<std_msgs::String>("/power_manage_verson", 10, true); // 发布版本
    voice_pub_ = nh_.advertise<robot_msgs::Voice>("/voiceplay", 10); // 发布语音
    set_motor_current_pub_ = nh_.advertise<std_msgs::Bool>("/set_motor_current", 5, true); // 20240806  发布电机限流
    set_motor_enable_pub_ = nh_.advertise<std_msgs::Bool>("/set_motor_enable", 5, true);  // 20240806  发布电机使能
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // 20240829  发布速度
    charge_success_pub_ = nh_.advertise<std_msgs::Int32>("/charge_result", 10, true);      // 20251223 dtx 添加充电成功发布话题 
    feedback_pub_ = nh_.advertise<std_msgs::String>("/aim_pile_status", 10);  // 20251223 dtx 发布充电反馈状态 
    cmd_sensor_pub_ = nh_.advertise<std_msgs::String>("/cmd_sensor", 10, true);
    station_localization_failed_pub_ = nh_.advertise<std_msgs::Bool>("/station_localization_failed", 5, true);
    apriltag_detect_pub_ = nh_.advertise<std_msgs::Int8>("/charge_state", 5, true);

    lamp_sub_ = nh_.subscribe<std_msgs::Int8>("/charge_state",1,&PowerManage::LampCtrl,this); // 控制氛围灯
    cam_err_sub_ = nh_.subscribe<std_msgs::Int8>("/cam_dev_err", 1, &PowerManage::CamCallBack, this); // 20250428  倒桩相机掉线处理
    motor_current_sub_ = nh_.subscribe("/motor_current", 1, &PowerManage::MotorCurrentCallBack, this); //20240829  订阅电机电流
    pile_start_pose_sub_ = nh_.subscribe("/pile_start_pose", 10, &PowerManage::PileStartPoseCallBack, this);
    push_io_sub_ = nh_.subscribe<std_msgs::String>("/push_io", 1, &PowerManage::PushIOCallBack, this); // 20250913  订阅托盘状态
    motor_jiting_sub_ = nh_.subscribe<std_msgs::Bool>("/set_motor_jiting", 1, &PowerManage::MotorJiTingCallBack, this); // 20251223 dtx 订阅软急停信号
    localization_failed_sub_ = nh_.subscribe("/localization_failed", 5, &PowerManage::MapDriftCallBack,this);
    io_data_sub_ = nh_.subscribe<std_msgs::String>("/io_data", 1, &PowerManage::IoDataCallBack, this);

    power_data_server_ = nh_.advertiseService("/srv_power_manage", &PowerManage::ManageCmdReceive, this); // 电源管理部分开关控制
  
    originpoint_client_ = nh_.serviceClient<robot_msgs::ElevatorPoint>("/map_point"); // 20250423 c_add 对桩后切换至默认地图
    db_client_ = nh_.serviceClient<robot_msgs::DataBase>("/database_operate");
    push_prohibition_client_ = nh_.serviceClient<robot_msgs::Web>("/srv_push_prohibition");
    ac_client_ = std::make_unique<actionlib::SimpleActionClient<robot_msgs::TaskAction>>("task_list", true); // 20251223 dtx
    task_service_client_ = nh_.serviceClient<robot_msgs::TaskService>("task_service");//20251223 dtx
	set_map_client_ = nh_.serviceClient<nav_msgs::SetMap>("/set_map"); // 20260318  通过set_map矫正位姿
    control_relay_client_ = nh_.serviceClient<robot_msgs::Peripheral>("/srv_lora_cmd");
    sub_motor_config_cmd_ = nh_.subscribe("/ui_motor_config", 10, &PowerManage::UIMotorConfigCallBack, this);//订阅UI电机参数配置话题  

    ///////////////////////////////串口的信号和槽///////////////////////////////////
    //qDEBUG() << "电源控制板协议版本V:" << power_serial_version_ << endl;
    if(power_serial_version_ == 1) 
    {
        pSerialserver_ = new SerialServer();
        connect(this, SIGNAL(SigSendCmd(int)), pSerialserver_, SLOT(SlotReciveCmd(int)));
        connect(this, SIGNAL(SigPowerCmd(int)), pSerialserver_, SLOT(SlotReciveCloseCmd(int)));
        connect(pSerialserver_, SIGNAL(SigPubLogSer(QString)), this, SLOT(SlotLogSerPub(QString)));
        //充电阈值控制
        connect(this, SIGNAL(SigChargeLimt(int)), pSerialserver_, SLOT(SlotChargeLimitCmd(int)));
        pSerialserver_->SerialServerConnect();
    }
    else if(power_serial_version_ == 2)
    {
        pser_ = new SerialPort();
        connect(this, SIGNAL(SigSendCmd(int)), pser_, SLOT(SlotReciveCmd(int)));
        connect(this, SIGNAL(SigPubCmd(QString, QString)), pser_, SLOT(SlotRecvCmd(QString,QString)));
        connect(pser_, SIGNAL(SigPubStatus(QString, QString)), this, SLOT(SlotPowerStatus(QString,QString)));
        connect(this, SIGNAL(SigChargeLimt(int)), pser_, SLOT(SlotChargeLimitCmd(int)));
        pser_->SerialPortInit();
        control_version_sub_ = nh_.subscribe("/control_version", 10, &PowerManage::ControlCallBack, this);  //订阅控制板版本 
        emergency_stop_sub_ = nh_.subscribe("/emergency_stop", 10, &PowerManage::EmergencyCallBack, this); //订阅从电机过来的应急状态
    }
    else
    {
        //qDEBUG() << "电源控制板协议版本错误" << endl;
    }

    if(car_type_ == "DR-01" || car_type_ == "CD-01" || car_type_ == "UC") 
    {
        emit SigChargeLimt(battery_high_threshold_); //电量阈值控制报文发送
    }

    // 定时发布电源相关数据100ms一次
    pPower_pub_timer_ = new QTimer(this);
    if(power_serial_version_ == 1)
    {
        connect(pPower_pub_timer_, SIGNAL(timeout()), this, SLOT(SlotPowerDataPub()));
    }
    else if(power_serial_version_ == 2)
    {
        connect(pPower_pub_timer_, SIGNAL(timeout()), this, SLOT(SlotPowerDataPubNew()));
        pser_->SerialStart();
    }
    pPower_pub_timer_->start(100 * 1);

    start();
}

PowerManage::~PowerManage()
{
    if(power_serial_version_ == 1)
    {
        pSerialserver_->deleteLater();
    }
    else if(power_serial_version_ ==2)
    {
        pser_->deleteLater();
    }
    
    pPower_pub_timer_->deleteLater();
}

void PowerManage::EmergencyCallBack(const std_msgs::UInt16 &msg) 
{
    int ret = msg.data;
    if(ret == 1)
    {
        g_modbus_data.emergency_stop = true;
    }
    else
    {
        g_modbus_data.emergency_stop = false;
    }
        
}

void PowerManage::MotorJiTingCallBack(const std_msgs::Bool::ConstPtr &msg)
{
    motor_jiting_flag_ = msg->data;
}

void PowerManage::DoneCb(const actionlib::SimpleClientGoalState &state, const robot_msgs::TaskResultConstPtr &result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    task_state_ = task_state::successed;
    std::string log_data = "INFO;power_manage;task succeed reason:" + state.text_;
    LogPub(log_data);
  } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
    task_state_ = task_state::aborted;
    std::string log_data = "INFO;power_manage;task ABORTED reason:" + state.text_;
    LogPub(log_data);
  } else {
  }
}
 
void PowerManage::ActiveCb() {
  LogPub("INFO;power_manage;Task just went active");
}

void PowerManage::FeedbackCb(const robot_msgs::TaskFeedbackConstPtr &feedback)
{
    std::unique_lock<std::mutex> lock(failed_id_mtx_);
    leavepile_dis_ = feedback->leavepile_map_to_baselink_dis;
    failed_id_ = feedback->failed_id;
    status_ = feedback->status;
    std::string log_data = "INFO;power_manage;leavepile_dis_:" + std::to_string(leavepile_dis_) + "failed_id_:" + failed_id_ + "status_" + std::to_string(status_);
    LogPub(log_data);
}

void PowerManage::SetChargingGoal(const std::string& str)
{
    robot_msgs::TaskService srv;
    srv.request.actionIndex = 2;
    if (task_service_client_.call(srv))
    {
        LogPub("INFO;power_manage;先取消任务，并且取消任务成功!");
    }
    else
    {
        LogPub("INFO;power_manage;不能取消任务!!!");
        return;
    }
    robot_msgs::TaskGoal action_goal;
    std::string goal_str;
    try {
        json pose;
        pose["x"] = 0;
        pose["y"] = 0;
        pose["theta"] = 0;

        json unit;
        unit["action"] = "";
        unit["type"] = str;
        unit["nav_type"] = str;
        unit["poses"] = json::array();
        unit["poses"].clear();
        unit["poses"].push_back(pose);

        json path;
        path["id"] = 0;
        path["data"].push_back(unit);
        goal_str = path.dump();
    } catch (json::exception &e) {
        ROS_WARN("%s,%s,%d:%s\n", __FILE__, __FUNCTION__, __LINE__, e.what());
    }
    action_goal.task = goal_str;
    task_state_ = task_state::none;
    std::string log_data = "INFO;power_manage;PowerManage::StartChargingGoalCb: get a " + str + " goal , publish to task_base";
    LogPub(log_data);
    ac_client_->sendGoal(action_goal, boost::bind(&PowerManage::DoneCb, this, _1, _2),
                        boost::bind(&PowerManage::ActiveCb, this),
                        boost::bind(&PowerManage::FeedbackCb, this, _1));
}

bool PowerManage::ExceptionHandle(const std::string &msg)
{
    if (monitor_usb_cam_.load())
    {
        if (!sendTaskCommand(0))
        {
            return false;
        }
        if (count_ % 10 == 0)
        {
            std::string log_data = "INFO;power_manage;During the " + msg + "process, the camera lost connection.";
            LogPub(log_data);
        }
        aim_pile_status_.data = "FIND_STATION";
        feedback_pub_.publish(aim_pile_status_);
        has_exception_ = true;
        return true;
    }
    else if (g_modbus_data.emergency_stop)
    {
        if (!sendTaskCommand(0))
        {
            return false;
        }
        if (count_ % 10 == 0)
        {
            // 20250102  日志记录不那么频繁
            std::string log_data = "INFO;power_manage;During the " + msg + "process, the hard emergency stop was triggered:" + std::to_string(g_modbus_data.battery_remaining);
            LogPub(log_data);
        }
        std_msgs::String charge_fault_info;
        charge_fault_info.data = "powermanage:B0400"; // 20250916  定义B0400错误码 急停按钮触发 充电或离桩失败
        serial_error_pub_.publish(charge_fault_info);
        has_exception_ = true;
        return true;
    }
    else if (motor_jiting_flag_)
    {
        if (!sendTaskCommand(0))
        {
            return false;
        }
        if (count_ % 10 == 0)
        {
            // 20250102  日志记录不那么频繁
            std::string log_data = "INFO;power_manage;During the " + msg + "process, the soft emergency stop was triggered:" + std::to_string(g_modbus_data.battery_remaining);
            LogPub(log_data);
        }
        has_exception_ = true;
        return true;
    }
    else
    {
        if (!pause_)
        {
            if (!sendTaskCommand(1))
            {
                return false;
            }
            has_exception_ = false;
        }
        return true;
    }
}

bool PowerManage::sendTaskCommand(const int &actionIndex) 
{
    robot_msgs::TaskService srv;
    srv.request.actionIndex = actionIndex;
    if (status_ == 106 && actionIndex == 0) // 106:task_base任务暂停执行   101:task_base任务正常运行
    {
        return true;
    }
    else if (status_ == 101 && actionIndex == 1)
    {
        return true;
    }
    else
    {
        if (task_service_client_.call(srv))
        {
            LogPub("INFO;power_manage;发送请求成功!");
            return true;
        }
        else
        {
            LogPub("INFO;power_manage;发送请求失败!");
            return false;
        }
    }
}

bool PowerManage::ManageCmdReceive(robot_msgs::PowerCommond::Request &request, robot_msgs::PowerCommond::Response &response)
{
    std::string cmd = request.type;  // 命令类型  peripheralpoweroff  peripheralpoweron poweroff reboot
    std::string data = request.data; // 多久后关机
    //std::cout << "cmd:" << cmd << std::endl;
    //std::cout << "data:" << data << std::endl;
    {
        std::lock_guard<std::mutex> lock(m_recharge_);
        if(recharge_over_)
        {
            recharge_over_ = false;
            redo_flag_ = true;
        }
    }

    //add new serial fangyao 20240902
    if(power_serial_version_ == 1)
    {
        if (cmd == "poweroff")
        {
            int seconds = atoi(data.c_str());
            std::string voice_data = "PowerOff";
            PublishVoice(voice_data);
            //qDEBUG() << seconds << "秒后关机";
            PowerCloseFun(seconds);
            std::string log = "INFO;power_manage;cmd_poweroff:" + data;
            LogPub(log);
            response.status = "success";
        }
        else if (cmd == "reboot")
        {
            int seconds = atoi(data.c_str());
            std::string voice_data = "Reboot";
            PublishVoice(voice_data);
            //qDEBUG() << seconds << "秒后重启";
            PowerRestartFun(seconds);
            std::string log = "INFO;power_manage;cmd_reboot:" + data;
            LogPub(log);
            response.status = "success";
        }
        else if(cmd == "close_charge")
        {
            emit SigSendCmd(EndCharge);
            LogPub("INFO;power_manage;cmd_关闭继电器");
            response.status="success";
        }
        else if(cmd == "open_charge")
        {
            emit SigSendCmd(StartCharge);
            LogPub("INFO;power_manage;cmd_下发打开继电器");
            response.status="success";
        }
        else if(cmd == "batteryCal")
        {
            emit SigSendCmd(BatteryCal);
            LogPub("INFO;power_manage;cmd_ 电源串口版本1,下发电池校准");
            response.status="success";
        }

    }
    else if(power_serial_version_ == 2)
    {
        if (cmd == "poweroff")
        {
            std::cout<<"receive poweroff data = "<<data<<std::endl;
            if (data.empty())
            {
                LogPub("INFO;power_manage;cmd_poweroff: data is null");
            }
            else
            {
                timetask_active_ = true;
                strWakeupTime_ = data;
                timetask_index_ = TIMING;
                std::string voice_data = "PowerOff";
                PublishVoice(voice_data);
                sleep(1);
                TimeTaskManage();
                std::string log = "INFO;power_manage;cmd_poweroff:" + data;
                LogPub(log);
            } 

            response.status = "success";           
        }
        else if (cmd == "reboot")
        {
            int seconds = atoi(data.c_str());            
            std::string voice_data = "Reboot";
            PublishVoice(voice_data);
            sleep(3);
            //qDEBUG() << seconds << "秒后重启";
            emit SigSendCmd(robot_restart);
            std::string log = "INFO;power_manage;cmd_reboot:" + data;
            LogPub(log);
            response.status = "success";
        }
        else if(cmd == "motor_poweron")
        {
            emit SigSendCmd(motor_poweron);
            LogPub("INFO;power_manage;cmd_打开电机电源");
            response.status="success";
        }
        else if(cmd == "motor_poweroff")
        {
            emit SigSendCmd(motor_poweroff);
            LogPub("INFO;power_manage;cmd_关闭电机电源");
            response.status="success";       
        }
        else if(cmd == "cancel_waring")
        {
            emit SigSendCmd(cancel_waring);
            LogPub("INFO;power_manage;cmd_取消报警");
            response.status="success";
        }
        else if(cmd == "aim_pile")
        {
            emit SigSendCmd(aim_pile);
            LogPub("INFO;power_manage;cmd_下发离桩信号");
            response.status="success";      
        }
        else if(cmd == "leave_pile")
        {
            emit SigSendCmd(leave_pile);
            LogPub("INFO;power_manage;cmd_下发关闭继电器");
            response.status="success";
        }
        else if(cmd == "close_charge")
        {
            emit SigSendCmd(leave_pile);
            LogPub("INFO;power_manage;cmd_关闭继电器");
            response.status="success";
        }
        else if(cmd == "open_charge")
        {
            emit SigSendCmd(open_charge);
            LogPub("INFO;power_manage;cmd_下发打开继电器");
            response.status="success";
        }
        else if(cmd == "cleanbrushon")//20250808CleanBrushOn
        {
            emit SigSendCmd(CleanBrushOn);
            LogPub("INFO;power_manage;打开扫地刷");
            response.status="success";
        }
        else if(cmd == "cleanbrushoff")//20250808CleanBrushOff
        {
            emit SigSendCmd(CleanBrushOff);
            LogPub("INFO;power_manage;关闭扫地刷");
            response.status="success";
        }
        else if(cmd == "washbrushon")//20250808WashBrushOn
        {
            emit SigSendCmd(WashBrushOn);
            LogPub("INFO;power_manage;打开洗地刷");
            response.status="success";
        }
        else if(cmd == "washbrushoff")//20250808WashBrushOff
        {
            emit SigSendCmd(WashBrushOff);
            LogPub("INFO;power_manage;关闭洗地刷");
            response.status="success";
        }
        else if(cmd == "batteryCal")
        {
            emit SigSendCmd(Battery_Cal);
            LogPub("INFO;power_manage;cmd_ 电源串口版本2,下发电池校准");
            response.status="success";
        }
        else if(cmd == "batteryCal")
        {
            emit SigSendCmd(Battery_Cal);
            LogPub("INFO;power_manage;cmd_ 电源串口版本2,下发电池校准");
            response.status="success";
        }
        //清除扫地刷故障码
        else if(cmd == "ClearCleanBrushErr")
        {
            emit SigSendCmd(ClearCleanBrushErr);
            LogPub("INFO;power_manage;cmd_ 电源串口版本2,ClearCleanBrushErr");
            response.status="success";
        }
        //清除洗地刷故障码
        else if(cmd == "ClearWashBrushErr")
        {
            emit SigSendCmd(ClearWashBrushErr);
            LogPub("INFO;power_manage;cmd_ 电源串口版本2,ClearWashBrushErr");
            response.status="success";
        }                    
    }

    // 关闭外设电源
    if (cmd == "peripheralpoweroff")
    {
        emit SigSendCmd(PeripheralOff);
        LogPub("INFO;power_manage;cmd_关闭外设电源");
        response.status = "success";
        //qDEBUG() << "关闭外设电源";
    }
    else if (cmd == "peripheralpoweron")
    {
        emit SigSendCmd(PeripheralOn);
        LogPub("INFO;power_manage;cmd_打开外设电源");
        response.status = "success";
        //qDEBUG() << "打开外设电源";
    }
    else if (cmd == "StartCharge")
    {
        int seconds = atoi(data.c_str());
        emit SigSendCmd(StartCharge); // 打开继电器
        //qDEBUG() << "开继电器";
        std::string log_data = "INFO;power_manage;cmd:开继电器,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
        LogPub(log_data);
        response.status = "success";
    }
    else if (cmd == "EndCharge")
    {
        int seconds = atoi(data.c_str());

        if(power_serial_version_ == 2)
        {
            emit SigSendCmd(leave_pile); // 20250116  关闭继电器操作由上位机下发 新电源协议
        }
        else
        {
           emit SigSendCmd(EndCharge); // 20250116  关闭继电器操作由上位机下发 旧电源协议
        }

        //qDEBUG() << "关继电器";
        std::string log_data = "INFO;power_manage;cmd:关继电器,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
        LogPub(log_data);
        response.status = "success";
    }

    else if (cmd == "charge" && data == "start_charge") // 对桩开继电器
    {  
        // 20250116  防止手动推动小车导致继电器未关闭 每次进桩前重新关闭一次继电器
        if(power_serial_version_ == 2)
        {
            emit SigSendCmd(leave_pile); // 20250116  关闭继电器操作由上位机下发 新电源协议
        }
        else
        {
           emit SigSendCmd(EndCharge); // 20250116  关闭继电器操作由上位机下发 旧电源协议
        }
        // 20251223 dtx
        if (monitor_usb_cam_.load())  
        {
            LogPub("WARNING;power_manage;usb_cam_, Not sending the aimplie task!");
            response.status = "failed";
        }
        else
        {
            // 20260327  如果是加装继电器的充电桩发送吸合继电器 充电桩上电
            if (charge_station_type_ == 2)
            {
                // 该服务掉线 直接返回 不执行充电动作
                if (!control_relay_client_.waitForExistence(ros::Duration(10)))
                {
                    LogPub("ERROR;power_manage;控制继电器服务未上线/srv_lora_cmd died");
                    response.status = "success";
                    return true;
                }
                else
                {
                    robot_msgs::Peripheral srv;
                    srv.request.type = "cmd_relay_open";
                    srv.request.data = "";
                    if (control_relay_client_.call(srv))
                    {
                        std::string control_relay_log_data = "INFO;power_manage;开始对桩充电,/srv_lora_cmd,吸合充电桩继电器";
                        LogPub(control_relay_log_data);
                    }
                }
            }
            SetChargingGoal("start_charging");
            {
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = AIM_PILE;
                pub_log_flag_ = true;
            }
            std_msgs::Bool locate_status_value;
            locate_status_value.data = true;
            locate_status_pub_.publish(locate_status_value);
            std_msgs::String msg;
            msg.data = "start_charge";
            cmd_sensor_pub_.publish(msg);

            std::string voice_data = "StartCharge";
            if (car_type_ == "DR-01")
            {
                voice_data = "StartChargeEn";
            }
            PublishVoice(voice_data);
            std::string log_data = "INFO;power_manage;cmd_开始对桩充电,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
            if (charge_station_type_ == 2)
            {
                log_data += ", 当前充电桩继电器状态为: " + std::to_string(station_relay_status_.load());
            }
            LogPub(log_data);
            response.status = "success";
            // qDEBUG() << "开始对桩充电";
        }
    }

    else if (cmd == "charge" && data == "end_charge") // 离桩并关继电器
    {
        { 
            std::unique_lock<std::mutex> lock(m_motor_current_);
            motor_current_flag_ = false; // 20240903  离桩后电机电流标志位设置为false            
        }
        // 20251223 dtx
        if (monitor_usb_cam_.load()) 
        {
            LogPub("WARNING;power_manage;usb_cam_, Not sending the leaveplie task!");
            response.status = "failed";
        }
        else
        {
            // 20260327  如果是加装继电器的充电桩发送断开继电器 充电桩掉电
            if (charge_station_type_ == 2)
            {
                // 该服务掉线 直接返回 不执行离桩动作
                if (!control_relay_client_.waitForExistence(ros::Duration(10)))
                {
                    LogPub("ERROR;power_manage;控制继电器服务未上线/srv_lora_cmd died");
                    response.status = "success";
                    return true;
                }
                else
                {
                    robot_msgs::Peripheral srv;
                    srv.request.type = "cmd_relay_close";
                    srv.request.data = "";
                    if (control_relay_client_.call(srv))
                    {
                        std::string control_relay_log_data = "INFO;power_manage;开始离桩/srv_lora_cmd,断开充电桩继电器";
                        LogPub(control_relay_log_data);
                    }
                }
            }
            SetChargingGoal("end_charging");
            {
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = END_CHARGE;
                pub_log_flag_ = true;
                is_relay_ = true;
            }
            std_msgs::String msg;
            msg.data = "end_charge";
            cmd_sensor_pub_.publish(msg);
            std::string voice_data = "EndCharge";
            if (car_type_ == "DR-01")
            {
                voice_data = "EndChargeEn";
            }
            PublishVoice(voice_data);
            std::string log_data = "INFO;power_manage;cmd_关闭继电器后离桩,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
            LogPub(log_data);
            response.status = "success";
            // qDEBUG() << "关闭继电器后离桩";
        }
    }
    // 20240919  start 添加充电流程中暂停和继续功能
    else if (cmd == "charge" && data == "pause") // 停止充电
    {  
        has_exception_ = true;
        pause_ = true;
        // 20250113  pub_log_flag_变量由m_charge_task_加锁
        {
            std::unique_lock<std::mutex> lock(m_charge_task_);
            pub_log_flag_ = true;
        }

        // 20251223 dtx
        if (!sendTaskCommand(0))
        {
            std::string log_data = "ERROR;power_manage;请求暂停充电任务失败";
            LogPub(log_data);
            response.status = "failed";
        }
        aim_pile_status_.data = "PAUSE";
        feedback_pub_.publish(aim_pile_status_);

        std::string voice_data = "ChargeTaskStop";
        if(car_type_ == "DR-01") 
        {
            voice_data = "ChargeTaskStopEn";
        } 
        PublishVoice(voice_data);
        std::string log_data = "INFO;power_manage;cmd_充电任务流程暂停,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
        LogPub(log_data);
        response.status = "success";
        //qDEBUG() << "充电任务流程暂停";  
    }
    else if (cmd == "charge" && data == "continue")
    {
        has_exception_ = false;
        pause_ = false;
        // 20250113  pub_log_flag_变量由m_charge_task_加锁
        {
            std::unique_lock<std::mutex> lock(m_charge_task_);
            pub_log_flag_ = true;
        }       

        // 20251223 dtx
        if (monitor_usb_cam_.load() || g_modbus_data.emergency_stop || !CloseIO() || motor_jiting_flag_)
        {
            LogPub("WARNING;power_manage;usb_cam_ or emergency_stop_ or !CloseIO() or motor_jiting, continue charge task failed!");
        }
        else
        {
            if (!sendTaskCommand(1))
            {
                std::string log_data = "ERROR;power_manage;请求继续充电任务失败";
                LogPub(log_data);
                response.status = "failed";
                return false;
            }
            aim_pile_status_.data = "CONTINUE";
            feedback_pub_.publish(aim_pile_status_);
        }
        std::string log_data = "INFO;power_manage;cmd_充电任务流程继续,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
        LogPub(log_data);
        response.status = "success";
        //qDEBUG() << "充电任务流程继续";
    }
    // 20240919  end 添加充电流程中暂停和继续功能

    else if (cmd == "charge" && data == "stop") // 停止充电
    {
        {
            std::unique_lock<std::mutex> lock(m_charge_task_);
            charge_task_ = STOP_TASK;
            pub_log_flag_ = true;
        }

        // 20251223 dtx
        if (!sendTaskCommand(2))
        {
            std::string log_data = "ERROR;power_manage;请求结束充电任务失败";
            LogPub(log_data);
            response.status = "failed";
            return false;
        }

        std::string voice_data = "ChargeTaskStop";
        if(car_type_ == "DR-01") 
        {
            voice_data = "ChargeTaskStopEn";
        } 
        PublishVoice(voice_data);
        std::string log_data = "INFO;power_manage;cmd_充电任务停止,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
        LogPub(log_data);
        response.status = "success";
        //qDEBUG() << "充电任务停止";
    }
    else if (cmd == "param_set_low")
    {
        LogPub("INFO;power_manage;cmd_设置参数配置battery_low_threshold_");
        //qDEBUG() << "设置配置参数";
        // 解析传入的参数
        std::map<std::string, float> result_low = ParseString(data);
        auto found_low = result_low.find("battery_low_threshold_");
        if (found_low != result_low.end())
        {
            // 20250113  更改set_param_flag_标志位需要加锁
            {
                std::unique_lock<std::mutex> lock(m_set_param_);
                set_param_flag_ = true;
            }
            
            battery_low_threshold_ = found_low->second;
            response.status = "success";
            //std::cout << "set param  success ,battery_low_threshold_: " << battery_low_threshold_ << std::endl;
        }
        else
        {
            // 20250113  更改set_param_flag_标志位需要加锁
            {
                std::unique_lock<std::mutex> lock(m_set_param_);
                set_param_flag_ = false;
            }            
            response.status = "failed[P0111]";
            //std::cout << "param not found: " << std::endl;
            LogPub("INFO;power_manage;cmd_参数 battery_low_threshold_ 解析失败");
        }
    }
    else if (cmd == "param_set_high")
    {
        LogPub("INFO;power_manage;cmd_设置参数配置battery_high_threshold_");
        //qDEBUG() << "设置配置参数";
        // 解析传入的参数
        std::map<std::string, float> result = ParseString(data);
        auto found = result.find("battery_high_threshold_");
        if (found != result.end())
        {
            // 20250113  更改set_param_flag_标志加锁
            {
                std::unique_lock<std::mutex> lock(m_set_param_);
                set_param_flag_ = true;
            }
            battery_high_threshold_ = found->second;
            response.status = "success";
            //std::cout << "set param  success ,battery_high_threshold_: " << battery_high_threshold_ << std::endl;
            if(car_type_ == "DR-01" || car_type_ == "CD-01" || car_type_ == "UC") 
            {
                emit SigChargeLimt(battery_high_threshold_); //电量阈值控制报文发送
            }
            //std::cout << "电量下限阈值控制报文发送: " << std::endl;
        }
        else
        {
            // 20250113  更改set_param_flag_标志位需要加锁
            {
                std::unique_lock<std::mutex> lock(m_set_param_);
                set_param_flag_ = false;
            } 

            response.status = "failed[P0111]";
            //std::cout << "param not found: " << std::endl;
            LogPub("INFO;power_manage;cmd_ battery_high_threshold_参数解析失败");
        }
    }
    else if(cmd == "open_pollution")
    {
        emit SigSendCmd(open_pollution);
        //std::cout<<"cmd_下发打开排污球阀继电器"<<std::endl;
        LogPub("INFO;power_manage;cmd_下发打开排污球阀继电器");
        response.status="success";
    }
    else if(cmd == "close_pollution")
    {
        emit SigSendCmd(close_pollution);
        //std::cout<<"cmd_下发关闭排污球阀继电器"<<std::endl;
        LogPub("INFO;power_manage;cmd_下发关闭排污球阀继电器");
        response.status="success";
    }
    else if(cmd == "open_coverLock")
    {
        emit SigSendCmd(open_coverLock);
        //std::cout<<"cmd_下发打开上盖电磁锁继电器"<<std::endl;
        LogPub("INFO;power_manage;cmd_下发打开上盖电磁锁继电器");
        response.status="success";
    }
    else if(cmd == "close_coverLock")
    {
        emit SigSendCmd(close_coverLock);
        //std::cout<<"cmd_下发关闭上盖电磁锁继电器"<<std::endl;
        LogPub("INFO;power_manage;cmd_下发关闭上盖电磁锁继电器");
        response.status="success";
    }
    else if(cmd == "LiftLampOpen")
    {
        emit SigSendCmd(LiftLampOpen);
        //std::cout<<"cmd_下发打开左转灯"<<std::endl;
        LogPub("INFO;power_manage;cmd_下发打开左转灯");
        response.status="success";
    }
    else if(cmd == "RightLampOpen")
    {
        emit SigSendCmd(RightLampOpen);
        //std::cout<<"cmd_下发打开右转灯"<<std::endl;
        LogPub("INFO;power_manage;cmd_下发打开右转灯");
        response.status="success";
    }
    else if(cmd == "LampClose")
    {
        emit SigSendCmd(LampClose);
        //std::cout<<"cmd_关灯"<<std::endl;
        LogPub("INFO;power_manage;cmd_关灯");
        response.status="success";
    }
    else if(cmd == "LampOpen")
    {
        emit SigSendCmd(LampOpen);
        //std::cout<<"cmd_开灯"<<std::endl;
        LogPub("INFO;power_manage;cmd_开灯");
        response.status="success";
    }
    else if (cmd == "initial_pose")
    {
        // 不在桩前开机 但是通过ui手动矫正位姿 即认为完成位姿矫正
        station_localization_failed_flag_.store(false);
        response.status="success";
        LogPub("INFO;power_manage;cmd_手动矫正位姿");
    }
    else
    {
        //std::cout << "下发指令不存在 !" << std::endl;
        LogPub("INFO;power_manage;cmd_ 下发指令不存在");
    }
    return true;
}

void PowerManage::RechargeThread(){
    ros::Rate rate(1);
    std::string leave_pile = "leavepile";
    int redo_num = 0;
    int count_time = 0;
    robot_msgs::PowerCommond::Request request;
    robot_msgs::PowerCommond::Response response;
    std::string aim_pile_status;

    while(ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(m_recharge_); 
            redo_flag_ = false;
            aim_pile_status = aim_pile_status_.data; 
        }
        count_time ++;
        if(leave_pile == "leavepile")
        {
            request.type = "charge";  
            request.data = "end_charge"; 
            ManageCmdReceive(request,response);
            leave_pile = "autocharge";
            SlotLogSerPub("INFO;power_manage;寻桩充电重试第" + QString::number(redo_num + 1) + "次离桩操作");
        }
        if(aim_pile_status == "LEAVE_SUCCESS" && leave_pile == "autocharge")
        {
            request.type = "charge";  
            request.data = "start_charge"; 
            ManageCmdReceive(request,response);
            SlotLogSerPub("INFO;power_manage;寻桩充电失败后重试第" + QString::number(redo_num + 1) + "次充电操作");
            leave_pile = "wait";
        }

        // 20250928  如果成功 直接跳出结束重试线程
        if(aim_pile_status == "STATION_CHARGE_SUCCESS")
        {
            break;
        }

        if(leave_pile == "wait")
        {    
            if(aim_pile_status == "STATION_CHARGE_failed[B0403]")
            {
                if(++redo_num < recharge_num_)
                {
                    leave_pile = "leavepile";
                    count_time = 0;
                    SlotLogSerPub("ERROR;power_manage;B0403重试第" + QString::number(redo_num) + "次寻桩充电失败");
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    recharge_over_ = false;
                }
                else
                {
                    redo_num = 0;
                    leave_pile = "overtimes";
                    std_msgs::String charge_fault_info;
                    charge_fault_info.data = "powermanage:B0403";
                    serial_error_pub_.publish(charge_fault_info);
                    SlotLogSerPub("ERROR;power_manage;寻桩充电重试结束，充电失败B0403！");
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    recharge_over_ = true;
                    break;
                }
            }
            else if(aim_pile_status == "STATION_CHARGE_failed[B0404]")
            {
                if(++redo_num < recharge_num_)
                {
                    leave_pile = "leavepile";
                    count_time = 0;
                    SlotLogSerPub("ERROR;power_manage;B0404重试第" + QString::number(redo_num) + "次寻桩充电失败");
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    recharge_over_ = false;
                }
                else
                {
                    redo_num = 0;
                    leave_pile = "overtimes";
                    std_msgs::String charge_fault_info;
                    charge_fault_info.data = "powermanage:B0404";
                    serial_error_pub_.publish(charge_fault_info);
                    SlotLogSerPub("ERROR;power_manage;寻桩充电重试结束，充电失败B0404！");
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    recharge_over_ = true;
                    break;
                }
            }
            else
            {
                if(count_time > recharge_over_time_)
                {
                    SlotLogSerPub("INFO;power_manage;寻桩充电重试结束");
                    count_time = 0;
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    recharge_over_ = true;
                    break;
                }       
            }
        }
        rate.sleep();
    }
    return;
}

void PowerManage::run()
{
    RobotState status = ROBOT_START; // 初始状态为刚开机状态
    ros::Rate rate(10); // 0.1s
    count_ = 0;
    while (ros::ok())
    {
        if (count_ >= 10) // 记录频率，以此频率进行状态发布
        {
            count_ = 0;
        }

        // 20251022 选择判断充电成功条件 1-> 电极状态 pole_status 0-> 充电状态 charge_status
        if(judge_charge_mode_ == 1)
        {
            charge_success_flag_ = g_modbus_data.pole_status;
            std_msgs::Int32 msg;
            msg.data = g_modbus_data.pole_status ? 1 : 0;
            charge_success_pub_.publish(msg);
        }
        else
        {
            charge_success_flag_ = g_modbus_data.charge_status;
            std_msgs::Int32 msg;
            msg.data = g_modbus_data.charge_status ? 1 : 0;
            charge_success_pub_.publish(msg);
        }
        
        // 获取数据，确定是否修改充电上下限状态
        bool set_param_flag = true;
        {
            std::unique_lock<std::mutex> lock(m_set_param_);
            set_param_flag = set_param_flag_;
        }

        if (!set_param_flag) // 若状态为false，则说明参数设置失败，发布错误码
        {
            if (count_ % 10 == 0)
            {
                std_msgs::String manage_fault;
                manage_fault.data = "powermanage:P0111";
                serial_error_pub_.publish(manage_fault);
            }
        }

        // 监测电量情况
        if (voice_pub_flag_ && g_modbus_data.battery_remaining < battery_low_threshold_)
        {
            voice_pub_flag_ = false;
            std::string voice_data = "LowPower";
            PublishVoice(voice_data);
        }
        if (g_modbus_data.battery_remaining >= battery_low_threshold_)
        {
            voice_pub_flag_ = true;
        }

        // 充电状态更改，接受ros服务话题
        {
            std::unique_lock<std::mutex> lock(m_charge_task_);
            status = charge_task_;
        }

        // 需要保证StationPoseCorrect在一个循环中
        // 方便定位丢失后循环调用 该函数需要在桩前定位失败或定位失败时调用 桩前定位模式开启时使用 咖啡车中模式为NONE
        // 手动矫正位姿localization_failed_flag_会被置为false
        if (location_mode_enum_ != NONE && (station_localization_failed_flag_.load() || localization_failed_flag_.load()))
        {
            StationPoseCorrect();
        }

        if (location_mode_enum_ != NONE && station_localization_failed_flag_.load())
        {
            // 桩前定位失败 发true
            std_msgs::Bool station_localization_failed;
            station_localization_failed.data = true;
            station_localization_failed_pub_.publish(station_localization_failed);
        }

        // 不开启该功能 或者在桩前定位成功 或者手动矫正位姿成功（由ui上层通过调用srv_power_mange触发）
        if (!finish_station_localization_ && (location_mode_enum_ == NONE || !station_localization_failed_flag_.load()))
        {
            std_msgs::Bool station_localization_failed;
            station_localization_failed.data = false;
            // 发三次定位成功 防止ui未接收到
            for (size_t i = 0; i < 3; i++)
            {
                station_localization_failed_pub_.publish(station_localization_failed);
            }
            finish_station_localization_ = true;

            // 关闭apriltag检测
            std_msgs::Int8 apriltag_detect;
            apriltag_detect.data = 0;
            apriltag_detect_pub_.publish(apriltag_detect);
            apriltag_detect_start_.store(false);
        }

        // 确定充电任务的状态
        switch (status)
        {

        case AIM_PILE:
        {
            std::lock_guard<std::mutex> lock(m_recharge_);
            bool flag = ExceptionHandle("aimpile");
            if(!flag || has_exception_)
            {
                break;
            }
            if (control_pallet_)
            {
                if (CloseIO())
                {
                    // 20251223 dtx
                    if (!sendTaskCommand(1))
                    {
                        break;
                    }
                    std::string log_data = "INFO;power_manage;开始寻桩充电，托盘上升服务已禁用!";
                    LogPub(log_data);
                }
                else
                {
				 // 更详细错误日志记录在closeio
                    if (!sendTaskCommand(0))
                    {
                        break;
                    }
                    std::string log_data = "INFO;power_manage;暂停寻桩充电，托盘上升服务禁用失败！";
                    LogPub(log_data);
                    break;
                }
            }
            // 执行寻桩充电时就播报语音直到充电结束，设置语音发布频率为8秒
			static int start_charge_frequence = 0;
			if(start_charge_frequence % 80 == 0)
            {
				std::string voice_data = "StartCharge";
                if(car_type_ == "DR-01") 
                {
                    voice_data = "StartChargeEn";
                } 
			   	PublishVoice(voice_data);
				start_charge_frequence = 0;
			}
			start_charge_frequence++;
            // 20240806  end 
 
            aim_pile_status_.data = "FIND_STATION";
            feedback_pub_.publish(aim_pile_status_);
            // 若充电任务结束，则直接退出执行部分
            if (charge_task_ == STOP_TASK)
            {
                break;
            }
            
            //20251223 dtx
            if (task_state_ == task_state::successed)
            {
                // 重置标志位 方便进行下一次关闭外设
                task_state_ = task_state::none; 
                cmd_push_flag_ = false;
                push_prohibition_flag_ = false;
                io_log_flag_ = true; 
                PublishStartMotorState(motor_control_mode_, set_motor_current_, set_motor_enable_); //  20240808 控制电机模式 限制电机电流或者控制电机失能 start
                std_msgs::String msg;
                msg.data = "over";
                cmd_sensor_pub_.publish(msg);
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = AIM_PILE_FINISH;
            }
            else if (task_state_ == task_state::aborted)
            {
                // 重置标志位 方便进行下一次关闭外设
                task_state_ = task_state::none;
                cmd_push_flag_ = false;
                push_prohibition_flag_ = false;
                io_log_flag_ = true;
                motor_status_ = false; // 20241203  电机状态控制标志位
                std_msgs::String msg;
                msg.data = "over";
                cmd_sensor_pub_.publish(msg);
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = AIM_PILE_FAILED;
            }
            break;
        }

        // 对桩成功
        case AIM_PILE_FINISH:
        {
            std::lock_guard<std::mutex> lock(m_recharge_);
            aim_pile_status_.data = "FIND_STATION_SUCCESS";
            feedback_pub_.publish(aim_pile_status_);
            {
                std::unique_lock<std::mutex> lock(m_charge_task_);
                // charge_task_ = WAITING_CHARGE;
                charge_task_ = CHARGE_SUCCESS; // 20241104  电极接触即上报充电成功
                std::string log_data = "INFO;power_manage;cmd_对桩完成,准备充电,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
                LogPub(log_data);
                is_first_charge_success_flag_ = true;
            }

            break;
        }

        // 对桩失败
        case AIM_PILE_FAILED:
        {
            // 添加充电桩类型为2时的判断
            bool charge_success = (charge_station_type_ == 2 && power_serial_version_ == 2) ? 
                                  (g_modbus_data.bms_charge_state && charge_success_flag_) : charge_success_flag_;
            if(charge_success) 
            {
                io_log_flag_ = true;
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = CHARGE_SUCCESS;
                pub_log_flag_ = true;
                is_first_charge_success_flag_ = true;
                break;
            }
            if (count_ % 10 == 0)
            {
                std::string failed_id;
                {
                    std::unique_lock<std::mutex> lock(failed_id_mtx_);
                    failed_id = failed_id_;
                }
                if (failed_id == "B0403") 
                {     
                    bool redo_flag;
                    bool recharge_over;
                    {
                        std::lock_guard<std::mutex> lock(m_recharge_);
                        aim_pile_status_.data = "STATION_CHARGE_failed[B0403]";
                        feedback_pub_.publish(aim_pile_status_);

                        redo_flag = redo_flag_;
                        recharge_over = recharge_over_;
                    }

                    std::string log_data = "INFO;power_manage;cmd_未与电极接触或者电极没电:" + std::to_string(g_modbus_data.battery_remaining);
                    LogPub(log_data);
                    if(!motor_status_)
                    {
                        PublishStartMotorState(motor_control_mode_, set_motor_current_, set_motor_enable_); //  20240809 只在当前状态下控制电机
                        motor_status_ = true; // 20241203  设置为true，确保发一次限流
                    }

                    if(recharge_over || recharge_num_ == 0)
                    {
                        std_msgs::String charge_fault_info;
                        charge_fault_info.data = "powermanage:B0403";
                        serial_error_pub_.publish(charge_fault_info);
                    }
                    if(redo_flag && recharge_num_ > 0 && charge_station_type_ == 1)
                    {
                        std::thread redo_thread(&PowerManage::RechargeThread, this);
                        redo_thread.detach();
                    }
                }
                else
                {  
                    std_msgs::String charge_fault_info;
                    charge_fault_info.data = "powermanage:" + failed_id; 
                    serial_error_pub_.publish(charge_fault_info);
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    aim_pile_status_.data = "FIND_STATION_failed[" + failed_id + "]"; 
                    feedback_pub_.publish(aim_pile_status_); 
                }
            }
            break;
        }

        // 执行开始充电
        case WAITING_CHARGE:
        {
            // 该状态下pole_status已经为true
            if (g_modbus_data.charge_success || g_modbus_data.charge_full)
            {
                charge_success_ = true;
            } 
            if (!charge_success_)
            {
                // if (is_out_time_) // 20251223 dtx
                // {
                //     charge_success_ = false;
                //     std::unique_lock<std::mutex> lock(m_charge_task_);
                //     charge_task_ = CHARGE_FAILED;
                //     std_msgs::Bool first_flag; 
                //     first_flag.data = true;
                //     first_flag_pub_.publish(first_flag);
                //     std::string log_data = "INFO;power_manage;cmd_等待充电超时，嵌入式未给出充电状态反馈,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
                //     LogPub(log_data);
                //     break;
                // }
                if (count_ % 10 == 0)
                {
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    aim_pile_status_.data = "READY_STATION_CHARGE"; 
                    feedback_pub_.publish(aim_pile_status_);
                }
                break; // 防止出现 false -> true
            }

            if (charge_success_)
            {
                charge_success_ = false;
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = CHARGE_SUCCESS;
            }
            
            break;
        }

        // 充电成功
        case CHARGE_SUCCESS:
        {
            // 20250913  上报充电成功前 必须保证托盘可控 开托盘
            if(control_pallet_)
            {
                if(io_log_flag_ && OpenIO())
                {
                    std::string log_data = "INFO;power_manage;充电成功，托盘上升服务已开启！";
                    LogPub(log_data);
                    io_log_flag_ = false;
                }
                else if(io_log_flag_)
                {
                    std::string log_data = "INFO;power_manage;充电成功，托盘上升服务开启失败！";
                    LogPub(log_data);
                    break;
                }   
            }

            if (count_ % 10 == 0)
            {
                // 不上报充电满 因为charge_full字段值在高电量跳转
                /*
                if (g_modbus_data.battery_remaining > battery_high_threshold_ && g_modbus_data.charge_full)
                {
                    pController_->aim_pile_status_.data = "STATION_CHARGE_FULL";
                    pController_->feedback_pub_.publish(pController_->aim_pile_status_);
                }
                else
                */
                // 从充电成功跳转充电失败 仅当电极状态为false时 延时判断 
                static uint8_t check_times = 0;
                /*
                判断逻辑说明
                charge_station_type_ == 2 power_serial_version_ == 2 g_modbus_data.bms_charge_state
                对于咖啡车 目前充电桩类型为1 只能通过charge_flag_判断是否从充电成功跳转到充电失败 手动推出不会上报
                对于清洁车 当充电桩类型为1时 逻辑判断与咖啡车一样
                         当充电桩类型为2时 需要检查bms_charge_state的值 而只有power_serial_version_ == 2时 这个变量才有值
                */
                if(!charge_success_flag_ || (charge_station_type_ == 2 && power_serial_version_ == 2 && !g_modbus_data.bms_charge_state))
                {
                    check_times++;
                    if(check_times > charge_success_to_failed_wait_time_) 
                    {
                        check_times = 0;
                        {
                            io_log_flag_ = true;
                            std::unique_lock<std::mutex> lock(m_charge_task_);
                            charge_task_ = CHARGE_FAILED;
                            pub_log_flag_ = true; // 20250113  在跳转到charge_filed后 能继续写入日志
                        }
   
                        if (charge_station_type_ == 2)
                        {
                            // 在这里控制继电器开关 关充电桩继电器和小车继电器  
                            robot_msgs::Peripheral srv;
                            srv.request.type = "cmd_relay_close";
                            srv.request.data = "";
                            if (control_relay_client_.call(srv))
                            {
                                std::string control_relay_log_data = "INFO;power_manage;手动离桩/srv_lora_cmd,关充电桩继电器";
                                LogPub(control_relay_log_data);
                            }
                        }
                        // 这里关闭小车继电器是为了避免手动推动接触电极时产生火花
                        // 因为此时小车电极带电 充电桩未带电 
                        if(power_serial_version_ == 2)
                        {
                            emit SigSendCmd(leave_pile); // 20260330  关闭小车继电器
                        }
                        else
                        {
                            emit SigSendCmd(EndCharge); // 20260330  关闭小车继电器
                        }
                    }
                    break;
                }
                else
                {
                    {
                        std::unique_lock<std::mutex> lock(m_charge_task_);
                        if (is_first_charge_success_flag_)
                        {
                            first_charge_success_battery_ = g_modbus_data.battery_remaining;
                            is_first_charge_success_flag_ = false;
                        }
                    }

                    int current_battery_remaining = g_modbus_data.battery_remaining;
                    if ((current_battery_remaining < first_charge_success_battery_) 
                        && ((first_charge_success_battery_ - current_battery_remaining) > battery_drop_interval_)
                        && current_battery_remaining < battery_low_threshold_)
                    {
                        std_msgs::String battery_fault;
                        battery_fault.data = "battery:P0110";
                        serial_error_pub_.publish(battery_fault);
                    }
                    
                    check_times = 0;
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    aim_pile_status_.data = "STATION_CHARGE_SUCCESS"; 
                    feedback_pub_.publish(aim_pile_status_);
                    // 20250928  重置标志 保证再次由成功跳转到失败后 触发重试
                    redo_flag_ = true;
                    recharge_over_ = false;
                }
             
                // 20240829  start 判断充电成功后电机电流
                {
                    std::unique_lock<std::mutex> lock(m_motor_current_);
                    if((motor_current_data_[0] > motor_current_max_threshold_) || (motor_current_data_[1] > motor_current_max_threshold_))
                    {
                        PublishCmdVel();
                        motor_current_flag_ = true;
                        break;
                    }
                    if(motor_current_flag_ && ((motor_current_data_[0] > motor_current_min_threshold_) || (motor_current_data_[1] > motor_current_min_threshold_)))
                    {
                        PublishCmdVel();
                    }
                    else
                    {
                        motor_current_flag_ = false;
                    }
                    // 20240829  end
                }
            }

            if (pub_log_flag_)
            {                  
                std::string log_data = "INFO;power_manage;cmd_充电成功,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
                LogPub(log_data); 
				std::unique_lock<std::mutex> lock(m_charge_task_);
                pub_log_flag_ = false;  
            }
            break;
        }

        // 充电失败
        case CHARGE_FAILED:
        {
            bool charge_success = (charge_station_type_ == 2 && power_serial_version_ == 2) ? 
                                  (g_modbus_data.bms_charge_state && charge_success_flag_) : charge_success_flag_;

            if(charge_success)
            {
                io_log_flag_ = true;
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = CHARGE_SUCCESS;
                pub_log_flag_ = true; // 20250113  在跳转到charge_success后 能继续写入日志
                is_first_charge_success_flag_ = true;
                break;
            }

            if (count_ % 10 == 0)
            {
                bool redo_flag;
                bool recharge_over;
                {
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    aim_pile_status_.data = "STATION_CHARGE_failed[B0404]"; 
                    feedback_pub_.publish(aim_pile_status_);

                    redo_flag = redo_flag_;
                    recharge_over = recharge_over_;
                }
                
                // 20250926  这个状态只能从成功跳转到失败
                // 检查托盘状态 如果不是下降状态 不进行充电重试
                // 托盘状态判断
                bool push_status;
                {
                    std::unique_lock<std::mutex> lock(m_push_io_);
                    push_status = push_io_status_;
                }
                // 控制托盘时 只有托盘是下降状态时 再进行失败重试 
                if(control_pallet_ && push_status)
                {
                    if(recharge_over || recharge_num_ == 0)
                    {
                        std_msgs::String charge_fault_info;
                        charge_fault_info.data = "powermanage:B0404";
                        serial_error_pub_.publish(charge_fault_info);
                    }
                    if(redo_flag && recharge_num_ > 0 && charge_station_type_ == 1)
                    {
                        std::thread redo_thread(&PowerManage::RechargeThread, this);
                        redo_thread.detach();
                    }

                }
                // 不控制托盘时 直接重试
                else if(!control_pallet_)
                {
                    if(recharge_over || recharge_num_ == 0)
                    {
                        std_msgs::String charge_fault_info;
                        charge_fault_info.data = "powermanage:B0404";
                        serial_error_pub_.publish(charge_fault_info);
                    }
                    // 对于需要开关继电器的充电桩 由于存在手动推离的情况 不进行充电重试
                    if(redo_flag && recharge_num_ > 0 && charge_station_type_ == 1)
                    {
                        std::thread redo_thread(&PowerManage::RechargeThread, this);
                        redo_thread.detach();
                    }

                }
                // 控托盘 但是托盘状态上升
                else
                {
                    if(io_log_flag_)
                    {
                        std::string log_data = "INFO;power_manage;充电失败时托盘未处于下降状态，不重试充电！";
                        LogPub(log_data);
                        io_log_flag_ = false;
                    }
                }
            }

            if (pub_log_flag_)
            {
                std::string log_data = "INFO;power_manage;cmd_充电失败,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
                LogPub(log_data);
                std::unique_lock<std::mutex> lock(m_charge_task_);
                pub_log_flag_ = false;
            }
            break;
        }

        // 执行充电任务
        case END_CHARGE:
        {
            // 检查托盘状态并关闭外设
            if(control_pallet_)
            {
                if(CloseIO())
                {
                    if (!sendTaskCommand(1))
                    {
                        break;
                    }
                    std::string log_data = "INFO;power_manage;开始离桩，托盘上升服务已禁用！";
                    LogPub(log_data);
                }
                else
                {
                    if (!sendTaskCommand(0))
                    {
                        break;
                    }
                    std::string log_data = "INFO;power_manage;暂停离桩，托盘上升服务禁用失败！";
                    LogPub(log_data);
                    break;
                }
            }
            PublishEndMotorState(motor_control_mode_, set_motor_current_, set_motor_enable_); //  20240808 控制电机模式 限制电机电流或者控制电机使能 
            std::unique_lock<std::mutex> lock(m_charge_task_);
            charge_task_ = LEAVE_PILE;
            break;
        }

        // 离桩
        case LEAVE_PILE:
        {
            std::lock_guard<std::mutex> lock(m_recharge_);
            bool flag = ExceptionHandle("leavepile");
            if(!flag || has_exception_)
            {
                break;
            }
            // 执行离桩时就播报语音直到离桩结束，设置语音发布频率为8秒
            static int end_charge_frequence = 0;
            if (end_charge_frequence % 80 == 0)
            {
                std::string voice_data = "EndCharge";
                if (car_type_ == "dinner")
                {
                    voice_data = "EndChargeEn";
                }
                PublishVoice(voice_data);
                end_charge_frequence = 0;
            }
            end_charge_frequence++;

            // 若停止离桩，此时将状态改为STOP_TASK
            aim_pile_status_.data = "LEAVE_STATION";
            feedback_pub_.publish(aim_pile_status_);
            // 若充电任务结束，则直接退出执行部分
            if (charge_task_ == STOP_TASK)
            {
                break;
            }
            
            // 20251223 dtx
            if (task_state_ == task_state::aborted)
            {
                // 重置标志位 方便进行下一次关闭外设
                task_state_ = task_state::none; 
                cmd_push_flag_ = false;
                push_prohibition_flag_ = false;
                io_log_flag_ = true;
                std_msgs::String msg;
                msg.data = "over";
                cmd_sensor_pub_.publish(msg);
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = END_CHARGE_FAILED;
                break;
            }
            if (task_state_ == task_state::successed)
            { 
                // 重置标志位 方便进行下一次关闭外设
                task_state_ = task_state::none;
                cmd_push_flag_ = false;
                push_prohibition_flag_ = false;
                io_log_flag_ = true;
                std_msgs::Bool locate_status_value;
                locate_status_value.data = false;
                locate_status_pub_.publish(locate_status_value);
                std_msgs::String msg;
                msg.data = "over";
                cmd_sensor_pub_.publish(msg);
                close_relay_status_ = false; // 20250213  添加关闭继电器标志 确保在结束充电成功后只发布一次
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = END_CHARGE_SUCCESS;
                pub_log_flag_ = true;
                // 20250522 c_add 添加离桩后校正位置
                CorrectPose();
            }
            
            // 20250122  当离桩距离大于该阈值时 下发关闭继电器信号
            if (leavepile_dis_ > close_relay_dis_ && is_relay_)
            {
                if(power_serial_version_ == 2)
                {
                    emit SigSendCmd(leave_pile); // 20250113  关闭继电器操作由上位机下发 新电源协议
                }
                else
                {
                    emit SigSendCmd(EndCharge); // 20250115  关闭继电器操作由上位机下发 旧电源协议
                }
                std::unique_lock<std::mutex> lock(m_charge_task_);
                is_relay_ = false;
	        }
		
            if(!charge_success_flag_ && pub_log_flag_)
            {
                std::string log_data = "INFO;power_manage;cmd_已经关闭了继电器, 当前电量:" + std::to_string(g_modbus_data.battery_remaining);    
                LogPub(log_data);
                std::unique_lock<std::mutex> lock(m_charge_task_);
                pub_log_flag_ = false;
            }
            break;
        }

        // 停止充电成功
        case END_CHARGE_SUCCESS:
        {
            // 20250913  上报离桩完成前 必须保证托盘可控
            if(control_pallet_)
            {
                if(io_log_flag_ && OpenIO())
                {
                    std::string log_data = "INFO;power_manage;离桩完成，托盘上升服务已开启！";
                    LogPub(log_data);
                    io_log_flag_ = false;
                }
                else if(io_log_flag_)
                {
                    std::string log_data = "INFO;power_manage;充电成功，托盘上升服务开启失败！";
                    LogPub(log_data);
                    break;
                }            
            }
            // 20250213  添加标志位确保只发一次
            if(!close_relay_status_)
            {
                // 20250210  分别在离桩成功和失败状态下再次下发关闭继电器
                if(power_serial_version_ == 2)
                {
                    emit SigSendCmd(leave_pile); // 20250116  关闭继电器操作由上位机下发 新电源协议
                }
                else
                {
                    emit SigSendCmd(EndCharge); // 20250116  关闭继电器操作由上位机下发 旧电源协议
                }
                close_relay_status_ = true;           
            }
            if (count_ % 10 == 0)
            {
                std::lock_guard<std::mutex> lock(m_recharge_);
                aim_pile_status_.data = "LEAVE_SUCCESS";
                feedback_pub_.publish(aim_pile_status_);
            }

            if (pub_log_flag_)
            {
                std::string log_data = "INFO;power_manage;cmd_停止充电成功,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
                LogPub(log_data);
                std::unique_lock<std::mutex> lock(m_charge_task_);
                pub_log_flag_ = false;
            }
            
            // 20260330  对于充电桩类型为2的车 可以在任意状态下进行手动充电 这里也添加是否充电成功跳转
            // bms_charge_state和charge_flag_有所不同 前者直接读取电池状态 后者根据充电电流判断或者只读取电极状态
            bool charge_success = false;
            if (charge_station_type_ == 2)
            {
                charge_success = power_serial_version_ == 2 && g_modbus_data.bms_charge_state && charge_success_flag_;
            }

            if(charge_success)
            {
                io_log_flag_ = true;
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = CHARGE_SUCCESS;
                pub_log_flag_ = true; // 20250113  在跳转到charge_success后 能继续写入日志
                is_first_charge_success_flag_ = true;
            }
            break;
        }

        // 停止充电失败
        case END_CHARGE_FAILED:
        { 
            // 目前离桩时不会进入这个状态 不需要在此判断充电失败到成功的跳转
            // 20250213  添加标志位确保只发一次
            if(!close_relay_status_)
            {
                // 20250210  分别在离桩成功和失败状态下再次下发关闭继电器
                if(power_serial_version_ == 2)
                {
                    emit SigSendCmd(leave_pile); // 20250116  关闭继电器操作由上位机下发 新电源协议
                }
                else
                {
                    emit SigSendCmd(EndCharge); // 20250116  关闭继电器操作由上位机下发 旧电源协议
                }
                close_relay_status_ = true;           
            }

            if (count_ % 10 == 0)
            {
                std_msgs::String charge_fault_info;
                charge_fault_info.data = "powermanage:B0405";
                serial_error_pub_.publish(charge_fault_info);
                std::lock_guard<std::mutex> lock(m_recharge_);
                aim_pile_status_.data = "END_STATION_CHARGE_failed[B0405]"; 
                feedback_pub_.publish(aim_pile_status_);
            }

            if (pub_log_flag_)
            {                    
                std::string log_data = "INFO;power_manage;cmd_停止充电失败,当前电量:" + std::to_string(g_modbus_data.battery_remaining);
                LogPub(log_data);               
                std::unique_lock<std::mutex> lock(m_charge_task_);
                pub_log_flag_ = false;
            }

            break;
        }

        // 发布零速，状态位置为WAITING_TASK 该状态暂未使用
        case STOP_TASK:
        {
            {
                std::unique_lock<std::mutex> lock(m_charge_task_);
                charge_task_ = WAITING_TASK;
                pub_free_flag_ = true; // 置true 确保跳转到WAITING_TASK记录一次日志
                if(power_serial_version_ == 2)
                {
                    emit SigSendCmd(leave_pile); // 20250116  关闭继电器操作由上位机下发 新电源协议
                }
                else
                {
                    emit SigSendCmd(EndCharge); // 20250116  关闭继电器操作由上位机下发 旧电源协议
                }
            }
            break;
        }

        // 刚开机等待任务执行状态
        case ROBOT_START:
        {
            // 电源串口未打开时 会重连
        	if (!g_modbus_data.serial_status)
        	{
        		break;
        	}
            if (deploy_on_pile_) // 默认在桩上
            { 
                // 20241018  start 通过电极状态判断
                // 小车重启时 电极状态为true 就判断充电成功
                static u_int8_t ploe_status_times = 0; 
                bool charge_success = (charge_station_type_ == 2 && power_serial_version_ == 2) ? 
                                      (g_modbus_data.bms_charge_state && charge_success_flag_) : charge_success_flag_;
                if(charge_success)
                {
                    ploe_status_times = 0;
                    if (pub_start_log_flag_ || start_log_flag_ % 2 == 1)
                    {
                        std::string log_data = "INFO;power_manage;ROBOT_START STATION_CHARGE_SUCCESS";
                        LogPub(log_data);
                        pub_start_log_flag_ = false;
                        start_log_flag_ = 2;
                    }
                    // 上报充电成功
                    if (count_ % 10 == 0)
                    {
                        std::lock_guard<std::mutex> lock(m_recharge_);
                        aim_pile_status_.data = "STATION_CHARGE_SUCCESS"; 
                        feedback_pub_.publish(aim_pile_status_);

                        redo_flag_ = true;
                        recharge_over_ = false;
                    }
                }
                else
                {
                    ploe_status_times++;
                    if (ploe_status_times > charge_success_to_failed_wait_time_ * 10) // 等待30s
                    {
                        ploe_status_times = 0; // 置 0
                        // B0403 表示未与电极接触或者电极不带电
                        if (pub_start_log_flag_ || start_log_flag_ % 2 == 0) // 记录日志
                        {
                            std::string log_data = "INFO;power_manage;ROBOT_START STATION_CHARGE_failed[B0406]"; //开机后未在桩上充电
                            LogPub(log_data);
                            pub_start_log_flag_ = false;
                            start_log_flag_ = 1;
                            if (charge_station_type_ == 2)
                            {
                                robot_msgs::Peripheral srv;
                                srv.request.type = "cmd_relay_close";
                                srv.request.data = "";
                                if (control_relay_client_.call(srv))
                                {
                                    std::string control_relay_log_data = "INFO;power_manage;充电成功跳转到充电失败/srv_lora_cmd,关充电桩继电器";
                                    LogPub(control_relay_log_data);
                                }
                                if(power_serial_version_ == 2)
                                {
                                    emit SigSendCmd(leave_pile); // 20260330  关闭小车继电器
                                }
                                else
                                {
                                    emit SigSendCmd(EndCharge); // 20260330  关闭小车继电器
                                }
                            } 
                        }
                        // 上报充电失败                        
                        std_msgs::String charge_fault_info;
                        charge_fault_info.data = "powermanage:B0406";
                        serial_error_pub_.publish(charge_fault_info);
                        std::lock_guard<std::mutex> lock(m_recharge_);
                        aim_pile_status_.data = "STATION_CHARGE_failed[B0406]"; 
                        feedback_pub_.publish(aim_pile_status_);
                    }
                }
                // 20241018  end 通过电极状态判断
            }
            else
            {
                if (pub_free_flag_)
                {
                    std::string log_data = "INFO;power_manage;ROBOT_START FREE";
                    LogPub(log_data);
                    pub_free_flag_ = false;
                }
                if (count_ % 10 == 0)
                {
                    std::lock_guard<std::mutex> lock(m_recharge_);
                    aim_pile_status_.data = "FREE"; 
                    feedback_pub_.publish(aim_pile_status_);
                }
            }
            break;
        }

        // 空闲状态
        case WAITING_TASK:
        {
            if (pub_free_flag_)
            {
                std::string log_data = "INFO;power_manage;WAITING_TASK FREE";
                LogPub(log_data);
                pub_free_flag_ = false;
            }
            if (count_ % 10 == 0)
            {
                std::lock_guard<std::mutex> lock(m_recharge_);
                aim_pile_status_.data = "FREE"; 
                feedback_pub_.publish(aim_pile_status_);
            }
            break;
            // 等待任务，空闲状态
        }
        }
        ros::spinOnce();
        rate.sleep();
        count_++;
    }
}
