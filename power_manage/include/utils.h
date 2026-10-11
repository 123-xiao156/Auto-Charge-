#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

struct RPYPose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

// 定义充电的状态机
typedef enum
{
    AIM_PILE,          // 对桩状态
    LEAVE_PILE,        // 离桩状态
    STOP_TASK,         // 停止任务
    AIM_PILE_FINISH,   // 对桩完成
    AIM_PILE_FAILED,   // 对桩失败
    LEAVE_PILE_FINISH, // 离桩完成
    LEAVE_PILE_FAILED, // 离桩失败
    READY_CHARGE,      // 正在嵌入式打开继电器
    CHARGE_SUCCESS,     // 充电成功
    CHARGE_FAILED,     // 充电失败
    END_CHARGE,        // 执行关闭充电
    END_CHARGE_SUCCESS, // 停止充电成功
    END_CHARGE_FAILED, // 停止充电失败
    WAITING_CHARGE,    // free及等待状态
    WAITING_TASK,      // 等待任务
    ROBOT_START,       // 机器人刚开机状态
} RobotState;

typedef enum
{
    NONE = 0, // 不使用桩前定位功能
    REFLECTOR_MODE, // 只使用反光板在桩前定位
    APRILTAG_MODE, // 只使用apriltag在桩前定位
    REFLECTOR_APRILTAG_MODE, // 使用反光板和apriltag在桩前定位
} LocationMode;

/**********************************************************
 * Description: 将geometry_msgs::PoseStamped类型转换成变换矩阵
 * Input:    pose_stamped: 包含位置和四元数
 * Output:   无
 * Return:   Eigen::Matrix4d 转换矩阵
 * Modify Date       Version       Author       Modification    
 * ---------------------------------------------------------
 * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
 **********************************************************/  
Eigen::Matrix4d PoseStampedToMatrix4d(const geometry_msgs::PoseStamped &pose_stamped);

/**********************************************************
 * Description: 位姿转换成转换矩阵
 * Input:    pose: 当前位姿 
 * Output:   无
 * Return:   Eigen::Matrix4d: 转换矩阵
 * Modify Date       Version       Author       Modification    
 * ---------------------------------------------------------
 * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
 **********************************************************/  
Eigen::Matrix4d RPYPoseToMatrix4d(RPYPose pose);

/**********************************************************
 * Description: 判断是否为单位帧
 * Input:    matrix: 转换矩阵 
 * Output:   无
 * Return:   true: 是单位帧
 *           false: 不是单位帧
 * Modify Date       Version       Author       Modification    
 * ---------------------------------------------------------
 * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
 **********************************************************/  
bool IsIdentityMatrix(const Eigen::Matrix4d &matrix);

/**********************************************************
 * Description: 转换矩阵转换成欧拉角并计算出位姿
 * Input:    matrix: 转换矩阵 
 * Output:   无
 * Return:   RPYPose: 转换得到的位姿
 * Modify Date       Version       Author       Modification    
 * ---------------------------------------------------------
 * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
 **********************************************************/  
RPYPose Matrix4dToEulerAnglesZYX(const Eigen::Matrix4d &matrix);

/**********************************************************
 * Description: 将转换矩阵转换成geometry_msgs::PoseWithCovarianceStamped
                类型消息，供矫正位姿发布
 * Input:    matrix: 转换矩阵 
             frame_id: 发布在该系下的位姿
 * Output:   无
 * Return:   geometry_msgs::PoseWithCovarianceStamped: 转换得到的消息
 * Modify Date       Version       Author       Modification    
 * ---------------------------------------------------------
 * 2026/02/11        v1.0          liuzhiwen    代码初次注释  
 **********************************************************/ 
geometry_msgs::PoseWithCovarianceStamped Matrix4dToPoseWithCovarianceStamped(const Eigen::Matrix4d &transform,
                                                 const std::string &frame_id = "map");
