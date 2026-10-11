#ifndef TAG_ODOM_LOCATION_H
#define TAG_ODOM_LOCATION_H

#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Core>

namespace TagLocation
{
    struct Params
    {
        Eigen::Matrix4d T_baselink_to_cam;
        Eigen::Matrix4d T_baselink_to_lidar;
        Eigen::Matrix4d T_apriltag_to_reference_map;
        Eigen::Matrix4d T_reflector_to_reference_map;
        Eigen::Matrix4d T_door_reflector_to_reference_map;
    };
    struct SensorPose
    {
        Eigen::Matrix4d transform_matrix;
        bool is_one_tag_valid;
        bool is_valid;
    };

    struct LocationInput
    {
        SensorPose apriltag_pose_in_baselink;
        SensorPose reflector_pose_in_baselink;
        SensorPose baselink_pose_in_odom;
    };

    class TagOdomLocation
    {
    private:
        Params params_;              

    public:
        /**********************************************************
         * Description: TagOdomLocation类构造函数
         * Input:    param: 待初始化相关参数
         * Output:   无
         * Return:   无
         * Modify Date       Version       Author       Modification    
         * ---------------------------------------------------------
         * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
         **********************************************************/  
        TagOdomLocation(Params &param);

        /**********************************************************
         * Description: TagOdomLocation类析构函数
         * Input:    无
         * Output:   无
         * Return:   无
         * Modify Date       Version       Author       Modification    
         * ---------------------------------------------------------
         * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
         **********************************************************/  
        ~TagOdomLocation();

        /**********************************************************
         * Description: 更新基坐标系在参考地图坐标系下的转换矩阵
         * Input:    input: 当前包含里程计，apriltag码，反光板计算得到的转换
         *           tag_update_flag: apriltag码更新标志位
         *           reflector_update_flag: 反光板更新标志位
         * Output:   T_map_to_baselink: 基坐标系在参考地图坐标系下的转换矩阵
         * Return:   true: 更新成功
         *           false: 更新失败（使用里程计更新）
         * Modify Date       Version       Author       Modification    
         * ---------------------------------------------------------
         * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
         **********************************************************/  
        bool Run(LocationInput &input, bool tag_update_flag, 
            bool reflector_update_flag, Eigen::Matrix4d &T_map_to_baselink);
        
        /**********************************************************
         * Description: 重置参数
         * Input:    无
         * Output:   无
         * Return:   无
         * Modify Date       Version       Author       Modification    
         * ---------------------------------------------------------
         * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
         **********************************************************/          
        void ResetParams();

        bool initialized_; // 是否完成首帧初始化
        Eigen::Matrix4d T_reference_map_to_odom_; // 当前的map在初始map中的变换关系，也就是误差矩阵
        double apriltag_pose_in_baselink_dis_;
    };

} // namespace TagLocation

#endif // TAG_ODOM_LOCATION_H
