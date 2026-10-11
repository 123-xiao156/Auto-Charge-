#include "controller/charge_controller/tag_odom_location.h"

namespace TagLocation
{
    TagOdomLocation::TagOdomLocation(Params &params) : params_(params), initialized_(false)
    {
        T_reference_map_to_odom_ = Eigen::Matrix4d::Identity();
    }

    TagOdomLocation::~TagOdomLocation()
    {
    }

    bool TagOdomLocation::Run(LocationInput &input, bool tag_update_flag, bool reflector_update_flag, Eigen::Matrix4d &T_reference_map_to_baselink)
    {
        if (initialized_ == false) // 若未进行初始化，则需要初始化坐标系 也就是建立map与odom之间的关系
        {
            if (input.reflector_pose_in_baselink.is_valid && reflector_update_flag) // 若激光雷达坐标系构建成功
            {
                Eigen::Matrix4d T_baselink_to_odom = input.baselink_pose_in_odom.transform_matrix.inverse();
                Eigen::Matrix4d T_reference_map_to_baselink = input.reflector_pose_in_baselink.transform_matrix.inverse();
                T_reference_map_to_odom_ = T_reference_map_to_baselink * T_baselink_to_odom; // 初始化得到 odom相对于map的位姿
                initialized_ = true;
                //LOG(INFO) << "init throungh reflector!";
            }
            else if (input.apriltag_pose_in_baselink.is_valid && tag_update_flag)
            {
                Eigen::Matrix4d T_reference_map_to_baselink = input.apriltag_pose_in_baselink.transform_matrix.inverse();
                Eigen::Matrix4d T_baselink_to_odom = input.baselink_pose_in_odom.transform_matrix.inverse();
                T_reference_map_to_odom_ =T_reference_map_to_baselink * T_baselink_to_odom;
                initialized_ = true;
                //LOG(INFO) << "init throungh apriltag!";
            }
            else
            {
                //LOG(INFO) << "等待初始化!";
                return false;
            }
            T_reference_map_to_baselink = T_reference_map_to_odom_ * input.baselink_pose_in_odom.transform_matrix;
            return true;
        }

        // 通过反光板维护误差
        if (input.reflector_pose_in_baselink.is_valid && reflector_update_flag)
        {
            // 反光板维护的坐标系变换
            Eigen::Matrix4d T_reference_map_to_baselink = input.reflector_pose_in_baselink.transform_matrix.inverse();
            // 里程计维护的坐标系变换
            T_reference_map_to_odom_ = T_reference_map_to_baselink * input.baselink_pose_in_odom.transform_matrix.inverse();
            //LOG(INFO) << "reflector update map";   
        }
        else if (input.apriltag_pose_in_baselink.is_valid && tag_update_flag) // 通过apriltag码维护T_map_to_odom之间的误差
        {
            // apriltag码维护的坐标系变换
            Eigen::Matrix4d T_reference_map_to_baselink = input.apriltag_pose_in_baselink.transform_matrix.inverse();
            // 里程计维护的坐标系变换
            // 20250327 liuzhiwen 在这里就判断一次 防止这次识别到的apriltag码是错误的 导致传出更新的T_reference_map_to_baselink也是错误的
            if(sqrt(T_reference_map_to_baselink(0, 3) * T_reference_map_to_baselink(0, 3) + T_reference_map_to_baselink(1, 3) * T_reference_map_to_baselink(1, 3)) < apriltag_pose_in_baselink_dis_)
            {
                T_reference_map_to_odom_ = T_reference_map_to_baselink * input.baselink_pose_in_odom.transform_matrix.inverse();
                //LOG(INFO) << "apriltag update map";
            }
        }
        // 不成功的话，使用上一次的变换
        T_reference_map_to_baselink = T_reference_map_to_odom_ * input.baselink_pose_in_odom.transform_matrix;
        return true;
    }

    void TagOdomLocation::ResetParams()
    {
        initialized_ = false;
        T_reference_map_to_odom_ = Eigen::Matrix4d::Identity();
    }

} // namespace TagLocation
