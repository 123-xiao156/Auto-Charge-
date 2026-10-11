#include "utils.h"

Eigen::Matrix4d PoseStampedToMatrix4d(const geometry_msgs::PoseStamped &pose_stamped)
{
    Eigen::Vector3d position(pose_stamped.pose.position.x,
                             pose_stamped.pose.position.y,
                             pose_stamped.pose.position.z);

    Eigen::Quaterniond orientation(pose_stamped.pose.orientation.w,
                                   pose_stamped.pose.orientation.x,
                                   pose_stamped.pose.orientation.y,
                                   pose_stamped.pose.orientation.z);

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = position;

    return transform;
}

geometry_msgs::PoseWithCovarianceStamped Matrix4dToPoseWithCovarianceStamped(const Eigen::Matrix4d &transform,
                                                 const std::string &frame_id)
{
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;

    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = ros::Time::now();

    pose_stamped.pose.pose.position.x = transform(0, 3);
    pose_stamped.pose.pose.position.y = transform(1, 3);
    pose_stamped.pose.pose.position.z = transform(2, 3);
    
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
    Eigen::Quaterniond quaternion(rotation_matrix);
    quaternion.normalize();

    pose_stamped.pose.pose.orientation.x = quaternion.x();
    pose_stamped.pose.pose.orientation.y = quaternion.y();
    pose_stamped.pose.pose.orientation.z = quaternion.z();
    pose_stamped.pose.pose.orientation.w = quaternion.w();

    return pose_stamped;
}

bool IsIdentityMatrix(const Eigen::Matrix4d &matrix)
{
    Eigen::Matrix4d identityMatrix = Eigen::Matrix4d::Identity();
    double epsilon = 1e-6; // 定义误差范围

    // 比较给定矩阵与单位矩阵的相似度
    bool isEqual = matrix.isApprox(identityMatrix, epsilon);
    return isEqual;
}

Eigen::Matrix4d RPYPoseToMatrix4d(RPYPose pose)
{
    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(pose.roll, Eigen::Vector3d::UnitX());

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            transform_matrix(row, col) = rotation_matrix(row, col);
        }
    }

    transform_matrix(0, 3) = pose.x;
    transform_matrix(1, 3) = pose.y;
    transform_matrix(2, 3) = pose.z;
    return transform_matrix;
}

RPYPose Matrix4dToEulerAnglesZYX(const Eigen::Matrix4d &matrix)
{
    RPYPose pose;
    pose.yaw = std::atan2(matrix(1, 0), matrix(0, 0));
    pose.pitch = std::atan2(-matrix(2, 0), std::sqrt(matrix(2, 0) * matrix(2, 0) + matrix(2, 2) * matrix(2, 2)));
    pose.roll = std::atan2(matrix(2, 1), matrix(2, 2));
    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);
    return pose;
}
