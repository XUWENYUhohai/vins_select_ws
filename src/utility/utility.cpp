/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{//初始化：本体坐标系与世界坐标系对齐，// 重力旋转到z轴上？
//从输入的加速度和重力加速度得到一个初始位姿
//计算g对齐到重力加速度方向所需的旋转
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();//归一化后变成了初始加速度的方向
    Eigen::Vector3d ng2{0, 0, 1.0};//这个是理想的重力加速度,ENU世界坐标系下的z方向（重力加速度方向）
    ///ng1为当前IMU坐标系的z方向，R0 * ng1 = ng2，表示当前IMU坐标系在世界系中的姿态，或者说R0可以将当前IMU坐标系点变换成世界坐标系点
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();// FromTwoVectors得到两个向量ng1, ng2之间的旋转
    double yaw = Utility::R2ypr(R0).x();
    // 我们只想对齐z轴，旋转过程中可能改变了yaw角，再旋转回去
    //这里是对yaw取反之后乘在原来的R0，让yaw=0的一个措施
    //TODO:如果加入磁力计的话就在这里
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;//IMU系到世界坐标系的旋转矩阵 R0?
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
