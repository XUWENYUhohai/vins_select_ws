/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../estimator/feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame//一帧图像
{
    public:
        ImageFrame(){};
        //1：路标点编号 
        //1：相机编号   7:3:归一化坐标，2：像素坐标，2：归一化速度
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>>& _points, double _t):t{_t},is_key_frame{false}//todo
        {
            points = _points;
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>> > > points;//todo 7
        double t;
        Matrix3d R; //imu相对世界坐标系的旋转、平移；R_w_i、t_w-i
                    //（PS：但是initialStructure求解过程中未考虑imu与camera之间的平移；原因：此时尺度因子未定，不太方便利用二者间的平移）
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs);
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);

//todo
bool VisualIMUAlignmentWithDepth(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
//todo