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

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"
/**
 * not-used
 * 位姿参考值约束，类似于先验
 * 残差维度是6（旋转只保留虚部），变量维度虽然是7
*/
class InitialPoseFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    InitialPoseFactor(const Eigen::Vector3d &_P, const Eigen::Quaterniond &_Q)
    {
    	init_P = _P;
    	init_Q = _Q;
    	sqrt_info = 1000 * Eigen::Matrix<double, 6, 6>::Identity();
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
    	Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
    	Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    	Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    	residual.block<3, 1>(0, 0) = P - init_P;
    	residual.block<3, 1>(3, 0) = 2 * (init_Q.inverse() * Q).vec();
    	residual = sqrt_info * residual;

    	if (jacobians)
    	{
    		if (jacobians[0])
    		{
    		    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
    		    jacobian_pose.setZero();
    		    jacobian_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    		    jacobian_pose.block<3, 3>(3, 3) = Utility::Qleft(init_Q.inverse() * Q).bottomRightCorner<3, 3>();
    		    jacobian_pose = sqrt_info * jacobian_pose;
    		}

    	}
    	return true;

    }

    void check(double **parameters)
    {
	    double *res = new double[6];
	    double **jaco = new double *[1];
	    jaco[0] = new double[6 * 7];
	    Evaluate(parameters, res, jaco);
	    puts("check begins");//puts()函数的功能是用于输出一个字符串，其中括号内的参数是输出字符串的起始地址。 [1]  用来向标准输出设备(屏幕)写字符串并换行

	    puts("my");

	    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(res).transpose() << std::endl
	              << std::endl;
	    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
	              << std::endl;

		Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
		Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

		Eigen::Matrix<double, 6, 1> residual;
		residual.block<3, 1>(0, 0) = P - init_P;
		residual.block<3, 1>(3, 0) = 2 * (init_Q.inverse()* Q).vec();
		residual = sqrt_info * residual;

	    puts("num");
	    std::cout << residual.transpose() << std::endl;

	    const double eps = 1e-6;
	    Eigen::Matrix<double, 6, 6> num_jacobian;
	    for (int k = 0; k < 6; k++)
	    {
	    	Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
	    	Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

	        int a = k / 3, b = k % 3;
	        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;//==是关系运算符，用来判断两个值大小是否相同，当左边的内容与右边的内容相同时，返回1，其余时候返回0。

	        if (a == 0)
	            P += delta;
	        else if (a == 1)
	            Q = Q * Utility::deltaQ(delta);


	        Eigen::Matrix<double, 6, 1> tmp_residual;
	        tmp_residual.block<3, 1>(0, 0) = P - init_P;
	        tmp_residual.block<3, 1>(3, 0) = 2 * (init_Q.inverse()* Q).vec();
	        tmp_residual = sqrt_info * tmp_residual;

	        num_jacobian.col(k) = (tmp_residual - residual) / eps;
	    }
	    std::cout << num_jacobian << std::endl;

    }

    Eigen::Vector3d init_P;
    Eigen::Quaterniond init_Q;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};
