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

#include "initial_ex_rotation.h"

InitialEXRotation::InitialEXRotation(){
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}


// 标定imu和相机之间的旋转外参，通过imu和图像计算的旋转使用手眼标定计算获得
//当外参完全不知道的时候，可以在线对其进行初步估计,然后在后续优化时，会在optimize函数中再次优化。
//输入是新图像和上一阵图像的位姿 和二者之间的imu预积分值,输出旋转矩阵
// 根据连续两帧的位置，两帧间预积分的 delta_q增量，得到校准的calib_ric 旋转量
// 根据窗口内的多帧 10帧
/**
 * 在线标定外参旋转
 * 利用两帧之间的Camera旋转和IMU积分旋转，构建最小二乘问题，SVD求解外参旋转
 * 1、Camera系，两帧匹配点计算本质矩阵E，分解得到四个解，根据三角化成功点比例确定最终正确解R、t，得到两帧之间的旋转R
 * 2、IMU系，积分计算两帧之间的旋转
 * 3、根据旋转构建最小二乘问题，SVD求解外参旋转
 * @param corres            前一帧与当前帧匹配点
 * @param delta_q_imu       前一帧与当前帧IMU预积分得到的旋转
 * @param calib_ric_result  在线标定IMU与Camera之间的外参（旋转）
*/
bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    frame_count++;
    Rc.push_back(solveRelativeR(corres));// 根据特征关联求解两个连续帧相机的旋转R12
    Rimu.push_back(delta_q_imu.toRotationMatrix());
      // 变换到Camera系下，R_ck_ik * R_ik_ik+1 * R_ik+1_ck+1 = R_ck_ck+1
    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);// ric是上一次求解得到的外参

    Eigen::MatrixXd A(frame_count * 4, 4);
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);//角度误差
        ROS_DEBUG(
            "%d %f", i, angular_distance);
 // huber核函数，超过5°，打折
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
        ++sum_ok;
        Matrix4d L, R;
 // 四元数的左乘矩阵     https://zhuanlan.zhihu.com/p/108658768?utm_id=0
        double w = Quaterniond(Rc[i]).w();
        Vector3d q = Quaterniond(Rc[i]).vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;
// 四元数的右乘矩阵
        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }

    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Quaterniond estimated_R(x);
    ric = estimated_R.toRotationMatrix().inverse();
    //cout << svd.singularValues().transpose() << endl;
    //cout << ric << endl;
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
    {
        calib_ric_result = ric;
        return true;
    }
    else
        return false;
}


/**
 * 两帧匹配点计算本质矩阵E，分解得到四个解，根据三角化成功点比例确定最终正确解R、t，得到两帧之间的旋转R
 * @param corres 两帧匹配点，归一化相机平面点
*/
Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
          // 求解出来的是E
        /**函数findFundamentalMat()既可以用来求基本矩阵F，也可以求本质矩阵E。求什么取决于你的参数points1和points2是像素坐标还是归一化平面坐标。
         *  Mat cv::findFundamentalMat(  返回通过RANSAC算法求解两幅图像之间的本质矩阵E
         *      nputArray  points1,             第一幅图像点的数组
         *      InputArray  points2,            第二幅图像点的数组
         *      int     method = FM_RANSAC,     RANSAC 算法
         *      double  param1 = 3.,            点到对极线的最大距离，超过这个值的点将被舍弃
         *      double  param2 = 0.99,          矩阵正确的可信度
         *      OutputArray mask = noArray()    输出在计算过程中没有被舍弃的点
         *  ) 
        */   
        cv::Mat E = cv::findFundamentalMat(ll, rr);
        cv::Mat_<double> R1, R2, t1, t2;
        decomposeE(E, R1, R2, t1, t2);

        if (determinant(R1) + 1.0 < 1e-09) // 旋转矩阵的行列式应该是1,这里如果是-1就取一下反
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        // 对本质矩阵E分解得到的四个解，测试三角化成功点比例，比例最高的是正确解
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;
// 解出来的是R21
        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j); // 这里转换成R12
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}


/**
 * @brief 通过三角化来检查R t是否合理
 * 
 * @param[in] l l相机的观测
 * @param[in] r r相机的观测
 * @param[in] R 旋转矩阵
 * @param[in] t 位移
 * @return double 
 */
double InitialEXRotation::testTriangulation(const vector<cv::Point2f> &l,
                                          const vector<cv::Point2f> &r,
                                          cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));

    // 三角化
    // 参数：第一帧投影矩阵（单位矩阵）、第二帧投影矩阵（在第二帧坐标系下）、第一帧归一化相机点、第二帧归一化相机点、三角化点结果（相机坐标系，第一帧为参考）
    cv::triangulatePoints(P, P1, l, r, pointcloud);
    int front_count = 0;
    for (int i = 0; i < pointcloud.cols; i++)
    {
        double normal_factor = pointcloud.col(i).at<float>(3);
 // 得到在各自相机坐标系下的3d坐标
        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
        if (p_3d_l(2) > 0 && p_3d_r(2) > 0)   // 深度值大于0
            front_count++;
    }
    ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
}

/**
 * 本质矩阵分解得到R、t， E=t^R
*/
void InitialEXRotation::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}
