/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "../utility/tic_toc.h"

//todo
#include <mutex>
#include <queue>
#include <nav_msgs/Odometry.h>
//todo


// 特征点在每一帧上的属性
// 它指的是空间特征点P1映射到frame1或frame2上对应的图像坐标、特征点的跟踪速度、空间坐标等属性都封装到类FeaturePerFrame中
class FeaturePerFrame //路标点j图像帧时刻i时刻的特征点信息
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 9, 1> &_point, double td)//todo 7
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 9, 1> &_point) //todo 7
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;
    Vector3d point, pointRight;//路标点在左相机、右相机的归一化相机坐标系下的位置坐标
    Vector2d uv, uvRight; //路标点在左相机、右相机的图像坐标系下的位置（去畸变的位置坐标）
    Vector2d velocity, velocityRight; //路标点在左相机、右相机的归一化相机坐标系下的速度
    bool is_stereo;
};


// 管理一个特征点
// 就特征点P1来说，它被两个帧观测到，第一次观测到P1的帧为frame1,即start_frame=1，
// 最后一次观测到P1的帧为frame2,即endframe()=2,并把start_frame~endframe() 对应帧的属性存储起来，
class FeaturePerId// 路标点j在所有观测到该路标点的图像帧上的特征点信息
{
  public:
    const int feature_id;  // 特征点id
    int start_frame;// 起始观测帧，在滑窗中的位置
    vector<FeaturePerFrame> feature_per_frame;// 该id对应的特征点在每个帧中的属性
    int used_num;//出现的次数
    double estimated_depth;// 三角化深度，在首帧观测帧下的深度值z
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail; // 该特征点的状态，是否被三角

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};



// 划窗内所有的路标点管理
// 管理所有路标点信息
class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
      // 计算新来的帧与上一帧中特征点的平均视差  return:  ture 视差较大， false视差较小
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> &image, double td);//todo 7
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //! no use
    //todo 12.25 use for calibrating camera extrinsic between camera and odom
    vector<pair<Vector3d, Vector3d>> getCorresponding_3d_points(int frame_count_l);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    
//todo
    //todo 1 for optimize
    //添加轮式编码器获取数据函数
    void getOdomData(Eigen::Quaterniond& Q, Eigen::Vector3d& P, nav_msgs::Odometry::ConstPtr& odomData);
    //bool getPoseByWheelOdom(Eigen::Vector3d& Pcam, Eigen::Matrix3d& Rcam, const double curTime);
    bool getPoseByWheelOdom(Eigen::Vector3d& Pcam, Eigen::Matrix3d& Rcam, const double curTime);//todo

    void linear_insert(Eigen::Quaterniond& Qodom, Eigen::Vector3d& Podom, const double sync_time, nav_msgs::Odometry::ConstPtr& front_data, nav_msgs::Odometry::ConstPtr& back_data);

    bool getRelativePoseByWheel(Eigen::Matrix3d& Rcam, Eigen::Vector3d& Pcam,
        const double prevTime, const double curTime);

    void solvePoseByICP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point3f> &pts3D_world, vector<cv::Point3f> &pts3D_camera);



    //todo 2 for horizon generator  好像并不用先不加实现了11.23
    void getOdomData(Eigen::Quaterniond& Q, Eigen::Vector3d& P, Eigen::Vector3d& V, Eigen::Vector3d& a, Eigen::Vector3d& w, 
                     Eigen::Vector3d& Ba, Eigen::Vector3d& Bg, nav_msgs::Odometry::ConstPtr& odomData);

    bool getPoseByWheelOdom(Eigen::Vector3d& Pcam, Eigen::Matrix3d& Rcam, Eigen::Vector3d& V, Eigen::Vector3d& a, Eigen::Vector3d& w, 
                     Eigen::Vector3d& Ba, Eigen::Vector3d& Bg, const double curTime);

    void linear_insert(Eigen::Quaterniond& Qodom, Eigen::Vector3d& Podom, Eigen::Vector3d& V, Eigen::Vector3d& a, Eigen::Vector3d& w, 
                     Eigen::Vector3d& Ba, Eigen::Vector3d& Bg,
                     const double sync_time, nav_msgs::Odometry::ConstPtr& front_data, nav_msgs::Odometry::ConstPtr& back_data);
    

    bool getRelativePoseByWheel(Eigen::Matrix3d& Rcam, Eigen::Vector3d& Pcam, Eigen::Vector3d& V, Eigen::Vector3d& a, Eigen::Vector3d& w, 
                     Eigen::Vector3d& Ba, Eigen::Vector3d& Bg, 
                     const double prevTime, const double curTime);
//todo


    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier(set<int> &outlierIndex);
    list<FeaturePerId> feature;; //划窗内所有的路标点，list的每一个元素都是一个路标点FeaturePerId
    int last_track_num;//追踪的数目
    double last_average_parallax;
    int new_feature_num;//新提的特征点的数目
    int long_track_num;//追踪的图总image数目+1

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);//计算frame_count-1帧 frame_count-2帧的其中一个特征点的视差
    const Matrix3d *Rs;
    Matrix3d ric[2]; // IMU-Camera 外参，左右目
};

#endif
//todo
// extern queue<nav_msgs::Odometry::ConstPtr> wheel_odom_buf;
// extern std::mutex m_wheel_odom;
//todo
