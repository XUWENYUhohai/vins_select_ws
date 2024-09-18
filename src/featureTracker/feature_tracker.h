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

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>

#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
//todo
#include "cvmodified.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt); //判断关键点是否在边界上
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);
void reduceVector(vector<double> &v, vector<uchar> status);


class FeatureTracker
{
public:
    FeatureTracker();
     // trackImage vins特征追踪的核心函数   //todo 7
    map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();

    void readIntrinsicParameter(const vector<string> &calib_file,const int depth);//todo

    void showUndistortion(const string &name);//显示 去畸变和归一化坐标映射的效果
    void rejectWithF();

    void rejectWithE_stereo();//todo 1.5

    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);//根据不同的相机模型去除畸变
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();//返回正在追踪的frame , 而且带有特征点和特征点追踪的线

    cv::Mat getSelectImage();//todo
    
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;// 当前正在追踪的图，左图或者 左右合并的图
    cv::Mat mask;//用于标记点的图像
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img; //当前需要追踪的图和上一张图  
    cv::Mat prev_img_tmp, cur_img_tmp;//todo 3.16 
    vector<cv::Point2f> n_pts; //光流追踪不够的，需要Corner角点补的特征点//从图片上返回的特征，shi-tomasi角点（Harris角点）
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;//setPrediction生成的
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts; //当前图的特征点，以及前一张图的特征点,//cur_pts当前帧上的特征点，双目中的左目，并且应该像素坐标
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;//为归一化相机座标系下的座标。
    vector<cv::Point2f> pts_velocity, right_pts_velocity;//当前帧每个特征点的 v_x, v_y //像素移动速度
    vector<int> ids, ids_right;//ids这个就是当前帧特征点数目的索引
    vector<int> track_cnt;//保存了当前追踪到的角点一共被多少帧图像追踪到
    vector<double> score0 , scores;//todo
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map; //cur_un_pts_map中存放ids[i]和cur_un_pts[i]构成的键值对。
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;//当前右目上的点
    map<int, cv::Point2f> prevLeftPtsMap;//上一帧的左目中的点
    vector<camodocal::CameraPtr> m_camera; //所选的相机模型
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;
    
    //todo
    bool depth_cam;

    int min_dist = 30;//TODO 1.5

    //TODO 3.22
    FILE* min_dis;
    FILE* new_point_num;
    FILE* last_score;
    FILE* new_last_score;
    FILE* true_match_pro;

    double cur_pts_num; 

    std::vector<cv::KeyPoint> keypoint1;
    cv::Mat des1;
};
