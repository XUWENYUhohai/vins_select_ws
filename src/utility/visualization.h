/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//todo
#include <sensor_msgs/LaserScan.h>  

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>//离群点去除滤波器（统计滤波器）
#include <pcl/filters/voxel_grid.h>//体素滤波
#include <pcl/filters/radius_outlier_removal.h>//半径滤波器

#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>// for octomap

#include <nav_msgs/Odometry.h>
//todo


#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include <fstream>

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;
extern nav_msgs::Path path;
extern ros::Publisher pub_pose_graph;

//todo
// extern ros::Publisher pub_octree;
// extern pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>* octree;
// extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
// extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;

// extern pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
// extern pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
// extern pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_filter;

// void pubOctreefromRGBD(const Estimator &estimator, const std_msgs::Header &header);
// void pubOctreefromStereo(const Estimator &estimator, const std_msgs::Header &header);

//extern FILE* out_File;
//todo


extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);// 将节点n中的信息发布

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubTrackImage(const cv::Mat &imgTrack, const double t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator & estimator, const std_msgs::Header &header);
