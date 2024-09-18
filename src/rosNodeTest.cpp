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

#include <stdio.h>//标准输入输出 printf
#include <queue>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>//这个类中提供的API主要功能是将图像从sensor_msgs/Image类型转化成cv::Mat类型
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
//todo
#include <ctime>
#include <random>

#include <iostream>
#include <stdio.h>
#include <string>

#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>//离群点去除滤波器（统计滤波器）
#include <pcl/filters/voxel_grid.h>//体素滤波
#include <pcl/filters/radius_outlier_removal.h>//了半径滤波器

#include <sensor_msgs/PointCloud2.h>

#include "/home/xuwenyu/vins_stereo_ws/src/VINS-Fusion/vins_estimator/thridparty/elas/elas.h"
//todo
Estimator estimator; //定义estimator全局变量，所以一开始就初始化了,也就是VIO。自动执行该类的构造函数

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
//todo
//img0 rbg图像
//img1 深度图像
//todo
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;// 操作缓存队列需要上锁

map<double,vector<Eigen::Matrix<float, 6, 1>>> point_rgbd;


map<double,double> odom_w_z;


//todo，这个现在在feature_manager.h中定义(换到parameters.h中了)
queue<nav_msgs::Odometry::ConstPtr> wheel_odom_buf;
queue<nav_msgs::Odometry::ConstPtr> wheel_odom_w_buf;

std::mutex m_wheel_odom;

std::mutex m_wheel_w_odom;

bool write_switch = false;
bool write_odom_switch = false;
FILE* out_File;
FILE* out_odom_File;

//todo
ros::Publisher pub_octree;
pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>* octree;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;



pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;
pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_filter;

//todo


//todo
void wheel_odom_callback(const nav_msgs::Odometry::ConstPtr &wheel_odomtry_msg)
{   
    //ROS_INFO("wheel_callback2");
    m_wheel_odom.lock();
    wheel_odom_buf.push(wheel_odomtry_msg);

    // odom_w_z.insert(pair<double,double>(wheel_odomtry_msg->header.stamp.toSec() , wheel_odomtry_msg->twist.twist.angular.z));//! 没用

    // wheel_odom_w_buf.push(wheel_odomtry_msg);

    m_wheel_odom.unlock();

    m_wheel_w_odom.lock();
    wheel_odom_w_buf.push(wheel_odomtry_msg);
    m_wheel_w_odom.unlock();

}

//! rgbd用于消除残影的线性差值，用于判断旋转使用
double linear_w_insert(const double sync_time,nav_msgs::Odometry::ConstPtr &front_data, nav_msgs::Odometry::ConstPtr &back_data)
{
    //y = (x1 - x)/(x1 - x0)y0 +(x - x0)/(x1 - x0)y1
    double front_scale = (back_data->header.stamp.toSec() - sync_time) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    double back_scale = (sync_time - front_data->header.stamp.toSec()) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());

    //线性插值
    //只插值twist.z
    double z = front_data->twist.twist.angular.z * front_scale + back_data->twist.twist.angular.z * back_scale;
    return z;
}


//todo 没用到
// bool getPose_W_ByWheelOdom(const double time)
// {
//     if (wheel_odom_w_buf.empty()) {
//     printf("odom w data has not push into buf yet! \n");
//     return false;
//     }

//     nav_msgs::Odometry::ConstPtr odomFront;
//     nav_msgs::Odometry::ConstPtr odomBack;
//     if(time < wheel_odom_w_buf.front()->header.stamp.toSec() || time > wheel_odom_w_buf.back()->header.stamp.toSec())
//     {   
//         cout << "旋转 1" << endl;
//         // if(abs(odom_w_z[time]) <= 0.05)
//         {
//             getPointCloudfromRGBD(image_rgb, image1,header);    //todo
//             cout << "旋转 2" << endl;
//         }
//     }
//     else
//     {
//         m_wheel_w_odom.lock();
//         while (wheel_odom_w_buf.front()->header.stamp.toSec() <= time)
//         {
//             if (wheel_odom_w_buf.front()->header.stamp.toSec() == time)
//             {
//                 if(abs(wheel_odom_w_buf.front()->twist.twist.angular.z) <= 0.05)
//                 {
//                     getPointCloudfromRGBD(image_rgb, image1,header);    //todo

//                     wheel_odom_w_buf.pop();
//                     m_wheel_w_odom.unlock();
//                 }
//             }

//             odomFront = wheel_odom_w_buf.front();
//             wheel_odom_w_buf.pop();
//         }

//         odomBack = wheel_odom_w_buf.front();//front  < cur < back
//         m_wheel_w_odom.unlock();

//         double z = linear_w_insert(time,odomFront,odomBack);

//         if(abs(z) <= 0.05)
//         {
//             getPointCloudfromRGBD(image_rgb, image1,header);    //todo

//             wheel_odom_w_buf.pop();
//             m_wheel_w_odom.unlock();
//         }
//     }

// }
//todo



//todo 输出位姿使用
void sub_pose_callback(const nav_msgs::Odometry::ConstPtr &odomtry_msg)
{
    // default_random_engine a;
    // //c最小随机值，b最大随机值 
    // uniform_real_distribution<double> b(-0.01,0.01);
    // a.seed(time(0));


    if(!write_switch)
    {
        //FILE* out_File;
        out_File = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	    if(out_File == NULL)
		    printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

        Eigen::Matrix<double, 4, 4> pose;



		estimator.getPoseInWorldFrame(pose);
        Eigen::Quaterniond q(pose.block<3,3>(0,0));

        // if(out_File != NULL)
        // 	fprintf (out_File, "%f %f %f %f %f %f %f %f %f %f %f %f\n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
        // 														       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
        // 														       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
        // if(out_File != NULL)
        //     fprintf (out_File, "%f %f %f %f %f %f %f %f\n", odomtry_msg->header.stamp.toSec(), odomtry_msg->pose.pose.position.x,
        //                                                                 odomtry_msg->pose.pose.position.y,
        //                                                                 odomtry_msg->pose.pose.position.z,
        //                                                                 odomtry_msg->pose.pose.orientation.x, 
        //                                                                 odomtry_msg->pose.pose.orientation.y,
        //                                                                 odomtry_msg->pose.pose.orientation.z,
        //                                                                 odomtry_msg->pose.pose.orientation.w
        //                                                                 );



        if(out_File != NULL)
            fprintf (out_File, "%f %f %f %f %f %f %f %f\n", odomtry_msg->header.stamp.toSec(), pose(0,3),
                                                                        pose(1,3),
                                                                        pose(2,3),
                                                                        q.x(), 
                                                                        q.y(),
                                                                        q.z(),
                                                                        q.w());
        


        write_switch = true;
        //cout << "out_file 1" << endl;
    }
    else
    {
        // cout << "out_file 2" << endl;
        Eigen::Matrix<double, 4, 4> pose;

		estimator.getPoseInWorldFrame(pose);
        Eigen::Quaterniond q(pose.block<3,3>(0,0));
    
			// if(out_File != NULL)
			// 	fprintf (out_File, "%f %f %f %f %f %f %f %f %f %f %f %f\n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
			// 														       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
			// 														       pose(2,0), pose(2,1), pose(2,2),pose(2,3));

            // if(out_File != NULL)
            //     fprintf (out_File, "%f %f %f %f %f %f %f %f\n", odomtry_msg->header.stamp.toSec(), odomtry_msg->pose.pose.position.x,
            //                                                                 odomtry_msg->pose.pose.position.y,
            //                                                                 odomtry_msg->pose.pose.position.z,
            //                                                                 odomtry_msg->pose.pose.orientation.x, 
            //                                                                 odomtry_msg->pose.pose.orientation.y,
            //                                                                 odomtry_msg->pose.pose.orientation.z,
            //                                                                 odomtry_msg->pose.pose.orientation.w
            //                                                                 );

        if(out_File != NULL)
            fprintf (out_File, "%f %f %f %f %f %f %f %f\n", odomtry_msg->header.stamp.toSec(), pose(0,3),
                                                                        pose(1,3),
                                                                        pose(2,3),
                                                                        q.x(), 
                                                                        q.y(),
                                                                        q.z(),
                                                                        q.w());
        // cout << "out_file 2" << endl;
    }
    
    if(!ros::ok())
    {
        if(out_File != NULL)
		    fclose (out_File);
        // cout << "out_file 3.1" << endl;
    }
}

//todo 输出 /odom 使用
 void sub_odom_callback(const nav_msgs::Odometry::ConstPtr &wheel_odomtry_msg)
{   
     if(!write_odom_switch)
     {
        
         out_odom_File = fopen((OUTPUT_FOLDER + "/odom.txt").c_str(),"w");
 	    if(out_odom_File == NULL)
 		    printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
        
        
         Eigen::Quaterniond Q;
         Q.x() = wheel_odomtry_msg->pose.pose.orientation.x;
         Q.y() = wheel_odomtry_msg->pose.pose.orientation.y;
         Q.z() = wheel_odomtry_msg->pose.pose.orientation.z;
         Q.w() = wheel_odomtry_msg->pose.pose.orientation.w;
         Eigen::Matrix<double, 3, 3> pose(Q);
         if(out_odom_File != NULL)
 				fprintf (out_odom_File, "%f %f %f %f %f %f %f %f %f %f %f %f\n",pose(0,0), pose(0,1), pose(0,2),wheel_odomtry_msg->pose.pose.position.x,
 																	       pose(1,0), pose(1,1), pose(1,2),wheel_odomtry_msg->pose.pose.position.y,
 																	       pose(2,0), pose(2,1), pose(2,2),wheel_odomtry_msg->pose.pose.position.z);
         write_odom_switch = true;
         //cout << "out__odom_file 1" << endl;
     }
     else
     {
         Eigen::Quaterniond Q;
         Q.x() = wheel_odomtry_msg->pose.pose.orientation.x;
         Q.y() = wheel_odomtry_msg->pose.pose.orientation.y;
         Q.z() = wheel_odomtry_msg->pose.pose.orientation.z;
         Q.w() = wheel_odomtry_msg->pose.pose.orientation.w;
         Eigen::Matrix<double, 3, 3> pose(Q);
         if(out_odom_File != NULL)
 				fprintf (out_odom_File, "%f %f %f %f %f %f %f %f %f %f %f %f\n",pose(0,0), pose(0,1), pose(0,2),wheel_odomtry_msg->pose.pose.position.x,
 																	       pose(1,0), pose(1,1), pose(1,2),wheel_odomtry_msg->pose.pose.position.y,
 																	       pose(2,0), pose(2,1), pose(2,2),wheel_odomtry_msg->pose.pose.position.z);
         //cout << "out_odom_file 2" << endl;
     }
}





// queue<sensor_msgs::LaserScanConstPtr> laser_buf;
// std::mutex m_laser;

// void laser_callback(const sensor_msgs::LaserScanConstPtr &laser_msg)
// {
//     m_laser.lock();
//     laser_buf.push(laser_msg);
//     m_laser.unlock();
// }



//todo





// 获得左目的message
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

// 获得右目的message
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

// ros color image topic to cv::Mat mono image 
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")//https://blog.csdn.net/I_canjnu/article/details/124855613
    {// 如果编码格式不是MONO8的话将编码格式进行转换
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";//灰度图
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}



//todo
cv::Mat getColorImageFromMsg(const sensor_msgs::ImageConstPtr &image_msg)
{
    cv_bridge::CvImageConstPtr ptr;
                {
                    sensor_msgs::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "rgb8";//改
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
                }
                cv::Mat image = ptr->image;
    cv::Mat img = ptr->image.clone();
    return img;
}



cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    //depth has encoding TYPE_16UC1
    cv_bridge::CvImageConstPtr depth_ptr;
    //if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = sensor_msgs::image_encodings::MONO16;
        depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    }
    //else
        //depth_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO16);
    cv::Mat img = depth_ptr->image.clone();
    return img;
}


//! 得到点云插入point_rgbd在pubOctree中处理(RGBD)
void getRGBDFromMsg(const sensor_msgs::ImageConstPtr &image_msg,const sensor_msgs::ImageConstPtr &depth_msg)
{
    cv_bridge::CvImageConstPtr depth_ptr;
                //if (depth_msg->encoding == "8UC1")
                {
                    sensor_msgs::Image img;
                    img.header = depth_msg->header;
                    img.height = depth_msg->height;
                    img.width = depth_msg->width;
                    img.is_bigendian = depth_msg->is_bigendian;
                    img.step = depth_msg->step;
                    img.data = depth_msg->data;
                    img.encoding = sensor_msgs::image_encodings::MONO16;//改
                    depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
                }
                cv::Mat depth = depth_ptr->image;
    
    cv_bridge::CvImageConstPtr ptr;
                {
                    sensor_msgs::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "rgb8";//改
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
                }
                cv::Mat image = ptr->image;

    vector<Eigen::Matrix<float, 6, 1>> Points;

    for (int v = 1; v < ROW - 1; v += 1) {
        for (int u = 1; u < COL - 1; u += 1) {
            Eigen::Vector2d a(u, v);
            Eigen::Vector3d a_3d;
            estimator.featureTracker.m_camera[0]->liftProjective(a, a_3d);

             if (depth.ptr<unsigned short>(v)[u] < 170 || depth.ptr<unsigned short>(v)[u] >= 4000 ) continue;
            //  if (depth.ptr<unsigned short>(v - 1)[u] < 170 || depth.ptr<unsigned short>(v - 1)[u] >= 4000 ) continue;
            //  if (depth.ptr<unsigned short>(v + 1)[u] < 170 || depth.ptr<unsigned short>(v + 1)[u] >= 4000 ) continue;
            //  if (depth.ptr<unsigned short>(v)[u - 1] < 170 || depth.ptr<unsigned short>(v)[u - 1] >= 4000 ) continue;
            //  if (depth.ptr<unsigned short>(v)[u + 1] < 170 || depth.ptr<unsigned short>(v)[u + 1] >= 4000 ) continue;

            // float d = (depth.ptr<unsigned short>(v)[u] + depth.ptr<unsigned short>(v - 1)[u] +
            //     depth.ptr<unsigned short>(v)[u - 1] + depth.ptr<unsigned short>(v + 1)[u] +
            //     depth.ptr<unsigned short>(v)[u + 1]) / 5000.0;// *1.15;

            float d = (depth.ptr<unsigned short>(v)[u]) / 1000.0;

            float r = image.ptr<cv::Vec3b>(v)[u][0];
            float g = image.ptr<cv::Vec3b>(v)[u][1];
            float b = image.ptr<cv::Vec3b>(v)[u][2];
            //if (r > 240 && g > 240 && b > 240) continue;
           if (d <= 0.17 || d >= 4 ) continue;
            Eigen::Matrix<float, 6, 1> point;
            point << a_3d.x() * d, a_3d.y() * d, d, r, g, b;

            Points.push_back(point);
            //cout <<"r:"<< r <<" g:"<< g <<" b:"<< b <<" d:"<< d << endl;
        }
    }

    //     for (int v = 0; v < ROW ; v += 1) {
    //         for (int u = 0; u < COL; u += 1) {
    //             Eigen::Vector2d a(u, v);
    //             Eigen::Vector3d a_3d;
    //         estimator.featureTracker.m_camera[0]->liftProjective(a, a_3d);
    //             float d = depth.ptr<unsigned short>(v)[u] / 1000.0;// *1.15;
    //             float r = image.ptr<cv::Vec3b>(v)[u][0];
    //             float g = image.ptr<cv::Vec3b>(v)[u][1];
    //             float b = image.ptr<cv::Vec3b>(v)[u][2];
    //             //if (r > 240 && g > 240 && b > 240) continue;
    //             if (d < 0.17 || d >= 4.0 ) continue;
    //             Eigen::Matrix<float, 6, 1> point;
    //             point << a_3d.x() * d, a_3d.y() * d, d, r, g, b;

    //             Points.push_back(point);
    //             //cout <<"r:"<< r <<" g:"<< g <<" b:"<< b <<" d:"<< d << endl;
    //         }
    // }


    double t = image_msg->header.stamp.toSec();
    point_rgbd.insert(pair<double,vector<Eigen::Matrix<float, 6, 1>>>(t,Points));
    printf("insert point_rgbd\n");
}



//! 直接得到点云(RGBD)
void getPointCloudfromRGBD(cv::Mat &image , cv::Mat &depth , std_msgs::Header &header)
{
    sensor_msgs::PointCloud2 tmp_pcl;

    vector<Eigen::Matrix<float, 6, 1>> Points;

    for (int v = 2; v < ROW - 2; v += 5) {
        for (int u = 2; u < COL - 2; u += 5) {
            Eigen::Vector2d a(u, v);
            Eigen::Vector3d a_3d;
            estimator.featureTracker.m_camera[0]->liftProjective(a, a_3d);

             if (depth.ptr<unsigned short>(v)[u] < 170 || depth.ptr<unsigned short>(v)[u] >= 4000 ) continue;
             if (depth.ptr<unsigned short>(v - 1)[u] < 170 || depth.ptr<unsigned short>(v - 1)[u] >= 4000 ) continue;
             if (depth.ptr<unsigned short>(v + 1)[u] < 170 || depth.ptr<unsigned short>(v + 1)[u] >= 4000 ) continue;
             if (depth.ptr<unsigned short>(v)[u - 1] < 170 || depth.ptr<unsigned short>(v)[u - 1] >= 4000 ) continue;
             if (depth.ptr<unsigned short>(v)[u + 1] < 170 || depth.ptr<unsigned short>(v)[u + 1] >= 4000 ) continue;

            // float d = (depth.ptr<unsigned short>(v)[u] + depth.ptr<unsigned short>(v - 1)[u] +
            //     depth.ptr<unsigned short>(v)[u - 1] + depth.ptr<unsigned short>(v + 1)[u] +
            //     depth.ptr<unsigned short>(v)[u + 1]) / 5000.0;// *1.15;

            float d = (depth.ptr<unsigned short>(v)[u]) / 1000.0;

            float r = image.ptr<cv::Vec3b>(v)[u][0];
            float g = image.ptr<cv::Vec3b>(v)[u][1];
            float b = image.ptr<cv::Vec3b>(v)[u][2];
            //if (r > 240 && g > 240 && b > 240) continue;
           if (d <= 0.17 || d >= 4 ) continue;
            Eigen::Matrix<float, 6, 1> point;
            point << a_3d.x() * d, a_3d.y() * d, d, r, g, b;

            Eigen::Vector3d points(point[0], point[1], point[2]);
            
            Eigen::Vector3d pointOdom = estimator.ric[0] * points + estimator.tic[0];

			// if(pointOdom[2] < 0 || pointOdom[2] >= 0.8 || pointOdom[0] <= -1.5 || pointOdom[0] >= 1.5 || pointOdom[1] <= -1.5 || pointOdom[1] >= 1.5 || 0 <= pointOdom[0] <= 0.5 || 0 <= pointOdom[1] <= 0.5) continue;
			// if(pointOdom[2] >= 1.6 || pointOdom[0] <= -1.6 || pointOdom[0] >= 1.6 || pointOdom[1] <= -1.6 || pointOdom[1] >= 1.6) continue;
			// if(pointOdom[2] >= 2 || pointOdom[0] <= -2.5 || pointOdom[0] >= 2.5 || pointOdom[1] <= -2.5 || pointOdom[1] >= 2.5) continue;
			if(pointOdom[2] >= 2 || pointOdom[0] <= -1.5 || pointOdom[0] >= 1.5 || pointOdom[1] <= -1.5 || pointOdom[1] >= 1.5) continue;
			// if(pointOdom[2] >= 1.5 || pointOdom[0] <= -1 || pointOdom[0] >= 1 || pointOdom[1] <= -1 || pointOdom[1] >= 1) continue;
			//! 不滤波2.5   滤波？

			Eigen::Matrix4d pose;
			estimator.getPoseInWorldFrame(pose);

			
			Eigen::Matrix3d T;
			T << pose(0,0), pose(0,1), pose(0,2),
				pose(1,0), pose(1,1), pose(1,2),
				pose(2,0), pose(2,1), pose(2,2);
			
			Eigen::Vector3d P(pose(0,3),pose(1,3),pose(2,3));
			

			Eigen::Vector3d pointWorld = T * pointOdom + P;

			//  if(pointWorld[2] <= 0) continue;

			
			pcl::PointXYZRGB searchPoint;
			searchPoint.x = pointWorld.x();
			searchPoint.y = pointWorld.y();
			searchPoint.z = pointWorld.z();
			searchPoint.r = point[3];
			searchPoint.g = point[4];
			searchPoint.b = point[5];
			
			double min_x, min_y, min_z, max_x, max_y, max_z;

			octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
			bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);
			// if (isInBox) cout << "searchPoint In Box \n" << endl;
			if(isInBox && octree->getVoxelDensityAtPoint(searchPoint) < 1)//不太懂
			{
				
				octree->addPointToCloud(searchPoint, cloud);
				// tmp->points.push_back(searchPoint);
			}
        }
    }
	        //!统计滤波器
            // statistical_filter.setMeanK(2);//50  2
            // statistical_filter.setStddevMulThresh(0.8);//1  0.8
            // statistical_filter.setInputCloud(tmp);
            // statistical_filter.filter(*cloud);

			//!体素滤波
            // voxel_filter.setLeafSize(0.03,0.03,0.03);
            // voxel_filter.setInputCloud(tmp);
            // voxel_filter.filter(*cloud);

            //!半径滤波(kitti能用,d415不能)
            // radius_filter.setInputCloud(tmp);
            // // 设置滤波半径
            // radius_filter.setRadiusSearch(0.8);
            // // 设置滤波最少近邻数
            // radius_filter.setMinNeighborsInRadius (50);
            
            // radius_filter.filter(*cloud);

	// printf("生成点云ros消息\n");
	pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);
    pcl::io::savePCDFileBinary(OUTPUT_FOLDER + "/point_map.pcd", *cloud);
    

	tmp_pcl.header = header;
	tmp_pcl.header.frame_id = "map";
	pub_octree.publish(tmp_pcl);
	// printf("发送点云ros消息\n");

	// cloud->clear();

    // double t = header.stamp.toSec();
    // point_rgbd.insert(pair<double,vector<Eigen::Matrix<float, 6, 1>>>(t,Points));
}




//! 直接得到点云(stereo)
void getPointCloudfromStereo(cv::Mat &left , cv::Mat &right , std_msgs::Header &header)
{
    sensor_msgs::PointCloud2 tmp_pcl;

    int width = left.cols;
	int height = left.rows;
	int dim[3] = { width, height, width };
    cv::Mat disp_left = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
	cv::Mat disp_right = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);


    //! elas
    // 参数设置	
        Elas::parameters param;
        param.disp_min = 0;                                     // 最小视差	
        param.disp_max = 80;                                // 最大视差	   32  64 80 96 160   320
        param.support_threshold = 0.85;              // 比率测试：最低match VS 次低match
        param.support_texture = 10;                     // 支持点的最小纹理
        param.candidate_stepsize = 5;                  // 用于支持点的sobel特征匹配的邻域半径
        param.incon_window_size = 5;                  // 不连续性窗口的尺寸
        param.incon_threshold = 5;                       // 不连续性窗口内的视差范围阈值
        param.incon_min_support = 5;                 // 不连续性窗口内的最低支持点数量
        param.add_corners = true;                        // 是否添加角点
        param.grid_size = 20;                                  // 网格尺寸
        param.beta = 0.02;                                      // 图像相似性度量的参数
        param.gamma = 3;                                      // 先验概率常数
        param.sigma = 1;                                         // 先验概率的标准差
        param.sradius = 3;                                       // 标准差半径
        param.match_texture = 1;                         // 最低纹理
        param.lr_threshold = 1;                             // 左右一致性检验阈值
        param.speckle_sim_threshold = 1;          // 连通域判断阈值
        param.speckle_size = 200;                        // 连通域噪声尺寸判断阈值
        param.ipol_gap_width = 3;                       // 空洞宽
        param.filter_median = false;                     // 是否中值滤波
        param.filter_adaptive_mean = true;        // 是否自适应中值滤波
        param.postprocess_only_left = true;     // 是否只对左视差图后处理，设置为True可以节省时间
        param.subsampling = false;                     // 每个两个像素进行视差计算，设置为True可以节省时间，但是传入的D1和D2的分辨率必须为(w/2) x (h/2)

        Elas elas(param);
        elas.process(left.data, right.data,disp_left.ptr<float>(0), disp_right.ptr<float>(0), dim);

        //! openCV (6.15 kitti效果好些但还没完全测试以后还需重新改参数再试试，  d435不太行)
		// cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);// 神奇的参数  160  32  96
		// // cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,96);// 神奇的参数
    	// cv::Mat disparity_sgbm,disparity;//差距
    	// sgbm->compute(left,right,disparity_sgbm);
    	// disparity_sgbm.convertTo(disparity,CV_32F,1.0 / 16.0f);


        // cv::Mat disparity_show;
        // disparity_sgbm.convertTo(disparity_show,CV_32F,1.0 / 16.0f / 96.0f);
        // cv::imshow("disparity_show", disparity_show);
        // cv::imshow("disparity", disparity);
        // cv::imshow("disparity_sgbm", disparity_sgbm);


        // cv::Mat disp_left_show;
        // // disp_left.convertTo(disp_left_show,CV_32F,1.0 / 96.0f);
        // disp_right.convertTo(disp_left_show,CV_32F,1.0 / 8.0f);
        // cv::imshow("disp_left_show", disp_left_show);
        // cv::imshow("disp_left", disp_left);
        // cv::imshow("disp_right", disp_right);
		// cv::waitKey(10);
    
    //  vector<Eigen::Matrix<float, 6, 1>> Points;

    //!  将rgbd和双目点云对比能否消除残影、漂移？或者通过滤波？
    for (size_t v = 5; v < left.rows - 5; v+= 5)
    {
        for (size_t u = 5; u < left.cols - 5; u+= 5)
        {
            if (disp_left.at<float>(v,u) <= 0.0 || disp_left.at<float>(v,u) >= 80.0)   continue;
            // if (disparity.at<float>(v,u) <= 0.0 || disparity.at<float>(v,u) >= 96.0)   continue;

            Eigen::Vector2d a(u, v);
            Eigen::Vector3d a_3d;
            estimator.featureTracker.m_camera[0]->liftProjective(a, a_3d);

            //! elas
            double depth = 389.109 * 0.0524 / disp_left.at<float>(v,u);
            // double depth = 389.109 * 0.0524 / disp_right.at<float>(v,u);

            // ! OpenCV
			// double depth = 389.109 * 0.0524 / disparity.at<float>(v,u);

            Eigen::Matrix<float, 6, 1> point;
            point << a_3d.x() * depth, a_3d.y() * depth, depth, left.at<uchar>(v,u) / 255.0, left.at<uchar>(v,u) / 255.0, left.at<uchar>(v,u) / 255.0;

            // Points.push_back(point);

            Eigen::Vector3d points(point[0], point[1], point[2]);
            
            Eigen::Vector3d pointOdom = estimator.ric[0] * points + estimator.tic[0];
			//  if(pointOdom[2] <= 0 || pointOdom[2] >= 30|| pointOdom[0] <= 0 || pointOdom[0] >= 20 || pointOdom[1] <= -30 || pointOdom[1] >= 30) continue;
			// if(pointOdom[2] < 0 || pointOdom[2] >= 2 || pointOdom[0] <= -3.5 || pointOdom[0] >= 3.5 || pointOdom[1] <= -3.5 || pointOdom[1] >= 3.5) continue;
			if(pointOdom[2] < 0 || pointOdom[2] >= 0.8 || pointOdom[0] <= -1.5 || pointOdom[0] >= 1.5 || pointOdom[1] <= -1.5 || pointOdom[1] >= 1.5) continue;
			
			Eigen::Matrix4d pose;
			estimator.getPoseInWorldFrame(pose);

			
			Eigen::Matrix3d T;
			T << pose(0,0), pose(0,1), pose(0,2),
				pose(1,0), pose(1,1), pose(1,2),
				pose(2,0), pose(2,1), pose(2,2);
			
			Eigen::Vector3d P(pose(0,3),pose(1,3),pose(2,3));
			

			Eigen::Vector3d pointWorld = T * pointOdom + P;

			//  if(pointWorld[2] <= 0) continue;

			
			pcl::PointXYZRGB searchPoint;
			searchPoint.x = pointWorld.x();
			searchPoint.y = pointWorld.y();
			searchPoint.z = pointWorld.z();
			searchPoint.r = 0.8;
			searchPoint.g = 0.3;
			searchPoint.b = 0.3;
			
			double min_x, min_y, min_z, max_x, max_y, max_z;

			octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
			bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);
			//if (isInBox) cout << "searchPoint In Box \n" << endl;
			if(isInBox && octree->getVoxelDensityAtPoint(searchPoint) < 1)//不太懂
			{
				
				octree->addPointToCloud(searchPoint, cloud);
				// tmp->points.push_back(searchPoint);
			}
        }
    }

        //! 滤波感觉确实能消除部分残影但无法做到实时，然后点云看起来也一般，还需要调试
	        //!统计滤波器
            // statistical_filter.setMeanK(2);//50  2
            // statistical_filter.setStddevMulThresh(0.8);//1  0.8
            // statistical_filter.setInputCloud(tmp);
            // statistical_filter.filter(*cloud);

			//!体素滤波
            // voxel_filter.setLeafSize(0.03,0.03,0.03);
            // voxel_filter.setInputCloud(tmp);
            // voxel_filter.filter(*cloud);

            //!半径滤波(kitti能用,d415不能)
            // radius_filter.setInputCloud(tmp);
            // // 设置滤波半径
            // radius_filter.setRadiusSearch(0.8);
            // // 设置滤波最少近邻数
            // radius_filter.setMinNeighborsInRadius (2);
            
            // radius_filter.filter(*cloud);

	// printf("生成点云ros消息\n");
	pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);

	tmp_pcl.header = header;
	tmp_pcl.header.frame_id = "map";
	pub_octree.publish(tmp_pcl);
	// printf("发送点云ros消息\n");

	// cloud->clear();
	// tmp->clear();

    // double t = header.stamp.toSec();
    // point_rgbd.insert(pair<double,vector<Eigen::Matrix<float, 6, 1>>>(t,Points));
}





//! 得到点云插入point_rgbd在pubOctree中处理(stereo)
// void getPointCloudfromStereo(cv::Mat &left , cv::Mat &right , double &t)
// {
//     sensor_msgs::PointCloud2 tmp_pcl;

//     int width = left.cols;
// 	int height = left.rows;
// 	int dim[3] = { width, height, width };
//     cv::Mat disp_left = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
// 	cv::Mat disp_right = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);

//     //! elas
//     // 参数设置	
//         Elas::parameters param;
//         param.disp_min = 0;                                     // 最小视差	
//         param.disp_max = 320;                                // 最大视差	   32  64 80 96 160   320
//         param.support_threshold = 0.85;              // 比率测试：最低match VS 次低match
//         param.support_texture = 10;                     // 支持点的最小纹理
//         param.candidate_stepsize = 5;                  // 用于支持点的sobel特征匹配的邻域半径
//         param.incon_window_size = 5;                  // 不连续性窗口的尺寸
//         param.incon_threshold = 5;                       // 不连续性窗口内的视差范围阈值
//         param.incon_min_support = 5;                 // 不连续性窗口内的最低支持点数量
//         param.add_corners = true;                        // 是否添加角点
//         param.grid_size = 20;                                  // 网格尺寸
//         param.beta = 0.02;                                      // 图像相似性度量的参数
//         param.gamma = 3;                                      // 先验概率常数
//         param.sigma = 1;                                         // 先验概率的标准差
//         param.sradius = 3;                                       // 标准差半径
//         param.match_texture = 1;                         // 最低纹理
//         param.lr_threshold = 1;                             // 左右一致性检验阈值
//         param.speckle_sim_threshold = 1;          // 连通域判断阈值
//         param.speckle_size = 200;                        // 连通域噪声尺寸判断阈值
//         param.ipol_gap_width = 3;                       // 空洞宽
//         param.filter_median = false;                     // 是否中值滤波
//         param.filter_adaptive_mean = true;        // 是否自适应中值滤波
//         param.postprocess_only_left = true;     // 是否只对左视差图后处理，设置为True可以节省时间
//         param.subsampling = false;                     // 每个两个像素进行视差计算，设置为True可以节省时间，但是传入的D1和D2的分辨率必须为(w/2) x (h/2)

//         Elas elas(param);
//         elas.process(left.data, right.data,disp_left.ptr<float>(0), disp_right.ptr<float>(0), dim);

//         //! openCV (6.15 kitti效果好些但还没完全测试以后还需重新改参数再试试，  d435不太行)
// 		// cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);// 神奇的参数  160  32  96
// 		// // cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,96);// 神奇的参数
//     	// cv::Mat disparity_sgbm,disparity;//差距
//     	// sgbm->compute(left,right,disparity_sgbm);
//     	// disparity_sgbm.convertTo(disparity,CV_32F,1.0 / 16.0f);


//         // cv::Mat disparity_show;
//         // disparity_sgbm.convertTo(disparity_show,CV_32F,1.0 / 16.0f / 96.0f);
//         // cv::imshow("disparity_show", disparity_show);
//         // cv::imshow("disparity", disparity);
//         // cv::imshow("disparity_sgbm", disparity_sgbm);


//         // cv::Mat disp_left_show;
//         // // disp_left.convertTo(disp_left_show,CV_32F,1.0 / 96.0f);
//         // disp_right.convertTo(disp_left_show,CV_32F,1.0 / 8.0f);
//         // cv::imshow("disp_left_show", disp_left_show);
//         // cv::imshow("disp_left", disp_left);
//         // cv::imshow("disp_right", disp_right);
// 		// cv::waitKey(10);
    
//      vector<Eigen::Matrix<float, 6, 1>> Points;

//     //!  将rgbd和双目点云对比能否消除残影、漂移？或者通过滤波？
//     for (size_t v = 5; v < left.rows - 5; v+= 5)
//     {
//         for (size_t u = 5; u < left.cols - 5; u+= 5)
//         {
//             if (disp_left.at<float>(v,u) <= 0.0 || disp_left.at<float>(v,u) >= 320.0)   continue;
//             // if (disparity.at<float>(v,u) <= 0.0 || disparity.at<float>(v,u) >= 96.0)   continue;

//             Eigen::Vector2d a(u, v);
//             Eigen::Vector3d a_3d;
//             estimator.featureTracker.m_camera[0]->liftProjective(a, a_3d);

//             //! elas
//             double depth = 389.109 * 0.0524 / disp_left.at<float>(v,u);
//             // double depth = 389.109 * 0.0524 / disp_right.at<float>(v,u);

//             // ! OpenCV
// 			// double depth = 389.109 * 0.0524 / disparity.at<float>(v,u);

//             Eigen::Matrix<float, 6, 1> point;
//             // point << a_3d.x() * depth, a_3d.y() * depth, depth, left.at<uchar>(v,u) / 255.0, left.at<uchar>(v,u) / 255.0, left.at<uchar>(v,u) / 255.0;
//             point << a_3d.x() * depth, a_3d.y() * depth, depth, 0.8, 0, 0;

//             Points.push_back(point);
//         }
//     }

//     point_rgbd.insert(pair<double,vector<Eigen::Matrix<float, 6, 1>>>(t,Points));
// }
//todo




 
// extract images with same timestamp from two topics，从两个图像队列中取出最早的一帧，并从队列删除，双目要求两帧时差不得超过0.003s
void sync_process()
{
    while(1)
    {
        if(STEREO || DEPTH)//todo
        {
            cv::Mat image0, image1, image_rgb;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();//front 返回值为队列中的第一个元素，也就是最早、最先进入队列的元素
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 )//todo rgbd:0   stereo - 0.003
                {
                    img0_buf.pop();//将队列中最靠前位置的元素拿掉
                    printf("throw img0\n");
                }
                else if(time0 > time1 )//todo rgbd:0   stereo + 0.003
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                   printf("rgbd\n");


//todo
                    if(DEPTH)
                    {
                        image_rgb = getColorImageFromMsg(img0_buf.front());
                        image1 = getDepthImageFromMsg(img1_buf.front());
                        // getRGBDFromMsg(img0_buf.front(),img1_buf.front());  //todo
                        img1_buf.pop();
                        printf("depth\n");
                    }
                    else
                    {
                        image1 = getImageFromMsg(img1_buf.front());
                        img1_buf.pop();
                    }

                    img0_buf.pop();
//todo
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
            {
                printf("start\n");
                estimator.inputImage(time, image0, image1);//前端线程追踪 //todo stereo
                // estimator.inputImage(time, image_rgb, image1);//前端线程追踪 //todo stereo

                // estimator.inputImage(time, image0);//todo rgbd  
                
                // getPointCloudfromStereo(image0, image1, header);  //! stereo  this cpp

                // getPointCloudfromStereo(image0, image1, time);  //todo  stereo

                // getPointCloudfromRGBD(image_rgb, image1, header);//rgbd

                //! 消除rgbd残影用
                nav_msgs::Odometry::ConstPtr odomFront;
                nav_msgs::Odometry::ConstPtr odomBack;
                if(time < wheel_odom_w_buf.front()->header.stamp.toSec() || time > wheel_odom_w_buf.back()->header.stamp.toSec())
                {   
                    // cout << "旋转 1" << endl;
                    // if(abs(odom_w_z[time]) <= 0.05)
                    // {
                        // getPointCloudfromRGBD(image_rgb, image1, header);    //todo
                        // cout << "旋转 2" << endl;
                    // }
                }
                else
                {
                    m_wheel_w_odom.lock();
                    while (wheel_odom_w_buf.front()->header.stamp.toSec() <= time)
                    {
                        if (wheel_odom_w_buf.front()->header.stamp.toSec() == time)
                        {
                            if(abs(wheel_odom_w_buf.front()->twist.twist.angular.z) <= 0.04)//! 现在（6.26）为0.05 以后可调
                            {
                                getPointCloudfromRGBD(image_rgb, image1, header);    //todo
                                // cout << "旋转 3" << endl;
                                wheel_odom_w_buf.pop();
                                m_wheel_w_odom.unlock();
                            }
                        }

                        odomFront = wheel_odom_w_buf.front();
                        wheel_odom_w_buf.pop();
                    }

                    odomBack = wheel_odom_w_buf.front();//front  < cur < back
                    m_wheel_w_odom.unlock();

                    double z = linear_w_insert(time,odomFront,odomBack);

                    if(abs(z) <= 0.04)                  //! 现在（6.26）为0.05 以后可调
                    {
                        getPointCloudfromRGBD(image_rgb, image1, header);    //todo
                        // cout << "旋转 4" << endl;
                    }
                }


            }
                
        }
        else//单目
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);		
    }
//todo	
}

//其中imu_callback中订阅imu信息，并将器填充到accBuf和gyrBuf中，之后执行了vins_estimator/src/estimator/estimator.cpp中inputIMU函数的fastPredictIMU、pubLatestOdometry函数
//fastPredictIMU使用上一时刻的姿态进行快速的imu预积分，这个信息根据processIMU的最新数据Ps[frame_count]、Rs[frame_count]、Vs[frame_count]、Bas[frame_count]、Bgs[frame_count]来进行预积分，从而保证信息能够正常发布。
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    /*
    *  inputIMU函数：
    * 1. 积分预测当前帧状态，latest_time，latest_Q，latest_P，latest_V，latest_acc_0，latest_gyr_0
    * 2. 发布里程计
    */
    return;
}

//订阅一帧跟踪的特征点，包括3D坐标、像素坐标、速度，交给estimator处理
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> featureFrame;//todo 7
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {// 数据格式为feature_id camera_id xyz_uv_velocity
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i]; //特征点的像素速度
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)//这啥？？？？
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);   // 检查是不是归一化
        Eigen::Matrix<double, 9, 1> xyz_uv_velocity;//todo 7
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, 0, 0;//todo 
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//emplace_back能就地通过参数构造对象，不需要拷贝或者移动内存，相比push_back能更好地避免内存的拷贝与移动，使容器插入元素的性能得到进一步提升。
        //https://blog.csdn.net/weixin_45880571/article/details/119450328,https://blog.csdn.net/weixin_44718794/article/details/108321232?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-108321232-blog-119282296.pc_relevant_3mothn_strategy_recovery&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-108321232-blog-119282296.pc_relevant_3mothn_strategy_recovery&utm_relevant_index=1
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}


// 是否重启estimator，并重新设置参数
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        /*
        *  清理状态，缓存数据、变量、滑动窗口数据、位姿等
        *  系统重启或者滑窗优化失败都会调用
        */
        estimator.clearState();
        estimator.setParameter();//为求解器设置参数
    }
    return;
}
// 是否使用IMU
void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

//单双目切换
void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//设置记录器级别ROS控制台默认名称
  /*
    * 设置消息日志级别，默认有Debug,Info,Warn,Error,Fatal
    * Debug : 输出程序正常运行需要的信息
    * Info : 输出大量用户需要的信息
    * Warn : 输出警告，或许影响程序的应用，但系统仍处于可控的预期状态
    * Error : 输出严重错误（但错误可恢复）
    * Fatal : 输出不可恢复的崩溃式错误
    */

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);



//!!!!!!!!!!!!!
    // FILE* input_file;
	// input_file = std::fopen("/home/xuwenyu/wheel_anomaly/anomaly.txt", "r");
	// if(input_file == NULL){
	//     printf("cannot find file");
	//     ROS_BREAK();
	//     return 0;          
	// }
	// double t,x,y,z,q_x,q_y,q_z,q_w;

	// while ( fscanf(input_file, "%f %f %f %f %f %f %f %f", &t, &x, &y, &z, &q_x, &q_y, &q_z, &q_w) != EOF)
	// {
	    
	// }
	// std::fclose(input_file);
//!!!!!!!!!!!!!!!!!!!!
//todo
        // FILE* out_File;
        // out_File = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	    // if(out_File == NULL)
		//     printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

        // cout << "out_file 1" << endl;


    //设置点云对象
	octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>( 0.01 );
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	tmp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	octree->setInputCloud(cloud);
	octree->addPointsFromInputCloud();
	octree->defineBoundingBox(-1000000, -1000000, -1000000, 1000000, 1000000, 1000000);
//todo

    readParameters(config_file);
    estimator.setParameter();// 设置参数
// 当setParameter()时候，就开启了一个Estimator类内的新线程：processMeasurements();
// processMeasurements();处理各种buffer里面的东西
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n); // 注册vins_estimator节点，在次节点下发布话题

    ros::Subscriber sub_imu;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());// ros::TransportHints().tcpNoDelay()使用这个参数会使发送端的缓冲数据在较少的情况下也会阻塞,相当于减少了发送端缓冲队列的拥挤程度
        //ros::TransportHints().tcpNoDelay(false/true)，用于设置是否对tcp进行较低延迟的传输
    }
//!!!!!!!!!!!!!!!!!!!!
    // ifstream fin("/home/xuwenyu/wheel_anomaly/anomaly.txt");
    // int num = 0;
	// while (!fin.eof())
	// {
    //     double t,x,y,z,q_x,q_y,q_z,q_w;
	//     // Eigen::Quaterniond Q;
	// 	// Eigen::Vector3d T;
	// 	fin >> t >> x >> y >> z >> q_x >> q_y >> q_z >>q_w;
	// 	//cout << "load odom 1" << endl;
	// 	nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
	// 	odom->header.stamp = ros::Time(t);
	// 	odom->header.frame_id = "odom";
	// 	odom->child_frame_id = "base_footprint";

	// 	odom->pose.pose.position.x = x;
	// 	odom->pose.pose.position.y = y;
	// 	odom->pose.pose.position.z = z;

	// 	odom->pose.pose.orientation.x = q_x;
	// 	odom->pose.pose.orientation.y = q_y;
	// 	odom->pose.pose.orientation.z = q_z;
	// 	odom->pose.pose.orientation.w = q_w;
	// 	//pulish_odom.publish(odom);
	// 	//nav_msgs::Odometry::ConstPtr odom_ptr(odom);
	// 	m_wheel_odom.lock();
	// 	wheel_odom_buf.push(odom);
	// 	m_wheel_odom.unlock();
	// 	num++;
	// 	if(num == 233) break;//!  00=4541 01=1101 02=4661 04=271  05=2761  06=1101
	// 	//cout << num << endl;
	// 	//cout << imageTimeList[num] << endl;
	// 	//cout << "x: " << T(0) << "y: " << T(1) << endl; 
	// }
//!!!!!!!!!!!!!!!!!!!!
    //todo
    // ros::Subscriber sub_wheel_odom = n.subscribe(ODOM_TOPIC,2000,wheel_odom_callback);//提供初值和约束用//!!!!!!!!!!!!!!!!!!!!
    //ros::Subscriber sub_lser = n.subscribe(LASER_TOPIC,2000,laser_callback);//雷达

    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/odometry",2000,sub_pose_callback);//输出位姿用
    // ros::Subscriber sub_odom = n.subscribe("/odom",2000,sub_odom_callback);//todo 原始odom位姿输出用
    //todo

    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);// 订阅一帧跟踪的特征点，没有找到对应的发布者,实际上应该是没用到，实际上是使用featureBuf进行传递
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    if(STEREO || DEPTH)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }

    //todo
    ros::Subscriber sub_wheel_odom = n.subscribe(ODOM_TOPIC,2000,wheel_odom_callback);//提供初值和约束用
    // //ros::Subscriber sub_lser = n.subscribe(LASER_TOPIC,2000,laser_callback);//雷达

    // ros::Subscriber sub_pose = n.subscribe("/vins_estimator/odometry",100,sub_pose_callback);//输出位姿用
    // ros::Subscriber sub_odom = n.subscribe("/odom",2000,sub_odom_callback);//  !   原始odom位姿输出用

    pub_octree = n.advertise<sensor_msgs::PointCloud2>("octree", 1000);//todo
    //todo


    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    

    std::thread sync_thread{sync_process}; //创建sync_thread线程，指向sync_process，这里边处理了processMeasurements的线程



    ros::spin();
    // 如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是
    // 立刻就被处理，而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用，这就是消息回到函数的原理

//todo
    if(!ros::ok())
    {
        if(out_File != NULL)
            fclose (out_File);
        // cout << "out_file 3.2" << endl;
    }

    if(!ros::ok())
    {
        if(out_odom_File != NULL)
            fclose (out_odom_File);
       cout << "out_odom_file 3.2" << endl;
    }
//todo

    return 0;
}
