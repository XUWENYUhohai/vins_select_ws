/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"
//todo
#include <iostream>
#include <stdio.h>
#include <string>
//todo
#include <unistd.h> 

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher pub_image_track;

//todo
// ros::Publisher pub_octree;
// pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>* octree;
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;
// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
// pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
// pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_filter;

// std::mutex m_octree;

// map<double,vector<Eigen::Matrix<float, 6, 1>>> point_rgbd;

// std::mutex m_wheel_w_odom;
// queue<nav_msgs::Odometry::ConstPtr> wheel_odom_w_buf;
// map<double,double> odom_w_z;
//FILE* out_File;
//todo


CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;


// 将节点n中的信息发布
/*** advertise（）函数是告诉ROS您要在给定主题名称上发布的方式。这将调用对ROS主节点的调用，
   * 该调用将保留谁正在发布和正在订阅的注册表。进行了advertise（）调用后，主节点将通知任何
   * 试图订阅此主题名称的人，然后他们将与此节点协商对等连接。 advertise（）返回一个Publisher
   * 对象，该对象使您可以通过调用publish（）来发布有关该主题的消息。一旦销毁了返回的Publisher对象
   * 的所有副本，该主题将自动取消发布。 advertise（）的第二个参数是用于发布消息的消息队列的大小。
   * 如果消息发布的速度快于我们发送消息的速度，则此处的数字指定在丢弃一些消息之前要缓冲多少条消息。
   */
void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);//todo 3.15 2
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);
//todo
    // pub_octree = n.advertise<sensor_msgs::PointCloud2>("octree", 100);

    //设置点云对象
    // octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>( 0.01 );
    // cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    // tmp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());



    // octree->setInputCloud(cloud);
    // octree->addPointsFromInputCloud();
    // octree->defineBoundingBox(-100000, -100000, -100000, 100000, 100000, 100000);

    // statistical_filter.setMeanK(2);//50
    // statistical_filter.setStddevMulThresh(1.6);//1
    // statistical_filter.setInputCloud(tmp);


    // printf("设置点云对象\n");k
//todo
    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
}


//todo
// void pubOctreefromRGBD(const Estimator &estimator, const std_msgs::Header &header)
// {
//     //cout << "pubOctree1" << endl;
//     if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
//         //m_octree.lock();
//          //tmp->clear();//!

//         sensor_msgs::PointCloud2 tmp_pcl;

//         if(point_rgbd.find(header.stamp.toSec()) != point_rgbd.end())
//         {
//             //cout << "pubOctree2" << endl;
//             for(auto &points : point_rgbd[header.stamp.toSec()])
//             {   
//                 //cout << "pubOctree3" << endl;
//                 // cout << "测试1" << endl;
//                 // if(odom_w_z.find(header.stamp.toSec()) != odom_w_z.end())
//                 // {
//                 //     cout << "测试2" << endl;
//                 //     if(abs(odom_w_z[header.stamp.toSec()]) > 0.05)
//                 //     {
//                 //         // map<double,double>::iterator pos = odom_w_z.find(header.stamp.toSec());

//                 //         // m_wheel_w_odom.lock();
//                 //         // odom_w_z.erase(odom_w_z.begin(),pos);
//                 //         // m_wheel_w_odom.unlock();
//                 //         cout << "丢弃此帧点云" << endl;
//                 //         continue;
//                 //     }
//                 // }

//                 Eigen::Vector3d point(points[0], points[1], points[2]);
                
//                 Eigen::Vector3d pointOdom = estimator.ric[0] * point + estimator.tic[0];

//                 // if (pointOdom.x() < 0.2 || pointOdom.x() > 4.1 || pointOdom.z() < 0.0 || pointOdom.z() > 0.3)
//                 // if (pointOdom.x() <= 0.17 || pointOdom.x() >= 4 || pointOdom.z() < 0.0 || pointOdom.z() >= 0.3 || pointOdom.y() <= -0.7 || pointOdom.y() >= 0.7)
//                 // if (pointOdom.x() >= 4 || pointOdom.z() < 0.0 || pointOdom.z() >= 0.3 || pointOdom.y() <= -0.75 || pointOdom.y() >= 0.75)
//                 //if (pointOdom.x() < 0.27 || pointOdom.x() > 4.1 )
//                     // continue;

//                 Eigen::Vector3d pointWorld = estimator.Rs[WINDOW_SIZE] *(pointOdom) + estimator.Ps[WINDOW_SIZE];

//                 // if (pointWorld.z() >= 0.3 || pointWorld.z() < 0.0)
//                     //  continue;
//                 pcl::PointXYZRGB searchPoint;
//                 searchPoint.x = pointWorld.x();
//                 searchPoint.y = pointWorld.y();
//                 searchPoint.z = pointWorld.z();
//                 searchPoint.r = points[3];
//                 searchPoint.g = points[4];
//                 searchPoint.b = points[5];

//                 double min_x, min_y, min_z, max_x, max_y, max_z;
//                 octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
//                 bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);
//                 //if (isInBox) cout << "searchPoint In Box \n" << endl;
//                 if(isInBox && octree->getVoxelDensityAtPoint(searchPoint) < 1)//不太懂
//                 {
//                     octree->addPointToCloud(searchPoint, cloud);
//                     // tmp->points.push_back(searchPoint);
//                 }
//             }
//             // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
//             // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
//             //!统计滤波器
//             // statistical_filter.setMeanK(50);
//             // statistical_filter.setStddevMulThresh(1.0);
//             // statistical_filter.setInputCloud(tmp);
//             // statistical_filter.filter(*cloud);

//             //!体素滤波
//             // voxel_filter.setLeafSize(0.03,0.03,0.03);
//             // voxel_filter.setInputCloud(tmp);
//             // voxel_filter.filter(*cloud);

//             //!半径滤波
//             // radius_filter.setInputCloud(tmp);
//             // // 设置滤波半径
//             // radius_filter.setRadiusSearch(0.8);
//             // // 设置滤波最少近邻数
//             // radius_filter.setMinNeighborsInRadius (2);
            
//             // radius_filter.filter(*cloud);


//             printf("生成点云ros消息\n");
//             pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);
//             //m_octree.unlock();
            
//             tmp_pcl.header = header;
//             tmp_pcl.header.frame_id = "map";
//             pub_octree.publish(tmp_pcl);
//             printf("发送点云ros消息\n");
//             //m_octree.unlock();


//             // m_octree.lock();
//             // octree->deleteTree();
//             // cloud->clear();
//             // tmp->clear();
//            // map<double,vector<Eigen::Matrix<float, 6, 1>>>::iterator pos = point_rgbd.find(header.stamp.toSec());
//             //point_rgbd.erase(pos);
//             // m_octree.unlock();
//             //printf("删除点云ros消息\n");
            
//         }
//     }
// }


// void pubOctreefromStereo(const Estimator &estimator, const std_msgs::Header &header)
// {
//     //cout << "pubOctree1" << endl;
//     if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
//         //m_octree.lock();
//          //tmp->clear();//!

//         sensor_msgs::PointCloud2 tmp_pcl;

//         if(point_rgbd.find(header.stamp.toSec()) != point_rgbd.end())
//         {
//             //cout << "pubOctree2" << endl;
//             for(auto &points : point_rgbd[header.stamp.toSec()])
//             {   
//                 //cout << "pubOctree3" << endl;
//                 // cout << "测试1" << endl;
//                 // if(odom_w_z.find(header.stamp.toSec()) != odom_w_z.end())
//                 // {
//                 //     cout << "测试2" << endl;
//                 //     if(abs(odom_w_z[header.stamp.toSec()]) > 0.05)
//                 //     {
//                 //         // map<double,double>::iterator pos = odom_w_z.find(header.stamp.toSec());

//                 //         // m_wheel_w_odom.lock();
//                 //         // odom_w_z.erase(odom_w_z.begin(),pos);
//                 //         // m_wheel_w_odom.unlock();
//                 //         cout << "丢弃此帧点云" << endl;
//                 //         continue;
//                 //     }
//                 // }

//                 Eigen::Vector3d point(points[0], points[1], points[2]);
                
//                 Eigen::Vector3d pointOdom = estimator.ric[0] * point + estimator.tic[0];

//                 // if (pointOdom.x() < 0.2 || pointOdom.x() > 4.1 || pointOdom.z() < 0.0 || pointOdom.z() > 0.3)
//                 // if (pointOdom.x() <= 0.17 || pointOdom.x() >= 4 || pointOdom.z() < 0.0 || pointOdom.z() >= 0.3 || pointOdom.y() <= -0.7 || pointOdom.y() >= 0.7)
//                 // if (pointOdom.x() >= 4 || pointOdom.z() < 0.0 || pointOdom.z() >= 0.3 || pointOdom.y() <= -0.75 || pointOdom.y() >= 0.75)
//                 //if (pointOdom.x() < 0.27 || pointOdom.x() > 4.1 )
//                     // continue;

//                 Eigen::Vector3d pointWorld = estimator.Rs[WINDOW_SIZE] *(pointOdom) + estimator.Ps[WINDOW_SIZE];

//                 // if (pointWorld.z() >= 0.3 || pointWorld.z() < 0.0)
//                     //  continue;
//                 pcl::PointXYZRGB searchPoint;
//                 searchPoint.x = pointWorld.x();
//                 searchPoint.y = pointWorld.y();
//                 searchPoint.z = pointWorld.z();
//                 searchPoint.r = points[3];
//                 searchPoint.g = points[4];
//                 searchPoint.b = points[5];

//                 double min_x, min_y, min_z, max_x, max_y, max_z;
//                 octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
//                 bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);
//                 //if (isInBox) cout << "searchPoint In Box \n" << endl;
//                 if(isInBox && octree->getVoxelDensityAtPoint(searchPoint) < 1)//不太懂
//                 {
//                     octree->addPointToCloud(searchPoint, cloud);
//                     // tmp->points.push_back(searchPoint);
//                 }
//             }
//             // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
//             // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
//             //!统计滤波器
//             // statistical_filter.setMeanK(50);
//             // statistical_filter.setStddevMulThresh(1.0);
//             // statistical_filter.setInputCloud(tmp);
//             // statistical_filter.filter(*cloud);

//             //!体素滤波
//             // voxel_filter.setLeafSize(0.03,0.03,0.03);
//             // voxel_filter.setInputCloud(tmp);
//             // voxel_filter.filter(*cloud);

//             //!半径滤波
//             // radius_filter.setInputCloud(tmp);
//             // // 设置滤波半径
//             // radius_filter.setRadiusSearch(0.8);
//             // // 设置滤波最少近邻数
//             // radius_filter.setMinNeighborsInRadius (2);
            
//             // radius_filter.filter(*cloud);


//             printf("生成点云ros消息\n");
//             pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);
//             //m_octree.unlock();
            
//             tmp_pcl.header = header;
//             tmp_pcl.header.frame_id = "map";
//             pub_octree.publish(tmp_pcl);
//             printf("发送点云ros消息\n");
//             //m_octree.unlock();


//             // m_octree.lock();
//             // octree->deleteTree();
//             // cloud->clear();
//             // tmp->clear();
//            // map<double,vector<Eigen::Matrix<float, 6, 1>>>::iterator pos = point_rgbd.find(header.stamp.toSec());
//             //point_rgbd.erase(pos);
//             // m_octree.unlock();
//             //printf("删除点云ros消息\n");
            
//         }
//     }
// }
//todo



//发送最新的位姿结果
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";//todo world
    // odometry.child_frame_id = "robot";//todo 无
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}



void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}



// 打印Estimator中的一些数据
void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
            ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}



// 发布里程计消息
void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";//todo world
        odometry.child_frame_id = "world";//todo world
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";//todo world
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";//todo world
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app); //定义从内存到文件的流
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);//https://www.shuzhiduo.com/A/RnJW7PMYJq/  设置浮点数精度
        foutC << header.stamp.toSec() * 1e9 << ",";//时间 纳秒单位
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","//当前帧的速度
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        printf("time: %f, t: %f %f %f q: %f %f %f %f\n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
                                                          tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());//todo



//todo
        // outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	    // if(outFile == NULL)
		//     printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

        // if(out_File != NULL)
		// 	fprintf (out_File, "%f %f %f %f %f %f %f %f %f %f %f %f\n",estimator.Rs[WINDOW_SIZE](0,0), estimator.Rs[WINDOW_SIZE](0,1), estimator.Rs[WINDOW_SIZE](0,2),estimator.Ps[WINDOW_SIZE].x(),
		// 															       estimator.Rs[WINDOW_SIZE](1,0), estimator.Rs[WINDOW_SIZE](1,1), estimator.Rs[WINDOW_SIZE](1,2),estimator.Ps[WINDOW_SIZE].y(),
		// 															       estimator.Rs[WINDOW_SIZE](2,0), estimator.Rs[WINDOW_SIZE](2,1), estimator.Rs[WINDOW_SIZE](2,2),estimator.Ps[WINDOW_SIZE].z());
        // cout << "out_file 2" << endl;
        // if(outFile != NULL)
		//     fclose (outFile);
//todo
    }
}



// 发布关键帧的消息
void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";//todo world
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;//球序列
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}


// 发布相机位姿的消息
void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;//就是这样的，经过优化和滑窗以后的此刻，WINDOW_SIZE - 1和WINDOW_SIZE 拥有相同值

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";//todo world
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        if(STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

// 发布点云数据
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;


    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;//相机坐标系下的 位置
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )//发布点的满足的条件：已经初始化、第一次观测的图像帧在第0帧，被观测的图像帧数小于2
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}



// 发布坐标系的相关消息
void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));//todo world

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";//todo world
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
}



void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)//MARGIN_OLD = 0
    {//MARGIN_OLD这样的情况下关键帧为倒数第二帧（WINDOW_SIZE - 2）
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        odometry.header.frame_id = "world";//todo world
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";//todo world
        // 能被 WINDOW_SIZE - 2帧看到并且是有效的地图点
        for (auto &it_per_id : estimator.f_manager.feature)  //首帧＜last second帧  最后一帧观测帧>=last second帧
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;// 相机坐标系下的坐标
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];// 转到世界坐标系
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d; // 在该帧相机坐标系下的归一化坐标以及像素坐标
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d); //路标点在WINDOW_SIZE - 2图像帧中的信息
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}
