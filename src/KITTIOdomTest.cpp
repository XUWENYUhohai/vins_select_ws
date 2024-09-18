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

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"
//todo
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
using namespace std;
using namespace Eigen;

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

//todo，这个现在在feature_manager.h中定义(换到parameters.h中了)
string odom_file = "/home/xuwenyu/06/KITTI_06_gt.txt";//!!!!!!!!!!!!!!!!!!!

queue<nav_msgs::Odometry::ConstPtr> wheel_odom_buf;
std::mutex m_wheel_odom;

// map<double,vector<Eigen::Matrix<float, 6, 1>>> point_rgbd;

//todo
ros::Publisher pub_octree;
pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>* octree;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;



pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;
pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_filter;

// std::mutex m_octree;

// map<double,vector<Eigen::Matrix<float, 6, 1>>> point_rgbd;

// std::mutex m_wheel_w_odom;
// queue<nav_msgs::Odometry::ConstPtr> wheel_odom_w_buf;
// map<double,double> odom_w_z;
//FILE* out_File;
//todo


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
        // Elas::parameters param;
		// param.postprocess_only_left = true;				// 是否只对左视差图后处理，设置为True可以节省时间
        // param.disp_min = 0;                                     // 最小视差	//todo原0   5   0
        // param.disp_max = 320;                                // 最大视差	//todo原256  64     96
        // param.support_threshold = 0.85;              // 比率测试：最低match VS 次低match
        // param.support_texture = 10;                     // 支持点的最小纹理
        // param.candidate_stepsize = 5;                  // 用于支持点的sobel特征匹配的邻域半径
        // param.incon_window_size = 5;                  // 不连续性窗口的尺寸
        // param.incon_threshold = 5;                       // 不连续性窗口内的视差范围阈值
        // param.incon_min_support = 5;                 // 不连续性窗口内的最低支持点数量
        // param.add_corners = true;                        // 是否添加角点
        // param.grid_size = 20;                                  // 网格尺寸
        // param.beta = 0.02;                                      // 图像相似性度量的参数
        // param.gamma = 3;                                      // 先验概率常数
        // param.sigma = 1;                                         // 先验概率的标准差
        // param.sradius = 3;                                       // 标准差半径
        // param.match_texture = 1;                         // 最低纹理
        // param.lr_threshold = 1;                             // 左右一致性检验阈值
        // param.speckle_sim_threshold = 1;          // 连通域判断阈值
        // param.speckle_size = 200;                        // 连通域噪声尺寸判断阈值
        // param.ipol_gap_width = 3;                       // 空洞宽
        // param.filter_median = false;                     // 是否中值滤波
        // param.filter_adaptive_mean = true;        // 是否自适应中值滤波
        // param.subsampling = false;                     // 每个两个像素进行视差计算，设置为True可以节省时间，但是传入的D1和D2的分辨率必须为(w/2) x (h/2)

        // Elas elas(param);
        // elas.process(left.data, right.data,disp_left.ptr<float>(0), disp_right.ptr<float>(0), dim);

		//! openCV(kitti 貌似效果好些)
		cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);// 神奇的参数
    	cv::Mat disparity_sgbm,disparity,disparity_show;//差距
    	sgbm->compute(left,right,disparity_sgbm);
    	disparity_sgbm.convertTo(disparity,CV_32F,1.0 / 16.0f);
    	// disparity_sgbm.convertTo(disparity_show,CV_32F,1.0 / 16.0f / 96.0f);//todo use to show

    //  vector<Eigen::Matrix<float, 6, 1>> Points;

	    // cv::imshow("disp_left", disp_left);
        // cv::imshow("disp_right", disp_right);

		// cv::imshow("disparity", disparity);
        // cv::imshow("disparity_sgbm", disparity_sgbm);
		// cv::waitKey(10);

    for (size_t v = 4; v < left.rows - 4; v += 4)
    {
        for (size_t u = 4; u < left.cols - 4; u += 4)
        {
            // if (disp_left.at<float>(v,u) <= 0 || disp_left.at<float>(v,u) >= 96.0)   continue;

            if (disparity.at<float>(v,u) <= 0.0 || disparity.at<float>(v,u) >= 96.0)   continue;

            Eigen::Vector2d a(u, v);
            Eigen::Vector3d a_3d;
            estimator.featureTracker.m_camera[0]->liftProjective(a, a_3d);
			//! elas
            // double depth = 718.856 * 0.573 / disp_left.at<float>(v,u);

			//! OpenCV
			double depth = 718.856 * 0.573 / disparity.at<float>(v,u);


            Eigen::Matrix<double, 6, 1> point;
            point << a_3d.x() * depth, a_3d.y() * depth, depth, left.at<uchar>(v,u) / 255.0, left.at<uchar>(v,u) / 255.0, left.at<uchar>(v,u) / 255.0;
			
			Eigen::Vector3d points(point[0], point[1], point[2]);
			//  if(points[2] <= 0 || points[2] >= 15|| points[0] <= -20 || points[0] >= 20 || points[1] <= -20 || points[1] >= 0) continue;
            
            Eigen::Vector3d pointOdom = estimator.ric[0] * points + estimator.tic[0];
			//  if(pointOdom[2] <= 0 || pointOdom[2] >= 30|| pointOdom[0] <= 0 || pointOdom[0] >= 20 || pointOdom[1] <= -30 || pointOdom[1] >= 30) continue;
			// if(pointOdom[2] >= 10|| pointOdom[0] <= -20 || pointOdom[0] >= 20 || pointOdom[1] <= -20 || pointOdom[1] >= 20) continue;
			if(pointOdom[2] >= 20|| pointOdom[0] <= -20 || pointOdom[0] >= 20 || pointOdom[1] <= -20|| pointOdom[1] >= 20) continue;
			
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
				tmp->points.push_back(searchPoint);//滤波用
			}


            // Points.push_back(point);

        }
    }

	             //!统计滤波器
            statistical_filter.setMeanK(4);//50
            statistical_filter.setStddevMulThresh(0.8);//1
            statistical_filter.setInputCloud(tmp);
            statistical_filter.filter(*cloud);

			//             //!体素滤波
            // voxel_filter.setLeafSize(0.03,0.03,0.03);
            // voxel_filter.setInputCloud(tmp);
            // voxel_filter.filter(*cloud);

//             //!半径滤波(现在能用)
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

    // double t = header.stamp.toSec();
    // point_rgbd.insert(pair<double,vector<Eigen::Matrix<float, 6, 1>>>(t,Points));
}
//todo


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage",1000);
	ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage",1000);

	pub_octree = n.advertise<sensor_msgs::PointCloud2>("octree", 1000);//todo

	if(argc != 3)
	{
		printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_odom_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
			   "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2];
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";
//todo
	// readParameters(config_file);
	// estimator.setParameter();
	// registerPub(n);

	//设置点云对象
	octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>( 0.01 );
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	tmp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	octree->setInputCloud(cloud);
	octree->addPointsFromInputCloud();
	octree->defineBoundingBox(-1000000, -1000000, -1000000, 1000000, 1000000, 1000000);
//todo
	// load image list
	FILE* file;
	file = std::fopen((dataPath + "times.txt").c_str() , "r");
	if(file == NULL){
	    printf("cannot find file: %stimes.txt\n", dataPath.c_str());
	    ROS_BREAK();
	    return 0;          
	}
	double imageTime;
	vector<double> imageTimeList;
	while ( fscanf(file, "%lf", &imageTime) != EOF)
	{
	    imageTimeList.push_back(imageTime);
	}
	std::fclose(file);

//todo   load odom
// load odom

	//ros::Subscriber sub_wheel_odom = n.subscribe(ODOM_TOPIC,2000,wheel_odom_callback);

	//ros::Publisher pulish_odom = n.advertise<nav_msgs::Odometry>(ODOM_TOPIC,1000);

	ifstream fin(odom_file);

	if(!fin)
	{
	    cout << "cannot find file " << odom_file << endl;
	    ROS_BREAK();
	    return 0;          
	}
	cout << "load odom" << endl;
	int num = 0;
	while (!fin.eof())
	{
	    Eigen::Matrix3d R;
		Eigen::Vector3d T;
		fin >> R(0,0) >> R(0,1) >> R(0,2) >> T(0) >> R(1,0) >> R(1,1) >> R(1,2) >> T(1) >> R(2,0) >> R(2,1) >> R(2,2) >> T(2);
		Eigen::Quaterniond q(R);
		q.normalize();//todo
		//cout << "load odom 1" << endl;
		nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
		odom->header.stamp = ros::Time(imageTimeList[num]);
		odom->header.frame_id = "odom";
		odom->child_frame_id = "base_footprint";

		odom->pose.pose.position.x = T(0);
		odom->pose.pose.position.y = T(1);
		odom->pose.pose.position.z = T(2);

		odom->pose.pose.orientation.x = q.x();
		odom->pose.pose.orientation.y = q.y();
		odom->pose.pose.orientation.z = q.z();
		odom->pose.pose.orientation.w = q.w();
		//pulish_odom.publish(odom);
		//nav_msgs::Odometry::ConstPtr odom_ptr(odom);
		m_wheel_odom.lock();
		wheel_odom_buf.push(odom);
		m_wheel_odom.unlock();
		num++;
		if(num == 1101) break;//!  00=4541 01=1101 02=4661 04=271  05=2761  06=1101
		//cout << num << endl;
		//cout << imageTimeList[num] << endl;
		//cout << "x: " << T(0) << "y: " << T(1) << endl; 
	}
		cout << "load odom finsh" << endl;
//todo

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);


	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	FILE* outFile;
	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	if(outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
	
	FILE* time_spend;
	time_spend = fopen((OUTPUT_FOLDER + "/time_spend.txt").c_str(),"w");
	TicToc t_s;
	// estimator.ceres_ci = fopen((OUTPUT_FOLDER + "/ci.txt").c_str(),"w");//todo 12.22
	// estimator.ceres_cf = fopen((OUTPUT_FOLDER + "/cf.txt").c_str(),"w");//todo 12.22
	// estimator.ceres_times = fopen((OUTPUT_FOLDER + "/times.txt").c_str(),"w");//todo 12.22
	// estimator.ceres_nums = fopen((OUTPUT_FOLDER + "/nums.txt").c_str(),"w");//todo 12.22
	// estimator.ceres_a_times = fopen((OUTPUT_FOLDER + "/a_times.txt").c_str(),"w");//todo 12.22
	// estimator.ceres_a_nums = fopen((OUTPUT_FOLDER + "/a_nums.txt").c_str(),"w");//todo 12.22


	// estimator.select_times = fopen((OUTPUT_FOLDER + "/select_times.txt").c_str(),"w");//todo 12.22
	// estimator.ceres_delta_ci = fopen((OUTPUT_FOLDER + "/ceres_delta_ci.txt").c_str(),"w");//todo 12.22
	// estimator.feature_delta_nums = fopen((OUTPUT_FOLDER + "/feature_delta_nums.txt").c_str(),"w");//todo 12.22
	// estimator.select_delta_nums = fopen((OUTPUT_FOLDER + "/select_delta_nums.txt").c_str(),"w");//todo 12.22
	// estimator.c_f = fopen((OUTPUT_FOLDER + "/c_f.txt").c_str(),"w");//todo 12.22
	// estimator.c_s = fopen((OUTPUT_FOLDER + "/c_s.txt").c_str(),"w");//todo 12.22

	// estimator.featureTracker.last_score = fopen((OUTPUT_FOLDER + "/last_score.txt").c_str(),"w");//todo 3.22
	// estimator.featureTracker.new_last_score = fopen((OUTPUT_FOLDER + "/new_last_score.txt").c_str(),"w");//todo 3.22
	// estimator.featureTracker.new_point_num = fopen((OUTPUT_FOLDER + "/new_point_num.txt").c_str(),"w");//todo 3.22
	// estimator.featureTracker.min_dis = fopen((OUTPUT_FOLDER + "/min_dis.txt").c_str(),"w");//todo 3.22
	// estimator.featureTracker.true_match_pro = fopen((OUTPUT_FOLDER + "/true_match_pro.txt").c_str(),"w");//todo 3.22
	// cout << "fopen" << endl;

	for (size_t i = 0; i < imageTimeList.size(); i++)
	{	
		if(ros::ok())
		{
			t_s.tic();//todo 3.23

			printf("\nprocess image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(6) << i;
			leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
			rightImagePath = dataPath + "image_1/" + ss.str() + ".png";
			//printf("%lu  %f \n", i, imageTimeList[i]);
			//printf("%s\n", leftImagePath.c_str() );
			//printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
			imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubLeftImage.publish(imLeftMsg);

			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
			imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubRightImage.publish(imRightMsg);


			estimator.inputImage(imageTimeList[i], imLeft, imRight);
			//todo
			//estimator.inputImage(imageTimeList[i], imLeft);
			//todo

			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			if(outFile != NULL)
				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f\n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			

			//todo
			//  if(i % 2 == 0)
				// getPointCloudfromStereo(imLeft, imRight, imLeftMsg->header);//! todo
			//estimator.inputImage(imageTimeList[i], imLeft);
			//todo
			
			//cv::imshow("leftImage", imLeft);
			//cv::imshow("rightImage", imRight);


			// cv::waitKey(1000);//todo
			// cout << "imageTimeList.size()  : "   <<   i << endl;


			//todo 3.23
			if(time_spend != NULL)
				fprintf(time_spend, "%f\n", t_s.toc());
		}
		else
		{
			// cout << "*******end  1  **********" << endl;
			break;
		}
			
	}

	if(outFile != NULL)
	{
		cout << "*********	end  1  **********" << endl;
		fclose (outFile);
	}
	//todo 12.22

	if(time_spend != NULL)
	{
		cout << "*********	end  1  **********" << endl;
		fclose (time_spend);
	}


	// if(estimator.ceres_ci != NULL && estimator.ceres_cf != NULL && estimator.ceres_times != NULL && estimator.ceres_nums != NULL && estimator.ceres_a_times != NULL && estimator.ceres_a_nums != NULL)
	// {
	// 	cout << "*********	end  2  **********" << endl;
	// 	fclose (estimator.ceres_ci);
	// 	fclose (estimator.ceres_cf);
	// 	fclose (estimator.ceres_times);
	// 	fclose (estimator.ceres_nums);
	// 	fclose (estimator.ceres_a_times);
	// 	fclose (estimator.ceres_a_nums);
	// }

	// if(estimator.select_times != NULL) fclose(estimator.select_times);//todo 12.23
	// if(estimator.feature_delta_nums != NULL) fclose(estimator.feature_delta_nums);//todo 12.23
	// if(estimator.select_delta_nums != NULL) fclose(estimator.feature_delta_nums);//todo 12.23
	// if(estimator.ceres_delta_ci != NULL) fclose(estimator.ceres_delta_ci);//todo 12.25
	// if(estimator.c_f != NULL) fclose(estimator.c_f);//todo 12.25
	// if(estimator.c_s != NULL) fclose(estimator.c_s);//todo 12.25
	
	// if(estimator.featureTracker.last_score != NULL) fclose(estimator.featureTracker.last_score);//todo 3.22
	// if(estimator.featureTracker.new_last_score != NULL) fclose(estimator.featureTracker.new_last_score);//todo 3.22
	// if(estimator.featureTracker.new_point_num != NULL) fclose(estimator.featureTracker.new_point_num);//todo 3.22
	// if(estimator.featureTracker.min_dis != NULL) fclose(estimator.featureTracker.min_dis);//todo 3.22
	// if(estimator.featureTracker.true_match_pro != NULL) fclose(estimator.featureTracker.true_match_pro);//todo 3.22
	// cout << "fclose" << endl;
	return 0;
}
