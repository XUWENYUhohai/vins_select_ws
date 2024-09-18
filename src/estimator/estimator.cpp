/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"

Estimator::Estimator(): f_manager{Rs}//todo 应该是用{}替代（）初始化
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();// 等待processThread线程释放
        printf("join thread \n");//等待该线程结束,但该线程的写法有问题，processMeasurements为死循环，不会自己结束，此处可完善修改
    }
}



/**
 * 清理状态，缓存数据、变量、滑动窗口数据、位姿等
 * 系统重启或者滑窗优化失败都会调用
*/
void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    while(!wheel_odom_buf.empty())//todo
        wheel_odom_buf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

// 设置参数，并开启processMeasurements线程
void Estimator::setParameter()
{
    mProcess.lock(); // 锁mProcess主要用于processThread线程互斥安全
    for (int i = 0; i < NUM_OF_CAM; i++)
    { // 外参，body_T_cam
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    //以下设置图像因子需使用的sqrt_info（即信息矩阵的根号值），该设置假定图像特征点提取存在1.5个像素的误差，
    // 存在FOCAL_LENGTH,是因为图像因子残差评估在归一化相机坐标系下进行
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // 图像的时间戳晚于实际采样时候的时间，硬件传输等因素
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    //todo
    featureTracker.readIntrinsicParameter(CAM_NAMES,DEPTH); // 设置相机内参，该参数主要用于特征点跟踪过程，CAM_NAMES是相机参数的路径
    printf("readIntrinsicParameter \n");

    feature_selector = std::make_shared<FeatureSelector>(&f_manager, featureTracker.m_camera[0], Rs, Ps);
    feature_selector->setPara(0.01, 0.001, 30, 40);//! 11.30  max的数量有待测试
    //todo

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)// 如果是单线程，且线程没有chuli则开启开启了一个Estimator类内的新线程：processMeasurements();
    {//申明并定义一个 处理 的线程 
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}


/**
 * 双目，双目+IMU，单目+IMU，如果重新使用IMU，需要重启节点
*/
void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)//更改传感器配置后，vio系统重启，此处重启与检测到failure后的重启一样，主要包括清除状态、设置参数两步
    {
        clearState();
        setParameter();
    }
}


/**
 * 输入一帧图像
 * 1、featureTracker，提取当前帧特征点
 * 2、添加一帧特征点，processMeasurements处理
*/
// 给Estimator输入图像
// 其实是给featureTracker.trackImage输入图像，之后返回图像特征featureFrame。填充featureBuf
// 之后执行processMeasurements
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{   
    printf("inputimage 1\n");

    inputImageCnt++;
    //todo 7
    map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> featureFrame;// 数据格式为feature_id camera_id（0或1） xyz_uv_velocity（空间坐标，像素坐标和像素速度）
    TicToc featureTrackerTime;

   /**
     * 跟踪一帧图像，提取当前帧特征点
     * 1、用前一帧运动估计特征点在当前帧中的位置，如果特征点没有速度，就直接用前一帧该点位置
     * 2、LK光流跟踪前一帧的特征点，正反向，删除跟丢的点；如果是双目，进行左右匹配，只删右目跟丢的特征点
     * 3、对于前后帧用LK光流跟踪到的匹配特征点，计算基础矩阵，用极线约束进一步剔除outlier点（代码注释掉了）
     * 4、如果特征点不够，剩余的用角点来凑；更新特征点跟踪次数
     * 5、计算特征点归一化相机平面坐标，并计算相对与前一帧移动速度
     * 6、保存当前帧特征点数据（归一化相机平面坐标，像素坐标，归一化相机平面移动速度）
     * 7、展示，左图特征点用颜色区分跟踪次数（红色少，蓝色多），画个箭头指向前一帧特征点位置，如果是双目，右图画个绿色点
    */

//    printf("inputimage 2\n");
    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());
    // printf("inputimage 3\n");
    if (SHOW_TRACK)// 发布跟踪图像
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }
    
    if(MULTIPLE_THREAD)  
    {      //多线程时，在函数setParameter中，processMeasurements已经作为回调函数、创建了新的线程processThread
        // if(inputImageCnt % 2 == 0)  //! todo
        {//多线程的时候，仅使用偶数计数的图像特征？如果计算量足够或者图像帧率较低，可去除此处限制
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements(); //执行了processMeasurements这个线程
        printf("process time: %f\n", processTime.toc());
    }
    
}



/**
 *  对imu数据进行处理，包括更新预积分量，和提供优化状态量的初始值
 * 
 * 输入一帧IMU
 * 1、积分预测当前帧状态，latest_time，latest_Q，latest_P，latest_V，latest_acc_0，latest_gyr_0
 * 2、发布里程计
 * 
 * 
// 填充了accBuf和gyrBuf
*/
void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR)// 如果已经初始化了
    {
        mPropagate.lock();
        // IMU预测状态，更新QPV
        // latest_time，latest_Q，latest_P，latest_V，latest_acc_0，latest_gyr_0
        fastPredictIMU(t, linearAcceleration, angularVelocity);//IMU 中值积分预测
        pubLatestOdometry(latest_P, latest_Q, latest_V, t); // 发布里程计
        mPropagate.unlock();
    }
}



// 在估计器中输入点云数据，并填充featureBuf
// 输入一帧特征点，processMeasurements处理
void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> &featureFrame) //该函数并未使用  //todo 7
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}


// 对imu的时间进行判断，将队列里的imu数据放入到accVector和gyrVector中，完成之后返回true
//从IMU数据队列中，提取(t0, t1)时间段的数据,t0和t1一般为相邻图像帧时间戳
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}


// 判断输入的时间t时候的imu是否可用
bool Estimator::IMUAvailable(double t)
{// 加速度vector不为空，并且图像帧时间戳不大于加速度时间戳，则认为IMU数据可用
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}




//todo
bool Estimator::wheelOdomAvailable(double t) {
    if (!wheel_odom_buf.empty() && t <= wheel_odom_buf.back()->header.stamp.toSec())
        return true;
    else
    {
        return false;
    }

}
//todo




/**
 * 处理一帧特征点
*/
void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>>> feature;//时间戳（double）、路标点编号（int）、相机编号（int）
                                        //特征点信息（Matrix<double, 7, 1>，归一化相机坐标系坐标（3维）、去畸变图像坐标系坐标（2维）、特征点速度（2维））//todo score(double) lifetime(int)
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty())
        {  // 当前图像帧特征
            feature = featureBuf.front(); //拿到当前image时刻feature
            curTime = feature.first + td;//td的使用是在图像的时间上加上这个值//feature的时间实际就是 current image frame时间//时间偏差补偿后的图像帧时间戳
            while(1)
            {
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))//如果不用imu或者imu数据可得到
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);//定义5ms的延迟
                    std::this_thread::sleep_for(dura);
                }
            }

            //todo
            while (1) {
                if (!USE_ODOM || wheelOdomAvailable(feature.first))
                //if (wheelOdomAvailable(feature.first + td))
                    break;
                else {
                    printf("wait for wheel odom data ... \n");
                    std::chrono::milliseconds dura(10);
                    std::this_thread::sleep_for(dura);
                }
            }
            //todo



            mBuf.lock();
            if(USE_IMU)// 前一帧与当前帧图像之间的IMU数据
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            featureBuf.pop();//每次运行完之后都删除featureBuf中的元素，直到为空，已经把要删除的这个值给了feature
            mBuf.unlock();
//对于两帧之间的所有IMU 处理    
            if(USE_IMU)
            { // 第一帧IMU姿态初始化  //位姿未初始化，则利用加速度初始化Rs[0]//基于重力，对准第一帧，即将初始姿态对准到重力加速度方向
                // 用初始时刻加速度方向对齐重力加速度方向，得到一个旋转，使得初始IMU的z轴指向重力加速度方向
                if(!initFirstPoseFlag)  //false 第一帧； true是第一帧
                    initFirstIMUPose(accVector);
                for(size_t i = 0; i < accVector.size(); i++)
                {// IMU积分
                // 用前一图像帧位姿，前一图像帧与当前图像帧之间的IMU数据，积分计算得到当前图像帧位姿
                // Rs，Ps，Vs
                    double dt;
                    if(i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;//倒数第二个IMU数据
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                        //IMU数据处理 - 计算预积分增量，更新雅克比、协方差 
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);//IMU数据处理，主要创建预积分因子、中值积分预测状态
                }
            }
            mProcess.lock();
            
            //todo
            Eigen::Vector3d predict_odom_p;
            Eigen::Quaterniond predict_odom_q;
            int num_odom = 3;//! 11.30 数量有待测试   3
            image_t image;
            std::pair<std::vector<int>, std::vector<int>> select_info;
            std::map<int, double, std::greater<double>> weight;

            static TicToc t_c;
            t_c.tic();
            
            //todo 12.25
            // // static int last_feature_num = 0; 
            // int delta_feature_num = feature.second.size() - last_feature_num;
            // last_feature_num = feature.second.size();
            // if(feature_delta_nums != NULL) fprintf(feature_delta_nums, "%d\n", delta_feature_num);

            // if(delta_feature_num <= 0 && delta_ci >= 0)
            // {
                
            //     if(c_f != NULL) fprintf(c_f, "%f\n", std::abs(delta_ci / delta_feature_num) );
            // }
            // else
            // {
            //     if(c_f != NULL) fprintf(c_f, "%f\n", 0);
            // }

            //todo 12.25
            //!
            // if (solver_flag == NON_LINEAR)//todo 
            // {
            //     for (auto & i : feature.second)
            //     {   //时间戳（double）
            //         //路标点编号（int）、相机编号（int）//特征点信息（Matrix<double, 7, 1>，归一化相机坐标系坐标（3维）、去畸变图像坐标系坐标（2维）、特征点速度（2维））//todo score(double) lifetime(int)
            //         int f_id = i.first;
            //         double x = i.second[0].second.coeff(0);
            //         double y = i.second[0].second.coeff(1);
            //         double z = i.second[0].second.coeff(2);
            //         double u = i.second[0].second.coeff(3);
            //         double v = i.second[0].second.coeff(4);
            //         double u_vel = i.second[0].second.coeff(5);
            //         double v_vel = i.second[0].second.coeff(6);
            //         double prob = i.second[0].second.coeff(7);
            //         double lifetime = i.second[0].second.coeff(8) / 10;
            //         ROS_ASSERT(z == 1);

            //         // weight[f_id] = prob * lifetime;//todo 12.13

            //         Eigen::Matrix<double, 8, 1> xyz_uv_vel_prob;
            //         xyz_uv_vel_prob << x, y, z, u, v, lifetime, lifetime, prob;// ! l 12.4
            //         // xyz_uv_vel_prob << x, y, z, u, v, u_vel, v_vel, prob;// ! 
            //         image[f_id].emplace_back(0, xyz_uv_vel_prob);



            //         //todo 12.20
            //         if(STEREO)
            //         {
            //             x = i.second[1].second.coeff(0);
            //             y = i.second[1].second.coeff(1);
            //             z = i.second[1].second.coeff(2);
            //             u = i.second[1].second.coeff(3);
            //             v = i.second[1].second.coeff(4);
            //             u_vel = i.second[1].second.coeff(5);
            //             v_vel = i.second[1].second.coeff(6);
            //             prob = i.second[1].second.coeff(7);
            //             lifetime = i.second[1].second.coeff(8) / 10;
            //             ROS_ASSERT(z == 1);
            //             xyz_uv_vel_prob << x, y, z, u, v, lifetime, lifetime, prob;
            //             image[f_id].emplace_back(1, xyz_uv_vel_prob);
            //         }
            //         //todo 12.20
            //     }

            //     // select_info = feature_selector->allIn(weight);//todo 12.13

            //     processOdom(predict_odom_p, predict_odom_q, prevTime, curTime);//todo 2.23  与odom optimization 冲突需添加新消息
            //     Eigen::Quaterniond Qs;
            //     Qs = Rs[frame_count];
            //     feature_selector->setNextStateFromOdomPropagation(curTime, Ps[frame_count], Qs, predict_odom_p, predict_odom_q);
                
            //     select_info = feature_selector->select(image, curTime, num_odom, STEREO, featureTracker.cur_img);//todo
            //     // select_info = feature_selector->allIn(image, curTime, num_odom, STEREO);//todo
            //     // select_info = feature_selector->roulette(image, curTime, num_odom, STEREO);
            // }


            // map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> select;

            // for (int i = 0; i < select_info.first.size(); i++)//todo
            // {
            //     int id = select_info.first[i];//todo
            //     // int id = select_info.second[i];
            //     if (feature.second.find(id) != feature.second.end())
            //     {
            //         select[id] = feature.second[id];
            //         // select[id] = feature.second.at(id);
                    
            //     }
            // }
            //!

            //todo 12.26  //jia xian chen suo
            // static int last_select_num = 0; 
            // int delta_select_num = select.size() - last_select_num;
            // last_select_num = select.size();
            // if(select_delta_nums != NULL) fprintf(select_delta_nums, "%d\n", delta_select_num);

            // if(delta_select_num <= 0 && delta_ci >= 0)
            // {
                
            //     if(c_s != NULL) fprintf(c_s, "%f\n", std::abs(delta_ci / delta_select_num) );
            // }
            // else
            // {
            //     if(c_s != NULL) fprintf(c_s, "%f\n", 0);
            // }

            //todo 12.26
           
            //todo 12.13 use for all in weight
            // int weight_max = 1;
            // for(auto i : weight)
            // {
            //     if(weight_max == 130) break;
            //     int fid = i.first;
            //     select[fid] = feature.second[fid];
            //     weight_max++;
            // }
            //todo

            // cout << "feature select took: " << t_c.toc() << "ms" << endl;

            // if(select_times != NULL) fprintf(select_times, "%f\n", t_c.toc());//todo 12.25
            
            // cout << "feature  num: " << feature.second.size() << "个" << endl;
            // cout << "select_info num: " << select_info.first.size() << "个" << endl;
            // cout << "feature select num: " << select.size() << "个" << endl;
            
            //! NOTE:开启后会影响结果
            // mImage.lock();

            // cv::Mat select_image = featureTracker.getSelectImage();
            // cv::Mat select_img = feature_selector->getSelectImg();
            // cv::Mat vins_image = featureTracker.getTrackImage();

            // cv::vconcat(select_img, select_image, select_img);

            // //todo 红色tracked，蓝色select, 绿色vins
            // for (auto & i : feature.second)
            // {
            //     cv::Point2f point(i.second[0].second.coeff(3), i.second[0].second.coeff(4) /* + select_image.rows / 2 */);
            //     cv::circle(select_img, point, 2, cv::Scalar(0, 255, 0), 2);
            // }


            // //todo 12.13 use for all in weight
            // // for (auto & k : select)
            // // {
            // //     cv::Point2f point(k.second[0].second.coeff(3), k.second[0].second.coeff(4) /* + select_image.rows / 2 */);
            // //     cv::circle(select_image, point, 3, cv::Scalar(0, 0, 255), 2);
            // // }
            // //todo


            // // for (int j = 0; j < select_info.first.size(); j++)
            // // {
            // //     int id = select_info.first[j];
            // //     if (feature.second.find(id) != feature.second.end())
            // //     {
            // //         cv::Point2f points(feature.second[id][0].second.coeff(3), feature.second[id][0].second.coeff(4) + select_image.rows / 2);
            // //         cv::circle(select_image, points, 2, cv::Scalar(255, 0, 0), 2);
            // //     }

            // // }
            
            // for (int i = 0; i < select_info.second.size(); i++)
            // {
            //     int id = select_info.second[i];
            //     if(feature.second.find(id) != feature.second.end())
            //     {
            //         cv::Point2f point(feature.second[id][0].second.coeff(3), feature.second[id][0].second.coeff(4) /* + select_image.rows / 2 */);
            //         cv::circle(select_img, point, 2, cv::Scalar(255, 0, 0), 2);
            //     }
            // }

            // cv::imshow("select_image", select_image);
            // cv::imshow("vins_image", select_img);
            // cv::waitKey(1);
            // cv::imwrite("/home/xuwenyu/00/" + to_string(curTime) + "_track.png", select_image);
            // cv::imwrite("/home/xuwenyu/00/" + to_string(curTime) + "_select.png", select_img);
            // mImage.unlock();


             //图像特征点数据处理
            // if(select.size() >= 50)//! 11.30 数量有待测试  select best(max150, >=130, horizon=3)  roulette(max 130 >=100 <=30)
            // {
            //     processImage(select, feature.first);//todo
            //     printf("processImage via select\n");
            // }
            // else
            {
                processImage(feature.second, feature.first);//函数名字不太妥当，后续优化等过程全在该函数实现;优化等过程的入口
                printf("processImage via origin\n");

            }
            
            //todo

            
            prevTime = curTime;

            printStatistics(*this, 0);  //打印调试信息; 优化后的外参输出也在该函数实现
            
            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);



            pubOdometry(*this, header);//发布优化后的信息//滑动窗口最新的优化帧的位置和速度
            
            pubKeyPoses(*this, header);//把滑动窗口内所有位姿发布出来
            pubCameraPose(*this, header);//相机在world下的odometry消息，没有速度
            pubPointCloud(*this, header);//即发布约束点云，也发布滑动窗口起始位置的所有点云
             //该函数即发布关键帧位置，也发布关键帧所有点云
            pubKeyframe(*this);//当MARGIN_OLD时，也即次次新帧为关键帧; 将次次新帧发布出去，但是前面关键帧判断条件较为宽松;可将关键帧选取更严格
            pubTF(*this, header); 

            //todo  30
            //  if(inputImageCnt % 30 == 0)
            // {
                // cout << "pubOctree" << endl;
                // pubOctreefromRGBD(*this, header);
                // pubOctreefromStereo(*this, header);
            // }
                //printf("pubOctree\n");
            //todo

            mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        
        //printf("processMeasurements\n");
        std::chrono::milliseconds dura(2);//2
        std::this_thread::sleep_for(dura);

        
    }
}


//初始第一个imu位姿//根据初始的IMU acc 算出初始旋转 R0
//用初始时刻加速度方向对齐重力加速度方向，得到一个旋转，使得初始IMU的z轴指向重力加速度方向
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;  //计算加速度的均值
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
     // 计算初始IMU的z轴对齐到重力加速度方向所需的旋转，后面每时刻的位姿都是在当前初始IMU坐标系下的，乘上R0就是世界系了
    // 如果初始时刻IMU是绝对水平放置，那么z轴是对应重力加速度方向的，但如果倾斜了，那么就是需要这个旋转让它水平

     // 主要利用基于重力方向，得到的roll pitch信息，由于yaw信息通过重力方向，并不能恢复出来，因此减去yaw
    Matrix3d R0 = Utility::g2R(averAcc);
    //多余？》
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;//另初始的航向为0
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}



/**
 * 处理一帧IMU，积分
 * 用前一图像帧位姿，前一图像帧与当前图像帧之间的IMU数据，算出当前帧的位置、速度、旋转Ps Vs Rs
 * @param t                     当前时刻
 * @param dt                    与前一帧时间间隔
 * @param linear_acceleration   当前时刻加速度
 * @param angular_velocity      当前时刻角速度
*/
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)// 第一个imu处理
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    // 滑窗中保留11帧，frame_count表示现在处理第几帧，一般处理到第11帧时就保持不变了//frame_count 初始化之后一直是10，
    // 由于预积分是帧间约束，因此第1个预积分量实际上是用不到的

// 如果是新的一帧,则新建一个预积分项目
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0) //不是滑窗中的第一帧
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);// push_back进行了重载，的时候就已经进行了预积分
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);// 这个量用来做初始化用的//这个是输入到图像中的预积分值
// 保存传感器数据 //将图像帧之间的imu数据存储至 元素为vector的数组 
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
   // 又是一个中值积分，更新滑窗中状态量，本质是给非线性优化提供可信的初始值，此处与预积分中的中值积分基本相似；
   //但此处的中值积分是以世界坐标系为基准，即更新的Rs、Ps、Vs在世界坐标系下表达 
     // 当前时刻 PVQ的中值积分离散形式，
        // 来了一帧IMU 中值积分一次， 又可以作为下次的初值

         // 对位移速度等进行累加
        // Rs Ps Vs是frame_count这一个图像帧开始的预积分值,是在绝对坐标系下的.      （下面的R,P,V是对processImage初始化最后那里（674行左右）进行的更新）
        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;// 前一时刻加速度 
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];// 中值积分，用前一时刻与当前时刻角速度平均值，对时间积分
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();//等式右侧对应dq，利用0.5*theta得到；左侧Rs[j]乘以dq更新旋转// 当前时刻姿态Q
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g; // 当前时刻加速度 
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);  // 中值积分，用前一时刻与当前时刻加速度平均值，对时间积分
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc; // 当前时刻位置P
        Vs[j] += dt * un_acc; // 当前时刻速度V
    }
    //此处的acc_0和gyr_0定义在estimator.h中,注意与integration_base.h中的区别；两处的作用都是为了存储之前的加速度和角速度，用于中值积分
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

//todo
void Estimator::processOdom(Eigen::Vector3d& predict_odom_p, Eigen::Quaterniond& predict_odom_q, const double pre_time, const double cur_time)
{
    Eigen::Vector3d rel_p;
    Eigen::Matrix3d rel_r;

    // f_manager.getRelativePoseByWheel(rel_r, rel_p, pre_time, cur_time);

    if(f_manager.getRelativePoseByWheel(rel_r, rel_p, pre_time, cur_time))
    {
        predict_odom_p = Ps[frame_count] + Rs[frame_count] * rel_p;
        // predict_odom_q = Eigen::Quaterniond(Rs[frame_count]);
        predict_odom_q = Rs[frame_count] * rel_r;//todo 重载了=

    }
    else
    {
        rel_p = Rs[frame_count].transpose() * (Ps[frame_count] - Ps[frame_count - 1]);
        rel_r = Rs[frame_count - 1].transpose() * Rs[frame_count];

        predict_odom_p = Ps[frame_count] + Rs[frame_count] * rel_p;
        //    predict_odom_q = Eigen::Quaterniond(Rs[frame_count] * rel_q);
        predict_odom_q = Rs[frame_count] * rel_r;//todo 重载了=
    }
}
//todo


/**
 * 处理一帧图像特征
 * 1、提取前一帧与当前帧的匹配点
 * 2、在线标定外参旋转
 *     利用两帧之间的Camera旋转和IMU积分旋转，构建最小二乘问题，SVD求解外参旋转
 *     1) Camera系，两帧匹配点计算本质矩阵E，分解得到四个解，根据三角化成功点比例确定最终正确解R、t，得到两帧之间的旋转R
 *     2) IMU系，积分计算两帧之间的旋转
 *     3) 根据旋转构建最小二乘问题，SVD求解外参旋转
 * 3、系统初始化
 * 4、3d-2d Pnp求解当前帧位姿
 * 5、三角化当前帧特征点
 * 6、滑窗执行Ceres优化，边缘化，更新滑窗内图像帧的状态（位姿、速度、偏置、外参、逆深度、相机与IMU时差）
 * 7、剔除outlier点
 * 8、用当前帧与前一帧位姿变换，估计下一帧位姿，初始化下一帧特征点的位置
 * 9、移动滑窗，更新特征点的观测帧集合、观测帧索引（在滑窗中的位置）、首帧观测帧和深度值，删除没有观测帧的特征点
 * 10、删除优化后深度值为负的特征点
 * @param image  图像帧特征
 * @param header 时间戳
*/
/* *********************************************************************************
* visual processing front end
* input: 一帧图像中对应的多个特征点
* 计算当前帧与窗口的平均视差
* 如果需要校准IMU camera的相对外参，校准一下
* 如果没有初始化，就进行初始化
* 如果初始化后，就进行非线性优化
********************************************************************************* */ 
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> &image, const double header)//todo
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))//todo 7
    {
        marginalization_flag = MARGIN_OLD;
        printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);//todo 7
    // 当前帧预积分（前一帧与当前帧之间的IMU预积分）
    imageframe.pre_integration = tmp_pre_integration;//当前图像帧对应的预积分

      // all_image_frame用来做初始化相关操作，他保留滑窗起始到当前的所有帧
    // 有一些帧会因为不是KF，被MARGIN_SECOND_NEW，但是及时较新的帧被margin，他也会保留在这个容器中，因为初始化要求使用所有的帧，而非只要KF
    all_image_frame.insert(make_pair(header, imageframe));
    // 这里就是简单的把图像和预积分绑定在一起，这里预积分就是两帧之间的，滑窗中实际上是两个KF之间的
    // 实际上是准备用来初始化的相关数据
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; // 重置预积分器

    // 没有外参初值
    // Step 2： 外参初始化
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {//获取frame_count - 1 与 frame_count两个图像帧匹配的特征点，特征点在归一化相机坐标系
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1; // 标志位设置成可信的外参初值
            }
        }
    }


    if (solver_flag == INITIAL)
    {


        //TODO 12.25 未测试 ESTIMATE_EXTRINSIC == 3
        if(ESTIMATE_EXTRINSIC == 3 && USE_ODOM)//todo 2.23
        {
            Eigen::Matrix3d R_odom;
            Eigen::Vector3d T_odom;
            ROS_INFO("calibrating extrinsic param, rotation movement is needed");
            if (frame_count != 0 && f_manager.getPoseByWheelOdom(T_odom, R_odom, header))
            {//获取frame_count - 1 与 frame_count两个图像帧匹配的特征点，特征点在归一化相机坐标系
                vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
                Matrix3d calib_ric;
                if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
                {
                    ROS_WARN("initial extrinsic rotation calib success");
                    ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                    ric[0] = calib_ric;
                    RIC[0] = calib_ric;
                    ESTIMATE_EXTRINSIC = 1; // 标志位设置成可信的外参初值
                    ROS_WARN_STREAM("initial extrinsic");
                }
            }
        }



        // monocular + IMU initilization// 单目初始化
        if (!(STEREO || DEPTH) && USE_IMU)//todo 单目初始化+imu
        {
            if (frame_count == WINDOW_SIZE)  // 要求滑窗满
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)//有外参且当前帧时间戳大于初始化时间戳0.1秒，就进行初始化操作
                {//initial_timestamp设为了0

                  /** //对应VINS_mono论文 V章节
                     * 系统初始化
                     * 1、计算滑窗内IMU加速度的标准差，用于判断移动快慢
                     * 2、在滑窗中找到与当前帧具有足够大的视差，同时匹配较为准确的一帧，计算相对位姿变换
                     *   1) 提取滑窗中每帧与当前帧之间的匹配点（要求点在两帧之间一直被跟踪到，属于稳定共视点），超过20个则计算视差
                     *   2) 两帧匹配点计算本质矩阵E，恢复R、t
                     *   3) 视差超过30像素，匹配内点数超过12个，则认为符合要求，返回当前帧
                     * 3、以上面找到的这一帧为参考系，Pnp计算滑窗每帧位姿，然后三角化所有特征点，构建BA（最小化点三角化前后误差）优化每帧位姿
                     *   1) 3d-2d Pnp求解每帧位姿
                     *   2) 对每帧与l帧、当前帧三角化
                     *   3) 构建BA，最小化点三角化前后误差，优化每帧位姿
                     *   4) 保存三角化点
                     * 4、对滑窗中所有帧执行Pnp优化位姿
                     * 5、Camera与IMU初始化，零偏、尺度、重力方向
                    */
                    result = initialStructure(); //视觉惯性联合初始化
                    initial_timestamp = header;     //更新初始化时间戳
                }
                if(result)//如果初始化成功
                { // 滑窗执行Ceres优化，边缘化，更新滑窗内图像帧的状态（位姿、速度、偏置、外参、逆深度、相机与IMU时差）
                    optimization();//先进行一次滑动窗口非线性优化
                    updateLatestStates();// 用优化后的当前帧位姿更新IMU积分的基础位姿，用于展示IMU轨迹  //获取滑窗中最新帧时刻的状态，并在世界坐标系下进行中值积分；重要：初始化完成后，最新状态在inputIMU函数中发布
                    solver_flag = NON_LINEAR;
                      // 移动滑窗，更新特征点的观测帧集合、观测帧索引（在滑窗中的位置）、首帧观测帧和深度值，删除没有观测帧的特征点
                    slideWindow();//滑动窗口
                    ROS_INFO("Initialization finish!");
                }
                else//滑掉一帧
                    slideWindow(); //初始化失败也需要进行滑窗处理。若global sfm求解引起的失败，则一律移除最老的图像帧；否则，根据addFeatureCheckParallax的判断，决定移除哪一帧
            }
        }




        if( !STEREO && USE_ODOM && DEPTH && !USE_IMU)//todo rgbd+odom
        {
            if(!f_manager.getPoseByWheelOdom(Ps[frame_count],Rs[frame_count], header))
            {
                f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
                cout << "inital初值由pnp给定" << endl;
            }//todo
           cout << "inital" << Ps[frame_count] << endl;
            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            // optimization();

            if(frame_count == WINDOW_SIZE)
            {
                // optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("rgbd + odom finish!");
            }
        }


        // if( !STEREO && USE_ODOM && DEPTH && USE_IMU)//todo rgbd+odom+imu
        // {
        //     if(!f_manager.getPoseByWheelOdom(Ps[frame_count],Rs[frame_count], header))
        //     {
        //         f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        //         cout << "inital初值由pnp给定" << endl;
        //     }//todo
        //     //cout << "inital" << Ps[frame_count] << endl;
        //     //f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        //     f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        //     optimization();

        //     if(frame_count == WINDOW_SIZE)
        //     {
                
        //           optimization();
        //         updateLatestStates();
        //         solver_flag = NON_LINEAR;
        //         slideWindow();
        //         ROS_INFO("rgbd + odom + imu finish!");
        //     }
        // }



        if(STEREO && !USE_IMU && USE_ODOM)//todo 双目+ODOM
        {
            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            if(!f_manager.getPoseByWheelOdom(Ps[frame_count],Rs[frame_count], header))
            {
                f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
                cout << "inital初值由pnp给定" << endl;
            }//todo
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            // optimization();

            if(frame_count == WINDOW_SIZE)
            {
            //    optimization();
               updateLatestStates();
               solver_flag = NON_LINEAR;
               slideWindow();
               ROS_INFO("双目 + ODOM  Initialization finish!");
            }
        }




        if (!STEREO && DEPTH && !USE_IMU && !USE_ODOM)//todo rgbd
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("rgbd finish!");
            }
        }
        

        // stereo + IMU initilization  // 双目+IMU系统初始化
        if(STEREO && USE_IMU)//todo 双目+IMU
        {//第0帧先通过SVD三角化特征点，后续利用其PnP求出后面各帧的位姿，然后再反过来三角化没有三角化的特征点
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric); //有了深度就可以进行求解了 //滑窗满之前，通过PnP求解图像帧之间的位姿，通过三角化初始化路标点（逆深度）
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)          
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs); // 零偏初始化
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]); // 对之前预积分得到的结果进行更新。
                // 预积分的好处查看就在于你得到新的Bgs，不需要又重新再积分一遍，可以通过Bgs对位姿，速度的一阶导数，进行线性近似，得到新的Bgs求解出MU的最终结果
                }
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("双目+IMUInitialization finish!");
            }
        }

        //stereo only initilization
         if(STEREO && !USE_IMU && !USE_ODOM)//todo 双目
         {
             f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
             f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
             optimization();

             if(frame_count == WINDOW_SIZE)
             {
                 optimization();
                 updateLatestStates();
                 solver_flag = NON_LINEAR;
                 slideWindow();
                 ROS_INFO("双目 Initialization finish!");
             }
         }

        if(frame_count < WINDOW_SIZE)//滑窗未满，下一图像帧时刻的状态量用上一图像帧时刻的状态量进行初始化
                                       //（PS：processIMU函数中，会对Ps、Vs、Rs用中值积分进行更新）
        {//todo
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else
    {
        TicToc t_solve;
        if(!USE_IMU) //当不存在imu时，使用pnp方法进行位姿预测;存在imu时使用imu积分进行预测
        //   f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);//直接对下一帧求解位姿  //todo

            if(!f_manager.getPoseByWheelOdom(Ps[frame_count],Rs[frame_count], header))
            {
               f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
               cout << "NON_LINEAR初值由pnp给定" << endl;
            }      //todo
            // cout << "NON_LINEAR" << Ps[frame_count] << endl0

        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        
        // optimization();
        set<int> removeIndex;
        /**
         * 剔除outlier点
         * 遍历特征点，计算观测帧与首帧观测帧之间的重投影误差，计算误差均值，超过3个像素则被剔除
        */
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        
        if (!MULTIPLE_THREAD)// ! 可以试试看(不好用)
        {
            featureTracker.removeOutliers(removeIndex);//若路标点为外点，则对前端图像跟踪部分的信息进行剔除更新;主要包括prev_pts, ids， track_cnt
            // 用当前帧与前一帧位姿变换，估计下一帧位姿，初始化下一帧特征点的位置
            predictPtsInNextFrame();
        }
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())//默认直接返回false，具体判定失败的条件可根据具体场景修正
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState(); //清除状态、重新设置参数，相当于重新开启vio
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();// 删除优化后深度值为负的特征点
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);
// 对划窗的状态进行更新,记录上一次划窗内的初始和最后的位姿
        //todo
        // Eigen::Matrix3d error_rotation = Rs[WINDOW_SIZE].inverse() * last_R;
        // Eigen::Vector3d euler_angle = error_rotation.eulerAngles(2,1,0);
        // error_yaw = euler_angle(0);
        // cout << "角度误差error_yaw : " << euler_angle.transpose() << endl;
        //todo

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
        
    }  
}




/**
 * 系统初始化
 * 1、计算滑窗内IMU加速度的标准差，用于判断移动快慢
 * 2、在滑窗中找到与当前帧具有足够大的视差，同时匹配较为准确的一帧，计算相对位姿变换
 *   1) 提取滑窗中每帧与当前帧之间的匹配点（要求点在两帧之间一直被跟踪到，属于稳定共视点），超过20个则计算视差
 *   2) 两帧匹配点计算本质矩阵E，恢复R、t
 *   3) 视差超过30像素，匹配内点数超过12个，则认为符合要求，返回当前帧
 * 3、以上面找到的这一帧为参考系，Pnp计算滑窗每帧位姿，然后三角化所有特征点，构建BA（最小化点三角化前后误差）优化每帧位姿
 *   1) 3d-2d Pnp求解每帧位姿
 *   2) 对每帧与l帧、当前帧三角化
 *   3) 构建BA，最小化点三角化前后误差，优化每帧位姿
 *   4) 保存三角化点
 * 4、对滑窗中所有帧执行Pnp优化位姿
 * 5、Camera与IMU初始化，零偏、尺度、重力方向
*/

/* ********************************************************************
单目+IMU的初始化过程
* 1. 检测IMU测量值的可观测性，如果IMU运动幅度太小，方差不够大，后续没法初始化
* 2. 将窗口中每一帧图像的特征点压入到 vector<SFMFeature> sfm_f
* 3. relativePose 找出窗口中哪一帧与窗口中最后一帧的视差较大，并求解出 l 帧与当前帧的 R&T
* 4. sfm.construct SfM求解过程
********************************************************************* */
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility//:该作用域段主要用于检测IMU运动是否充分，但是目前代码中，运动不充分时并未进行额外处理，此处可改进；或者直接删除该段
    { // 通过计算线加速度的标准差，是否小于0.25 判断IMU是否有充分运动激励，以进行初始化
    // 注意这里并没有算上all_image_frame的第一帧，所以求均值和标准差的时候要减一(//计算所有图像帧间的平均加速度均值（注意size-1，因为N个图像帧，只有N-1个平均加速度）)
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) // 从第二帧开始检查imu
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt; //通过速度计算，上一图像帧时刻到当前图像帧时刻的平均加速度
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));//标准差
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");// 实际上检查结果并没有用
            //return false;
        }
    }

    

    // Step 2 global sfm
    // 做一个纯视觉slam
    
    // global sfm
    // 将f_manager中的所有feature保存到vector<SFMFeature> sfm_f中
    // 这里解释一下SFMFeature，其存放的是特征点的信息
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;//观测到的路标点的在世界坐标系的位置，索引为路标点的编号
    vector<SFMFeature> sfm_f;
        //     struct SFMFeature 其存放的是特征点的信息
    // {
    //     bool state;//状态（是否被三角化）
    //     int id;
    //     vector<pair<int,Vector2d>> observation;//所有观测到该特征点的图像帧ID和图像坐标
    //     double position[3];//3d坐标
    //     double depth;//深度
    // };
    for (auto &it_per_id : f_manager.feature)// 遍历所有特征点
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame) // 遍历特征点出现的帧
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));//将每一帧图像的所有特征点的归一化坐标的x,y压入sfm_feature数组
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    // 保证具有足够的视差,由E矩阵恢复R、t
    // 这里的第L帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，会作为参考帧到下面的全局sfm使用，得到的Rt为当前帧到第l帧的坐标系变换Rt
    //当前帧==滑窗最后一帧（WINDOW_SIZE）
        /**
     * 在滑窗中找到与当前帧具有足够大的视差，同时匹配较为准确的一帧，计算相对位姿变换
     * 1、提取滑窗中每帧与当前帧之间的匹配点（要求点在两帧之间一直被跟踪到，属于稳定共视点），超过20个则计算视差
     * 2、两帧匹配点计算本质矩阵E，恢复R、t
     * 3、视差超过30像素，匹配内点数超过12个，则认为符合要求，返回当前帧
    */
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }


  /**
     * 以第l帧为参考系，Pnp计算滑窗每帧位姿，然后三角化所有特征点，构建BA（最小化点三角化前后误差）优化每帧位姿
     * 1、3d-2d Pnp求解每帧位姿
     * 2、对每帧与l帧、当前帧三角化
     * 3、构建BA，最小化点三角化前后误差，优化每帧位姿
     * 4、保存三角化点
    */

     // 对窗口中每个图像帧求解sfm问题，得到所有图像帧相对于参考帧的旋转四元数Q、平移向量T和特征点坐标sfm_tracked_points。
     // SfM求解过程 得到固定尺度的每一帧的旋转和平移
    GlobalSFM sfm;
    //frame_count == 10              //只有frame_count == WINDOW_SIZE才会调用initialStructure，此时frame_count即为WINDOW_SIZE
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!"); //global sfm求解失败，对老的图像帧进行边缘化
        marginalization_flag = MARGIN_OLD;
        return false;
    }


// Step 3 solve pnp for all frame
// step2只是针对KF进行sfm，初始化需要all_image_frame中的所有元素，因此下面通过KF来求解其他的非KF的位姿

// 对于所有的图像帧，包括不在滑动窗口中的，提供初始的RT估计，然后solvePnP进行优化
 //对所有图像帧处理，并得到imu与world之间的变换（对所有图像帧处理应该是重新初始化的时候）
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++) //对所有图像帧进行遍历，i为滑窗图像帧index，frame_it为所有图像帧索引； 
                                                                    //滑窗图像帧是所有图像帧的子集,由于滑窗中可能通过MARGIN_SECOND_NEW，边缘化某些中间帧
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])//该图像帧在滑窗里面
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();//得到R_w_i
            frame_it->second.T = T[i];// 初始化不估计平移外参
            i++;
            continue; //若图像帧在滑窗中，直接利用上述global sfm的结果乘以R_c_i，得到imu到world的变换矩阵即R_w_i
        }
        if((frame_it->first) > Headers[i])// 时间戳比较，仅仅在所有图像的时间戳大于等于滑窗中图像帧时间戳时，才递增滑窗中图像时间戳
        {
            i++;
        }
         //注意这里的 Q和 T是图像帧的位姿，而不是求解PNP时所用的坐标系变换矩阵，两者具有对称关系
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix(); // 最近的KF提供一个初始值，Twc -> Tcw，这里i应该还是10??????
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);//罗德里格斯公式将旋转矩阵转换成旋转向量
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)//遍历其所有特征点（路标点编号（int） 相机编号（int）特征点信息（Matrix<double, 7, 1>，归一化相机坐标系坐标（3维）、去畸变图像坐标系坐标（2维）、特征点速度（2维）））
        {
            int feature_id = id_pts.first;//特征点id
            for (auto &i_p  : id_pts.second)//遍历其每个特征点的帧属性
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())  // 有对应的三角化出来的3d点
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>(); //得到路标点在归一化相机坐标系中的位置
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)//保证特征点数大于 6
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        /** 
         *bool cv::solvePnP(    求解 pnp问题
         *   InputArray  objectPoints,   特征点的3D坐标数组
         *   InputArray  imagePoints,    特征点对应的图像坐标
         *   InputArray  cameraMatrix,   相机内参矩阵
         *   InputArray  distCoeffs,     失真系数的输入向量
         *   OutputArray     rvec,       旋转向量
         *   OutputArray     tvec,       平移向量
         *   bool    useExtrinsicGuess = false, 为真则使用提供的初始估计值
         *   int     flags = SOLVEPNP_ITERATIVE 采用LM优化
         *)   
         */
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose(); // 通过PnP求解得到R_w_c
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();//得到R_w_i
        frame_it->second.T = T_pnp; //t_w_i （PS：未考虑camera与imu之间的平移,由于尺度因子未知，此时不用考虑；在求解出尺度因子后，会考虑）
    }
     // Step 4 视觉惯性对齐
    if (visualInitialAlign())// Camera与IMU初始化，零偏、尺度、重力方向
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}



// 视觉和惯性的对其,对应https://mp.weixin.qq.com/s/9twYJMOE8oydAzqND0UmFw中的visualInitialAlign
//估计旋转外参. 2估计陀螺仪bias 3估计中立方向,速度.尺度初始值 4对重力加速度进一步优化 5将轨迹对其到世界坐标系

// 视觉和IMU 求解的初始值互补对齐
bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale  // Camera与IMU初始化，零偏、尺度、重力方向
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);//g == 0.0, 0.0, 9.8
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }


    // change state // 初始化完成之后,对所有状态进行更改change state
    // 首先把对齐后KF的位姿附给滑窗中的值，Rwi twc
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;//并没有用到processIMU里给的初值
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {//得到陀螺仪bias新值，预积分重新传播；此处的预积分为Estimator类中的预积分；
    //图像帧中预积分的重新传播，在计算bg后已经完成
    //Estimator与图像帧中均维护预积分，重复，是否有必要？待优化
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)//利用尺度因子，对t_w_b的更新；注意此时用到了camera与imu之间的平移量
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);// twi - tw0 = toi,就是把所有的平移对齐到滑窗中的第0帧
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    // 把求解出来KF的速度赋给滑窗中
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) //更新Estimator中的速度，注意此时的速度值相对于世界坐标系；
                                                                                          //而初始化过程中的速度，相对于对应的机体坐标系
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }


// 所有的P V Q全部对齐到第0帧的，同时和对齐到重力方向

 //根据基于当前世界坐标系计算得到的重力方向与实际重力方向差异，计算当前世界坐标系的修正量；
//注意：由于yaw不可观，修正量中剔除了yaw影响，也即仅将世界坐标系的z向与重力方向对齐
    Matrix3d R0 = Utility::g2R(g); // g是枢纽帧下的重力方向，得到R_w_j
    double yaw = Utility::R2ypr(R0 * Rs[0]).x(); // Rs[0]实际上是R_j_0，
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;// 第一帧yaw赋0
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;//将世界坐标系与重力方向对齐，之前的世界坐标系Rs[0]根据图像帧定义得到，并未对准到重力方向
    for (int i = 0; i <= frame_count; i++)
    {// 全部对齐到重力下，同时yaw角对齐到第一帧
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();//清除路标点状态，假定所有路标点逆深度均为估计；注意后续优化中路标点采用逆深度，而初始化过程中路标点采用三维坐标
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric); //基于SVD的路标点三角化，双目情形：利用左右图像信息； 非双目情形：利用前后帧图像信息

    return true;
}


/**
 * 在滑窗中找到与当前帧具有足够大的视差，同时匹配较为准确的一帧，计算相对位姿变换
 * 1、提取滑窗中每帧与当前帧之间的匹配点（要求点在两帧之间一直被跟踪到，属于稳定共视点），超过20个则计算视差
 * 2、两帧匹配点计算本质矩阵E，恢复R、t
 * 3、视差超过30像素，匹配内点数超过12个？，则认为符合要求，返回当前帧
*/
//寻找滑窗内一个帧作为枢纽帧，要求和最后一帧既有足够的共视也要有足够的视差
// 该函数判断每帧到窗口最后一帧对应特征点的平均视差大于30，且内点数目大于12？则可进行初始化，同时返回当前帧到第l帧的坐标系变换R和T
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)    // 遍历滑窗
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);      // 提取滑窗中每帧与当前帧之间的匹配点（要求点在两帧之间一直被跟踪到，属于稳定共视点）
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {// 计算匹配点之间的累积、平均视差（归一化相机坐标系下），作为当前两帧之间的视差
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());//计算平均视差

                        //判断是否满足初始化条件：视差>30和内点数满足要求(大于12？)
            //solveRelativeRT()通过基础矩阵计算当前帧与第l帧之间的R和T,并判断内点数目是否足够
            //同时返回窗口最后一帧（当前帧）到第l帧（参考帧）的relative_R，relative_T

            // 有足够的视差在通过本质矩阵恢复第i帧和最后一帧之间的 R t T_i_last
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            { //上述的460表示焦距f（尽管并不太严谨，具体相机焦距并不一定是460），从而在图像坐标系下评估视差
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}




// vector转换成double数组，因为ceres使用数值数组，由于ceres的参数块都是double数组，因此这里把参数块从eigen的表示转成double数组
/*可以看出来，这里面生成的优化变量由：
para_Pose（7维，相机位姿）、
para_SpeedBias（9维，相机速度、加速度偏置、角速度偏置）、
para_Ex_Pose（6维、相机IMU外参）、
para_Feature（1维，特征点深度）、
para_Td（1维，标定同步时间）
五部分组成，在后面进行边缘化操作时这些优化变量都是当做整体看待。*/
void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};

        q.normalize();//todo

        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

 // 当前帧特征点逆深度，限观测帧数量大于4个的特征点
    VectorXd dep = f_manager.getDepthVector();// 特征点逆深度
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
 // 传感器时间同步
    para_Td[0][0] = td;
}





/**
 * 更新优化后的参数，包括位姿、速度、偏置、外参、特征点逆深度、相机与IMU时差
 * double -> eigen 同时fix第一帧的yaw和平移，固定了四自由度的零空间
*/
void Estimator::double2vector()
{// 取出优化前的第一帧的位姿
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)// 如果上一次优化失败了，Rs、Ps都会被清空，用备份的last_R0、last_P0
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

  // 使用IMU时，第一帧没有固定，会有姿态变化
    if(USE_IMU)   // pitch角接近90°
    {// 优化后的第一帧的位姿
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();// yaw角差
        //TODO   // 接近万象节死锁的问题 https://blog.csdn.net/AndrewFan/article/details/60981437
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");//欧拉奇点
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {// 保持第1帧的yaw不变

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // 保持第1帧的位移不变
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }
 // 更新外参
    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }


  // 重新设置各个特征点的逆深度
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];

}



//检测是否发生错误
bool Estimator::failureDetection()
{
    return false; //失败检测策略还可自己探索
    if (f_manager.last_track_num < 2)  // 上一帧老特征点几乎全丢
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)  // 偏置太大
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}



// 基于滑动窗口的紧耦合的非线性优化，残差项的构造和求解
void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    // 滑窗中的帧位姿、速度、偏置、外参、特征点逆深度等参数，转换成数组
    vector2double(); //状态向量用ceres优化方便使用的形式存储

    ceres::Problem problem;
    ceres::LossFunction *loss_function;//核函数

//todo
    ceres::LossFunction *loss_fun;
//todo

    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);//HuberLoss当预测偏差小于 δ 时，它采用平方误差,当预测偏差大于 δ 时，采用的线性误差。
//todo
    loss_fun = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
//todo
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    /**
     * Step1. 调用AddParameterBlock，显式添加待优化变量（类似于g2o中添加顶点），需要固定的顶点固定一下
     *  参数块 1： 滑窗中位姿包括位置和姿态，共11帧
    */

    for (int i = 0; i < frame_count + 1; i++)
    { // 由于姿态不满足正常的加法，也就是李群上没有加法，因此需要自己定义他的加法
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);//https://zhuanlan.zhihu.com/p/567635409     //https://blog.csdn.net/qq_42700518/article/details/105898222
        if(USE_IMU)  // AddParameterBlock   向该问题添加具有适当大小和参数化的参数块。
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU) // 如果不使用IMU，固定第一帧位姿，IMU下第一帧不固定
    // SetParameterBlockConstant 在优化过程中，使指示的参数块保持恒定。设置任何参数块变成一个常量
    // 固定第一帧的位姿不变!  这里涉及到论文2中的
        problem.SetParameterBlockConstant(para_Pose[0]);//双目版本时，Rs[0]、Ps[0]固定

  // 参数块 2： 相机imu间的外参
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)//速度过低时，不估计td
        {
            
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;//打开外部估计
            ROS_WARN_STREAM("openExEstimation");

        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);  // 不估计外参的时候，固定外参
            // ROS_WARN_STREAM("fix extinsic param");
        }
    }

    problem.AddParameterBlock(para_Td[0], 1);
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);




  //构建残差
    /*******先验残差*******/
     // 在问题中添加先验信息作为约束
    // construct new marginlization_factor  会调用marginalization_factor——>Evaluate计算边缘化后的残差与雅克比
     // 上一次的边缘化结果作为这一次的先验
    if (last_marginalization_info && last_marginalization_info->valid)//last_marginalization_info->valid待优化m为0时为false
    {
        // construct new marginlization_factor
        /* 通过提供参数块的向量来添加残差块。
        ResidualBlockId AddResidualBlock(
            CostFunction* cost_function,//损失函数
            LossFunction* loss_function,//核函数
            const std::vector<double*>& parameter_blocks); */
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }


    /*******预积分残差*******/
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)// 时间过长这个约束就不可信了
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
              // 后面四个参数为变量初始值，优化过程中会更新
               //添加残差格式：残差因子，鲁棒核函数，优化变量（i时刻位姿，i时刻速度与偏置，i+1时刻位姿，i+1时刻速度与偏置）
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }



   /*******重投影残差*******/
    //重投影残差相关，此时使用了Huber损失核函数
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)// 遍历特征点
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point; // 首帧归一化相机平面点

        for (auto &it_per_frame : it_per_id.feature_per_frame) // 遍历特征点的观测帧
        {
            imu_j++;
            if (imu_i != imu_j)   // 非首帧观测帧
            {
                Vector3d pts_j = it_per_frame.point;  // 当前观测帧归一化相机平面点
                // 首帧与当前观测帧建立重投影误差
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                // 优化变量：首帧位姿，当前帧位姿，外参（左目），特征点逆深度，相机与IMU时差
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }//todo 原loss_function
  // 双目，重投影误差
            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {  // 首帧与当前观测帧右目建立重投影误差
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                // 优化变量：首帧位姿，当前帧位姿，外参（左目），外参（右目），特征点逆深度，相机与IMU时差
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {// 首帧左右目建立重投影误差
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);  // 优化变量：外参（左目），外参（右目），特征点逆深度，相机与IMU时差
                }
               
            }
            f_m_cnt++;
        }
    }



    //todo  与feature select的odom消息冲突，徐添加新的消息
    if(USE_ODOM)
    {
        // 1. 同一时刻vins与里程计位置对齐，约束
        //  //for (int i = 0; i < frame_count + 1; i++)
        // // if(frame_count == 10)
        // {
        //     Eigen::Matrix3d R_odom;
        //     Eigen::Vector3d T_odom;
        //     R_odom.setIdentity();
        //     T_odom.setZero();
        //     //if(f_manager.getPoseByWheelOdom(T_odom,R_odom,Headers[i]))
        //     if(f_manager.getPoseByWheelOdom(T_odom,R_odom,Headers[frame_count]))
        //     {
        //         // T_odom = Rs[frame_count] * R_odom.inverse() * T_odom;
        //         double t_x = T_odom.x();
        //         double t_y = T_odom.y();
        //         double t_z = T_odom.z();

        //         Eigen::Quaterniond q = Eigen::Quaterniond(R_odom);
                
        //         // q.normalize();

        //         double q_x = q.x();
        //         double q_y = q.y();
        //         double q_z = q.z();
        //         double q_w = q.w();
        //         ceres::CostFunction * odom = OdomError::Create(t_x,t_y,t_z,q_w,q_x,q_y,q_z);

        //         //! 9.3 简单实现离线标定(不好用)    下面的相对变换是否可以用于在线标定？参考InitialEXRotation::CalibrationExRotation（）函数，函数里的最小二乘可替换当前的
        //         //! 11.16  实机odom时再添加  离线标定参考本文件631行    ，   在线参考1521行
        //         // double t_error[3];
        //         // t_error[0] = para_Pose[frame_count][0] - t_x;
        //         // t_error[1] = para_Pose[frame_count][1] - t_y;
        //         // t_error[2] = para_Pose[frame_count][2] - t_z;

        //         // Eigen::Quaterniond q_cam;
        //         // q_cam.x() = para_Pose[frame_count][3];
        //         // q_cam.y() = para_Pose[frame_count][4];
        //         // q_cam.z() = para_Pose[frame_count][5];
        //         // q_cam.w() = para_Pose[frame_count][6];
        //         // Eigen::MatrixX3d R_cam = q_cam.normalized().toRotationMatrix();

        //         // Eigen::MatrixX3d R_error = R_odom.transpose() * R_cam;

        //         // cout << "T_o_c: " << t_error[0] << " " <<  t_error[1] << " " <<  t_error[2] << endl; 
        //         // cout << "R_o_c: " << R_error << endl;




        //         // problem.AddResidualBlock(odom,loss_function,para_Pose[frame_count]);//! best
        //         // problem.AddResidualBlock(odom,loss_fun,para_Pose[frame_count]);
        //         problem.AddResidualBlock(odom,NULL,para_Pose[frame_count]);
        //         // cout << "odom optimization" << endl;
        //     }
        // }


        //2.约束相邻里程计之间的相对变换
        //todo  kitti时得注释掉feature_manager.cpp的1140，1147
        // if(frame_count == 10)
        // {
        //  int j = 0;
        //  for (int i = 0; i < frame_count; i++)
        //  {
            
        //      if(frame_count != 0)
        //      {
                

        //          j++;
        //          Eigen::Matrix3d R_odom;
        //          Eigen::Vector3d T_odom;
        //          R_odom.setIdentity();
        //          T_odom.setZero();

        //          if(f_manager.getRelativePoseByWheel(R_odom,T_odom,Headers[i],Headers[j]))
        //          {
                    
        //              double t_x = T_odom.x();
        //              double t_y = T_odom.y();
        //              double t_z = T_odom.z();

        //              Eigen::Quaterniond q = Eigen::Quaterniond(R_odom);
        //              double q_x = q.x();
        //              double q_y = q.y();
        //              double q_z = q.z();
        //              double q_w = q.w();

                     
        //              ceres::CostFunction * odom = RelativeOdomError::Create(t_x,t_y,t_z,q_w,q_x,q_y,q_z,1,1); //! 100(100)   0.01(0.01)  0.001(0.001) 1000000(1000000)
                     
        //              problem.AddResidualBlock(odom,NULL,para_Pose[i],para_Pose[j]);//todo  0.01(0.01)    rel(0.01)   rel2(1)
        //             //  problem.AddResidualBlock(odom,loss_function,para_Pose[i],para_Pose[j]);
        //             //  problem.AddResidualBlock(odom,loss_fun,para_Pose[i],para_Pose[j]);//todo  1与null差不多
        //             // ROS_WARN_STREAM("odom optimization");
                    
        //          }
        //      }
        //  }
        // }
	// cout << "odom optimization" << endl;
    }

    //todo



    //ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());



    // ------------------------------------写下来配置优化选项,并进行求解-----------------------------------------
//优化参数配置
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR; //normal equation求解方法
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0; // 下面的边缘化老的操作比较多，因此给他优化时间就少一些
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    //todo 12.12 https://blog.csdn.net/qq_69198927/article/details/132895537
    // static double ci_last = 0;

    // delta_ci = summary.initial_cost - ci_last;

    // ci_last = summary.initial_cost;

    // if(ceres_delta_ci != NULL) fprintf(ceres_delta_ci, "%f\n", delta_ci);

    
    // cout << "summary.final_cost = " << summary.final_cost << endl;
    // cout << "summary.initial_cost = " << summary.initial_cost << endl;
    // cout << "summary.total_time_in_seconds = " << summary.total_time_in_seconds << endl;
    // cout << "summary.iterations = " << summary.iterations.size() << endl;
    // if(ceres_ci != NULL && ceres_cf != NULL && ceres_times != NULL && ceres_nums != NULL && ceres_a_times != NULL && ceres_a_nums != NULL)
    // {
    //     cout << "ceres printf" << endl;
    //     fprintf(ceres_ci, "%f\n", summary.initial_cost);
    //     fprintf(ceres_cf, "%f\n", delta_c);
    //     fprintf(ceres_times, "%f\n", summary.total_time_in_seconds);
    //     fprintf(ceres_nums, "%d\n", static_cast<int>(summary.iterations.size()));
    //     // fprintf(ceres_a_times, "%f\n", summary.initial_cost + (summary.final_cost - summary.initial_cost) * (1 - summary.total_time_in_seconds / 0.08));
    //     // fprintf(ceres_a_nums, "%f\n", summary.initial_cost + (summary.final_cost - summary.initial_cost) * (1 - static_cast<int>(summary.iterations.size()) / 10));
    // }
    //todo 12.12 12.22

    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    double2vector();  // 更新优化后的参数，包括位姿、速度、偏置、外参、特征点逆深度、相机与IMU时差
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)//滑窗未满
        return;
    



   /*%%%%%滑窗满了，进行边缘化处理%%%%%%%*/
    // 以下是边缘化操作
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo(); // 一个用来边缘化操作的对象
        vector2double();// 滑窗中的帧位姿、速度、偏置、外参、特征点逆深度等参数，转换成数组

 // 关于边缘化有几点注意的地方
        // 1、找到需要边缘化的参数块，这里是地图点，第0帧位姿，第0帧速度零偏
        // 2、找到构造高斯牛顿下降时跟这些待边缘化相关的参数块有关的残差约束，那就是预积分约束，重投影约束，以及上一次边缘化约束
        // 3、这些约束连接的参数块中，不需要被边缘化的参数块，就是被提供先验约束的部分，也就是滑窗中剩下的位姿和速度零偏

  // 先验残差
        if (last_marginalization_info && last_marginalization_info->valid)//valid在new MarginalizationInfo()构造时为true，只有当m=0时才为false
        { // last_marginalization_parameter_blocks是上一次边缘化对哪些当前参数块有约束
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            { // 上一次Marg剩下的参数块
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||// 涉及到的待边缘化的上一次边缘化留下来的当前参数块只有位姿和速度零偏
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);

            /* 是为了将不同的损失函数_cost_function以及优化变量_parameter_blocks统一起来再一起添加到marginalization_info中
            ResidualBlockInfo(ceres::CostFunction *_cost_function, 
                            ceres::LossFunction *_loss_function, 
                            std::vector<double *> _parameter_blocks, 
                            std::vector<int> _drop_set) */
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);


            marginalization_info->addResidualBlockInfo(residual_block_info); // 将上一步marginalization后的信息作为先验信息
        }



 // 滑窗首帧与后一帧之间的IMU残差 // 然后添加第0帧和第1帧之间的IMU预积分值以及第0帧和第1帧相关优化变量
        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1}); // 这里就是第0和1个参数块是需要被边缘化的,也就是para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }



 // 滑窗首帧与其他帧之间的视觉重投影残差
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0) // 只找能被第0帧看到的特征点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point; //左相机在i时刻、在j时刻分别观测到路标点
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,  //todo,原loss_function
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},//优化变量
                                                                                        vector<int>{0, 3});//为0和3的原因是，para_Pose[imu_i]是第一帧的位姿，需要marg掉，而3是para_Feature[feature_index]是和第一帧相关的特征点，需要marg掉 
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        { //左相机在i时刻、右相机在j时刻分别观测到路标点
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        { //左相机在i时刻、右相机在i时刻分别观测到路标点
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }



        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());


// 即将滑窗，因此记录新地址对应的老地址

//在optimization的最后会有一部滑窗预移动的操作
// 值得注意的是，这里仅仅是相当于将指针进行了一次移动，指针对应的数据还是旧数据，因此需要结合后面调用的 slideWindow() 函数才能实现真正的滑窗移动

//仅仅改变滑窗double部分地址映射，具体值的通过slideWindow和vector2double函数完成；记住边缘化仅仅改变A和b，不改变状态向量
//由于第0帧观测到的路标点全被边缘化，即边缘化后保存的状态向量中没有路标点;因此addr_shift无需添加路标点

 // marg首帧之后，将参数数组中每个元素的地址设为前面元素的地址，记录到addr_shift里面
        // [<p1,p0>,<p2,p1>,...]
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)//从1开始，因为第一帧的状态不要了
        { //这一步的操作指的是第i的地址指针存放的的是i-1的地址指针，这就意味着窗口向前移动了一格 

            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];//因此para_Pose这些变量都是双指针变量，因此这一步是指针操作
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
// parameter_blocks实际上就是addr_shift的索引的集合及搬进去的新地址
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;

        // 保存marg信息
        last_marginalization_info = marginalization_info; // 本次边缘化的所有信息
        last_marginalization_parameter_blocks = parameter_blocks;  // 代表该次边缘化对某些参数块形成约束，这些参数块在滑窗之后的地址
        
    }
    else
    {   // 要求有上一次边缘化的结果同时，即将被margin掉的在上一次边缘化后的约束中
        // 预积分结果合并，因此只有位姿margin掉
         //存在先验边缘化信息时才进行次新帧边缘化;否则仅仅通过slidewindow，丢弃次新帧
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))//https://blog.csdn.net/phd17621680432/article/details/122562641
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);//仅仅只边缘化WINDOW_SIZE - 1位姿变量， 对其特征点、图像数据不进行处理？？？？why
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
//https://blog.csdn.net/hltt3838/article/details/109649675
//由于边缘化次新帧，边缘化的状态向量仅为para_Pose[WINDOW_SIZE - 1];而保留的状态向量为在上一次边缘化得到的保留部分基础上、剔除para_Pose[WINDOW_SIZE - 1]的结果;
//因此，边缘化次新帧得到的保留部分也未包含路标点，因此addr_shift无需添加路标点
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)//WINDOW_SIZE - 1会被边缘化，不保存
                    continue;
                else if (i == WINDOW_SIZE)// 滑窗，最新帧成为次新帧
                {//WINDOW_SIZE数据保存到WINDOW_SIZE-1指向的地址
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {//其余的保存地址不变
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift); //提取保存的数据
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}



// 移动滑窗，更新特征点的观测帧集合、观测帧索引（在滑窗中的位置）、首帧观测帧和深度值，删除没有观测帧的特征点
void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0]; // 备份
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        { //1、滑窗中的数据往前移动一帧；运行结果就是WINDOW_SIZE位置的状态为之前0位置对应的状态
        // 0,1,2...WINDOW_SIZE——>1,2...WINDOW_SIZE,0
            for (int i = 0; i < WINDOW_SIZE; i++) // 道理很简单,就是把前后元素交换,这样的话最后的结果是1234567890
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
        
            //2、处理前，WINDOW_SIZE位置的状态为之前0位置对应的状态；处理后，WINDOW_SIZE位置的状态为之前WINDOW_SIZE位置对应的状态;之前0位置对应的状态被剔除
            // 0,1,2...WINDOW_SIZE——>1,2...WINDOW_SIZE,WINDOW_SIZE
             // 新增末尾帧，用当前帧数据初始化
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};//重新构造一个预积分，使用WINDOW_SIZE帧的acc_0, gyr_0和偏置
  // buffer清空，等待新的数据来填
                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

 //3、对时刻t_0(对应滑窗第0帧)之前的所有数据进行剔除；即all_image_frame中仅保留滑窗中图像帧0与图像帧WINDOW_SIZE之间的数据
            if (true || solver_flag == INITIAL) // 删除首帧image    // 清空all_image_frame最老帧之前的状态
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration; // 预积分量是堆上的空间，因此需要手动释放
                all_image_frame.erase(all_image_frame.begin(), it_0); // 释放完空间之后再erase
            }
            slideWindowOld(); // 移动滑窗，从特征点观测帧集合中删除该帧，计算新首帧深度值
        }
    }
    else// 将最后两个预积分观测合并成一个
    {
        if (frame_count == WINDOW_SIZE)
        {//0,1,2...WINDOW_SIZE-2, WINDOW_SIZE-1, WINDOW_SIZE——>0,,1,2...WINDOW_SIZE-2,WINDOW_SIZE, WINDOW_SIZE
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);//预积分的传播

                    dt_buf[frame_count - 1].push_back(tmp_dt);//数据保存有冗余，integration_base中也保存了同样的数据
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];
   // reset最新预积分量
                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }//更新第一次观测到路标点的图像帧的索引
            slideWindowNew();  // 边缘化当前帧前面一帧后，从特征点的观测帧集合中删除该帧，如果特征点没有观测帧了，删除这个特征点
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// 由于地图点是绑定在第一个看见它的位姿上的，因此需要对被移除的帧看见的地图点进行解绑，以及每个地图点的首个观测帧id减1
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)//如果不是初始化 //非线性优化阶段，除了更新第一次观测到路标点的图像帧的索引，还需更新路标点的逆深度
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];//R0、P0表示被边缘化的图像帧，即老的第0帧的位姿； R1、P1表示新的第0帧的位姿
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        /**
         * 边缘化第一帧后，从特征点的观测帧集合中删除该帧，观测帧的索引相应全部-1，如果特征点没有观测帧少于2帧，删除这个特征点
         * 与首帧绑定的estimated_depth深度值，重新计算
        */
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();//初始化未完成，只是更新第一次观测到路标点的图像帧的索引
}

// 得到当前帧的变换矩阵T 当前帧位姿 Twi
void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}
// 得到某一个index处图像的变换矩阵T  位姿 Twi
void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}



/**
 * 用当前帧与前一帧位姿变换，估计下一帧位姿，初始化下一帧特征点的位置
*/
void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;

    // TODO:动态检测 这里做了一个简单的预测!!可以用来做动态检测
    // 使用匀速模型预测下一个位置predict next pose. Assume constant velocity motion
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
       // 用前一帧位姿与当前帧位姿的变换，预测下一帧位姿
    nextT = curT * (prevT.inverse() * curT);//假设这一次的位姿变化和上一次相同
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)  //仅对观测次数不小于两次、且在最新图像帧中观测到的路标点进行预测
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));  //路标在在下一时刻（预测的）体坐标系下坐标
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]); ////路标在在下一时刻（预测的）相机坐标系下坐标
                int ptsIndex = it_per_id.feature_id;// 根据路标点编号，存储预测的坐标
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }// 设置下一帧跟踪点初始位置
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}



// 计算重投影误差
double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi; //路标点在世界坐标系下的坐标
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj); //路标点在j时刻左或右相机坐标系下的坐标
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>(); //归一化相机坐标系下的重投影误差
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);//返回重投影误差均方根
}



/**
 * 剔除outlier点
 * 遍历特征点，计算观测帧与首帧观测帧之间的重投影误差，计算误差均值，超过3个像素则被剔除
*/
void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)// 对观测少于4次的路标点，不进行外点判断
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)  // 非首帧观测帧
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo) // 双目，右目同样计算一次
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)//不同时刻，左右图像帧之间的重投影误差
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else//相同时刻，左右图像帧之间的重投影误差
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;// 重投影误差均值（归一化相机坐标系）
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}



 //IMU中值积分，计算位姿与速度，注意此时的中值积分在世界坐标系下进行
// 使用上一时刻的姿态进行快速的imu预积分
// 用来预测最新P,V,Q的姿态
// -latest_p,latest_q,latest_v,latest_acc_0,latest_gyr_0 最新时刻的姿态。这个的作用是为了刷新姿态的输出，但是这个值的误差相对会比较大，是未经过非线性优化获取的初始值。
void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}


/**
 * 用优化后的当前帧位姿更新IMU积分的基础位姿，用于展示IMU轨迹
 * // 让此时刻的值都等于上一时刻的值,用来更新状态
*/
void Estimator::updateLatestStates()  //获取滑窗中最新帧时刻的状态，并在世界坐标系下进行中值积分；初始化完成后，最新状态在inputIMU函数中发布
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;//这里的acc和gyr应该是比frame_out更新的一组数据
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty())
    {// 当前帧之后的一部分imu数据，用当前帧位姿预测，更新到最新，这个只用于IMU路径的展示
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);//世界坐标系下进行中值积分
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}
