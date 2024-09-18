/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

//todo
//使用rosNodeTest.cpp中的wheel_odom_buf数据
queue<nav_msgs::Odometry::ConstPtr> wheel_odom_buf;
std::mutex m_wheel_odom;
//todo

//得到该特征点最后一次跟踪到的帧号
int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}



FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}



void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}



void FeatureManager::clearState()
{
    feature.clear();
}


// 返回观测次数超过4次的路标点数目
// 得到这一帧上特征点的数量//只统计了用了4次以上的点
int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}



/* addFeatureCheckParallax
对倒数第二与倒数第三帧进行视差比较，如果是当前帧变化很小，就会删去倒数第二帧，如果变化很大，就删去最旧的帧。并把这一帧作为新的关键帧
这样也就保证了划窗内优化的,除了最后一帧可能不是关键帧外,其余的都是关键帧
VINS里为了控制优化计算量，在实时情况下，只对当前帧之前某一部分帧进行优化，而不是全部历史帧。局部优化帧的数量就是窗口大小。
为了维持窗口大小，需要去除旧的帧添加新的帧，也就是边缘化 Marginalization。到底是删去最旧的帧（MARGIN_OLD）还是删去刚
刚进来窗口倒数第二帧(MARGIN_SECOND_NEW)
如果大于最小像素,则返回true */

/**
 * 
 * 添加特征点记录，并检查当前帧是否为关键帧
 * @param frame_count   当前帧在滑窗中的索引
 * @param image         当前帧特征（featureId，cameraId，feature）
*/

// 计算上一的帧与上上一帧中特征点的平均视差 
// return:  true 视差较大， false视差较小
// ture 说明上一帧是关键帧， false说明上一帧是非关键帧
//todo 7
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size()); //当前帧的特征点数量
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;//所有特征点视差总和
    int parallax_num = 0;//满足某些条件的特征点个数
    last_track_num = 0;//被跟踪点的个数
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);  //todo 7
        assert(id_pts.second[0].first == 0);//如果它的条件返回错误，则终止程序执行
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second); //todo 7
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first; //该图像内,每一个特征点的id

  // find_if 函数，找到一个interator使第三个仿函数参数为真
           // 在已有的id中寻找是否是有相同的特征点
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        { // 在特征点管理器中，新创建一个特征点id，这里的frame_count就是该特征点在滑窗中的当前位置，作为这个特征点的起始位置（start_frame）
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        //如果找到了相同ID特征点，就在其FeaturePerFrame内增加此特征点在此帧的位置以及其他信息，
        // 然后增加last_track_num，说明此帧有多少个相同特征点被跟踪到
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)//该路标点被至少四个图像帧观测到
                long_track_num++;
        }
    }

    // 关键帧判断的策略1（PS:long_track_num < 40没啥用）

    //a、滑窗中图像帧数目少于2帧;  b、跟踪到的路标点数量少于20个;  c、新观测到的路标点数目大于0.5倍跟踪的路标点数目;
    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
     // 前两帧都设置为KF，追踪过少也认为是KF
       //如果刚开始，或者追踪质量太差，直接选为关键帧
       // 新点比较多，认为是关键帧
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;  


    // 两帧之前的特征点，一直跟踪到了当前帧还在
    
    // 计算的实际上是frame_count-1,也就是前一帧是否为关键帧
    // 因此起始帧至多得是frame_count - 2,同时至少覆盖到frame_count - 1帧
    for (auto &it_per_id : feature) //关键帧判断的策略2（视差足够大（PS:注意计算视差的路标点需满足条件判断））：(其实上面那个注释好些)//最开始观测到该路标点的图像帧，距离最新帧不小于2 // 在最新帧中能够被观测到
    {
        if (it_per_id.start_frame <= frame_count - 2 && 
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {  // 计算当前特征点在前两帧中，归一化相机平面上的距离
            parallax_sum += compensatedParallax2(it_per_id, frame_count);//todo 增加了利用rgbd来判断位移，但没想好判断阈值  
            parallax_num++;
        }
    }

    if (parallax_num == 0) // 这个和上一帧没有相同的特征点
    {
        return true;
    }
    else
    {// 前两帧之间视差是否足够大（像素坐标系下10个像素），足够大，删除滑窗最早帧；不够大，删除倒数第二帧； MIN_PARALLAX = 10 / f
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX; //关键帧判断策略2, 平均视差大于设定的最小值
    }
}

/**
 * @brief 得到同时被frame_count_l frame_count_r帧看到的特征点在各自的坐标
 *  * 提取两帧的匹配点，要求这些点在两帧之间一直被跟踪到
 */
//找出两帧图像之间相匹配的特征点
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{//获取路标点在图像帧frame_count_l与图像帧frame_count_r的左归一化相机坐标系的位置pair<Vector3d, Vector3d>
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)// 保证需要的特征点被这两帧都观察到
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

//! no use
//todo 12.25 use for calibrating camera extrinsic between camera and odom
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding_3d_points(int frame_count_l)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame == frame_count_l && it.solve_flag == 1)// 保证需要的特征点被这两帧都观察到
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();

            a = it.feature_per_frame[it.start_frame].point;

            b = it.feature_per_frame[it.start_frame].point * it.estimated_depth;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}


// 设置深度,这个在void Estimator::double2vector()中用了
// 如果失败,把solve_flag设置为2
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);//深度
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}




void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}



void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}



/**
 * 当前帧特征点逆深度，限观测帧数量大于4个的特征点
*/
VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

/**
 * SVD计算三角化点
 * @param Pose0     位姿Tcw
 * @param Pose1     位姿Tcw
 * @param point0    归一化相机平面点
 * @param point1    归一化相机平面点
 * @param point_3d  output 三角化点，世界坐标点
*/
//https://blog.csdn.net/weixin_37953603/article/details/108035773
void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}



//TODO 有问题（4.1)
//! 能用（5.8） 
void FeatureManager::solvePoseByICP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, vector<cv::Point3f> &pts3D_world, vector<cv::Point3f> &pts3D_camera)
{
    cv::Point3f p1,p2;
    int N = pts3D_world.size();
    for (int i = 0; i < N; i++)
    {
        p1 += pts3D_world[i];
        p2 += pts3D_camera[i];
    }
    p1 = p1 / N;
    p2 = p2 / N;
    vector<cv::Point3f> q1(N),q2(N);

    for (int i = 0; i < N; i++)
    {
        q1[i] = pts3D_world[i] - p1;
        q2[i] = pts3D_camera[i] - p2;
    }
    
    Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        w += Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z) * Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();
    }

    //SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(w , Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    R_initial = U * (V.transpose());

    if(R_initial.determinant() < 0)
        R_initial = -R_initial;
    
    P_initial = Eigen::Vector3d(p1.x,p1.y,p1.z) - R_initial * Eigen::Vector3d(p2.x,p2.y,p2.z);
    //printf("ICP finsh\n");
}
//TODO



// 求解新图像的位姿
/**
 * 3d-2d求解位姿，Twc
 * @param R     Rwc
 * @param P     twc
 * @param pts2D 像素坐标
 * @param pts3D 世界坐标
*/
bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)// 特征点太少
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);//旋转矩阵到旋转向量
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r); //旋转向量到旋转矩阵
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}


/**
 * 3d-2d Pnp求解当前帧位姿
 * 1、世界坐标-归一化坐标
 * 2、用前一帧位姿初始化当前帧位姿
*/
// 有了深度，当下一帧照片来到以后就可以利用pnp求解位姿了
// Ps, Rs, tic, ric是得到的结果
void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{// 先判断当前特征中那些已经三角化出深度的点，计算出世界系坐标存入pts3D，相应的当前帧的归一化平面坐标存入pts2D

    if(frameCnt > 0)//对第一帧图像不做处理；因为此时路标点还未三角化，需要利用第一帧双目图像，进行路标点三角化
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D_world;
        vector<cv::Point3f> pts3D_camera;//TODO

        for (auto &it_per_id : feature)
        {
            // if(DEPTH)//TODO
            // {//!下面这个if的条件感觉重复了，不会出现>0.4和<4的
            //     int index = frameCnt - it_per_id.start_frame;
            //     if (it_per_id.estimated_depth > 0.17 && it_per_id.estimated_depth < 4 && it_per_id.feature_per_frame[index].pointRight.x() > 0.17 && it_per_id.feature_per_frame[index].pointRight.x() < 4)
            //     {
                    
            //         if((int)it_per_id.feature_per_frame.size() >= index + 1)// 该路标点从start_frame图像帧到frameCnt对应的图像帧都能被观测到
            //         { 
                    
            //             Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
            //             // 通过第一帧的位姿转到世界系 Twi
            //             Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];
            //             cv::Point3f point3d_world(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());

            //             // cout << "point3d_world  x:" << point3d_world.x << endl;
            //             // cout << "point3d_world  y:" << point3d_world.y << endl;
            //             // cout << "point3d_world  depth:" << point3d_world.x << endl;

            //             cv::Point3f point3d_camera((it_per_id.feature_per_frame[index].point.x() * it_per_id.feature_per_frame[index].pointRight.x()), (it_per_id.feature_per_frame[index].point.y() * it_per_id.feature_per_frame[index].pointRight.x()), it_per_id.feature_per_frame[index].pointRight.x());

            //             // cout << "point3d_camera  x:" << point3d_camera.x << endl;
            //             // cout << "point3d_camera  y:" << point3d_camera.y << endl;
            //             // cout << "point3d_camera  depth:" <<it_per_id.feature_per_frame[index].pointRight.x() << endl;

            //             pts3D_world.push_back(point3d_world);
            //             pts3D_camera.push_back(point3d_camera); 
            //         }
            //     }
            //     //printf("push_back pts3D_world and pts3D_camera\n");
            // }
            // else
            {
                    if (it_per_id.estimated_depth > 0)
                {
                    int index = frameCnt - it_per_id.start_frame;
                    if((int)it_per_id.feature_per_frame.size() >= index + 1)// 该路标点从start_frame图像帧到frameCnt对应的图像帧都能被观测到
                    { // 在第一帧IMU下的坐标
                    //路标点在IMU坐标系坐标，第一次看到该路标点的图像帧时刻  // 前面描述错误，此时不存在imu，应该是在start_frame图像帧左相机坐标系的坐标
                        Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                        // 通过第一帧的位姿转到世界系 Twi
                        Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                        cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                        cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//frameCnt帧的归一化坐标
                        pts3D_world.push_back(point3d);
                        pts2D.push_back(point2d); 
                    }
                }
            }//todo

        }

        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];//frameCnt的前一帧作为初始变换
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        // if(DEPTH)
        // {       Eigen::Matrix3d R_;
        //         Eigen::Vector3d t_;

        //         solvePoseByICP(R_, t_, pts3D_world, pts3D_camera);
        //         RCam = R_;
        //         PCam = t_;
        //         //cout << "icp位移" << t_.x() << t_.y() << t_.z() << endl;
        //         // trans to w_T_imu
        //         Rs[frameCnt] = RCam * ric[0].transpose(); 
        //         Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;//用向量画画看咯
            
        // }
        // else
        {
                if(solvePoseByPnP(RCam, PCam, pts2D, pts3D_world))
            {
                // trans to w_T_imu
                Rs[frameCnt] = RCam * ric[0].transpose(); 
                Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;//用向量画画看咯

                Eigen::Quaterniond Q(Rs[frameCnt]);  //:这个四元数并未使用 
                //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
                //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
            }
        }
        //TODO
    }
}



/**
 * 三角化当前帧特征点
*/
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;
  // 双目三角化
        if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
        {// 起始观测帧位姿 Tcw
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;//https://blog.csdn.net/weixin_37953603/article/details/108035773
            //cout << "left pose " << leftPose << endl;
// 起始观测帧位姿，右目 Tcw
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint; // 相机点
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)//! 还可以加个< 多少的约束
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        // 单目三角化，观测帧至少要2帧，利用前后帧图像对路标点进行三角化
        else if(it_per_id.feature_per_frame.size() > 1 && !DEPTH)//TODO
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            //todo 这个是先三角化不行再用rgbd的深度，试试看哪种好一些，如果要用这个需要把上面的if条件里的!DEPTH去掉
            // else if(DEPTH && it_per_id.feature_per_frame[0].is_stereo)
            //     double depth = it_per_id.feature_per_frame[0].pointRight.x();
            //     if (depth > 0 && depth < 7)//! 这个距离应该要改
            //         it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }//TODO
        else if(DEPTH && it_per_id.feature_per_frame[0].is_stereo)
        {
            double depth = it_per_id.feature_per_frame[0].pointRight.x();
            if (depth > 0.17 && depth < 4)//! 这个距离应该要改3.30 已改(5.9)
            {
                it_per_id.estimated_depth = depth;
                //cout << "RGBD depth: " << depth << "m" << endl;
            }
            else if(it_per_id.feature_per_frame.size() > 1)//if(it_per_id.feature_per_frame.size() > 1)//todo
            {
                int imu_i = it_per_id.start_frame;
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
                Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;

                imu_i++;
                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
                Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[1].point.head(2);
                triangulatePoint(leftPose, rightPose, point0, point1, point3d);
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
                double depth = localPoint.z();
                if (depth > 0)
                    it_per_id.estimated_depth = depth;
                else
                    it_per_id.estimated_depth = INIT_DEPTH;
                
                //cout << "三角化 depth" << depth << endl;
            }
            else
            {
                it_per_id.estimated_depth = INIT_DEPTH;
            }
           // printf("rgbd 三角化完成\n");
            continue;
        }
    //TODO  



        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;



// todo 会走到这里吗，下面是vins-mono的方法
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;
  // 首帧观测帧的位姿 Twc
        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();
  // 遍历当前特征点观测帧，与首帧观测帧构建最小二乘，SVD三角化
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
             // 与首帧观测帧之间的位姿变换 Tij
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            // Tji
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];
  // 得到的深度值实际上就是第一个观察到这个特征点的相机坐标系下的深度值
        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {// 具体太近就设置成默认值
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}
/**
 * 剔除outlier特征点，在滑窗优化之后计算重投影误差，剔除误差过大点
*/
void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

/**
 * 边缘化第一帧后，从特征点的观测帧集合中删除该帧，观测帧的索引相应全部-1，如果特征点没有观测帧少于2帧，删除这个特征点
 * 与首帧绑定的estimated_depth深度值，重新计算
 * /**
 * @param[in] marg_R  被移除的位姿
 * @param[in] marg_P 
 * @param[in] new_R    转接地图点的位姿
 * @param[in] new_P 
 */
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
  //第一次观察到该路标点的图像帧不是被边缘化掉的老的第0帧，将start_frame递减1
        if (it->start_frame != 0)
            it->start_frame--;
        else
        { // 首帧观测帧的归一化相机点
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            // 删除当前marg掉的观测帧
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            { // 如果观测帧少于2个，删掉该点
                feature.erase(it);
                continue;
            }
            else // 该路标点被观测的次数满足要求，将深度信息在新的第0个图像帧中进行表示
            {  // estimated_depth是在首帧观测帧下的深度值，现在更新到后一帧下
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // 在边缘化之后删除跟踪丢失的特征
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}


/**
 * 边缘化第一帧后，从特征点的观测帧集合中删除该帧，观测帧的索引相应全部-1，如果特征点没有观测帧了，删除这个特征点
*/
/**
 * @brief 这个还没初始化结束，因此相比刚才，不进行地图点新的深度的换算，因为此时还有进行视觉惯性对齐（应该是初始化失败到这步来了）
 */
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}


// 对margin倒数第二帧进行处理
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count) // 如果地图点被最后一帧看到，由于滑窗，他的起始帧减1
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame; // 倒数第二帧在这个地图点对应KF vector的idx
            if (it->endFrame() < frame_count - 1) // 如果该地图点不能被倒数第二帧看到，那没什么好做的
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);// 能被倒数第二帧看到，erase掉这个索引
            //剔除观测到路标点的图像帧中即将被边缘化的图像帧（即WINDOW_SIZE-1）
            //相当于直接丢掉了被边缘化图像帧的观测

            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}



//计算frame_count-1帧 frame_count-2帧的其中一个特征点的视差
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    //其实算的一直是倒数第二三帧的视差
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    // 这一步与上一步重复，不知道必要性在哪里，目前没有必须性
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;//视差
 // 归一化相机坐标系的坐标差
    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;

    //todo 增加了利用rgbd来判断位移，但没想好判断阈值  
    // double distance;
    // double depth_i = frame_i.pointRight.x();
    // double depth_j = frame_j.pointRight.x();
    // distance = sqrt(depth_j - depth_i);
    // return distance;
    //todo
}






//todo  use for odom optimize 
//todo 1 for optimize
bool slerp_insert(Eigen::Quaterniond &Q, double t, nav_msgs::Odometry::ConstPtr
    &front_data, nav_msgs::Odometry::ConstPtr &back_data)
{
    Eigen::Quaterniond front_Q;
    front_Q.x() = front_data->pose.pose.orientation.x;
    front_Q.y() = front_data->pose.pose.orientation.y;
    front_Q.z() = front_data->pose.pose.orientation.z;
    front_Q.w() = front_data->pose.pose.orientation.w;
    front_Q.normalize();

    Eigen::Quaterniond back_Q;
    back_Q.x() = back_data->pose.pose.orientation.x;
    back_Q.y() = back_data->pose.pose.orientation.y;
    back_Q.z() = back_data->pose.pose.orientation.z;
    back_Q.w() = back_data->pose.pose.orientation.w;
    back_Q.normalize();

    double cos_theta = front_Q.x() * back_Q.x() + front_Q.y() * back_Q.y() + front_Q.z() * back_Q.z() + front_Q.w() * back_Q.w();

    if(cos_theta < 0)
    {
        back_Q.x() = -back_Q.x();
        back_Q.y() = -back_Q.y();
        back_Q.z() = -back_Q.z();
        back_Q.w() = -back_Q.w();

        cos_theta = front_Q.x() * back_Q.x() + front_Q.y() * back_Q.y() + front_Q.z() * back_Q.z() + front_Q.w() * back_Q.w();
    }

    double theta = acos(cos_theta);

    double sin_1_t_theta = sin((1-t) * theta);
    double sin_t_theta = sin(t * theta);

    if(sin(theta) == 0 || abs(front_data->twist.twist.angular.z) <= 0.05 || abs(back_data->twist.twist.angular.z) <= 0.05)//todo 
    {
        return false;
    }
    else
    {
        front_Q.x() = sin_1_t_theta * front_Q.x() / sin(theta);
        front_Q.y() = sin_1_t_theta * front_Q.y() / sin(theta);
        front_Q.z() = sin_1_t_theta * front_Q.z() / sin(theta);
        front_Q.w() = sin_1_t_theta * front_Q.w() / sin(theta);

        back_Q.x() = sin_t_theta * back_Q.x() / sin(theta);
        back_Q.y() = sin_t_theta * back_Q.y() / sin(theta);
        back_Q.z() = sin_t_theta * back_Q.z() / sin(theta);
        back_Q.w() = sin_t_theta * back_Q.w() / sin(theta);

        Q.x() = front_Q.x() + back_Q.x();
        Q.y() = front_Q.y() + back_Q.y();   
        // Q.x() = 0;
        // Q.y() = 0;

        Q.z() = front_Q.z() + back_Q.z();
        Q.w() = front_Q.w() + back_Q.w();
        Q.normalize();
        return true;
    }
}

void FeatureManager::linear_insert(Eigen::Quaterniond &Qodom, Eigen::Vector3d& Podom,const double sync_time,nav_msgs::Odometry::ConstPtr
    &front_data, nav_msgs::Odometry::ConstPtr &back_data) {
//y = (x1 - x)/(x1 - x0)y0 +(x - x0)/(x1 - x0)y1
    double front_scale = (back_data->header.stamp.toSec() - sync_time) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    double back_scale = (sync_time - front_data->header.stamp.toSec()) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    //线性插值位置
    //只插值x,y和旋转的z,w
    Podom << front_data->pose.pose.position.x * front_scale + back_data->pose.pose.position.x * back_scale,
        front_data->pose.pose.position.y* front_scale + back_data->pose.pose.position.y * back_scale,
        front_data->pose.pose.position.z* front_scale + back_data->pose.pose.position.z * back_scale;
        // 0;

    // if(slerp_insert(Qodom, back_scale, front_data, back_data))//todo 自己写的实现
    // {
    //     cout << "slerp_insert" << endl;
    // }
    // else
    // {
    //     //线性插值旋转
        // Qodom.x() = front_data->pose.pose.orientation.x * front_scale + back_data->pose.pose.orientation.x * back_scale;
        // Qodom.y() = front_data->pose.pose.orientation.y * front_scale + back_data->pose.pose.orientation.y * back_scale;
        // // Qodom.x() = 0.0;
        // // Qodom.y() = 0.0;
        // Qodom.z() = front_data->pose.pose.orientation.z * front_scale + back_data->pose.pose.orientation.z * back_scale;
        // Qodom.w() = front_data->pose.pose.orientation.w * front_scale + back_data->pose.pose.orientation.w * back_scale;
        // Qodom.normalize();
    // }


    //todo 函数实现
    Eigen::Quaterniond front_Q;
    front_Q.x() = front_data->pose.pose.orientation.x;
    front_Q.y() = front_data->pose.pose.orientation.y;
    front_Q.z() = front_data->pose.pose.orientation.z;
    front_Q.w() = front_data->pose.pose.orientation.w;
    front_Q.normalize();

    Eigen::Quaterniond back_Q;
    back_Q.x() = back_data->pose.pose.orientation.x;
    back_Q.y() = back_data->pose.pose.orientation.y;
    back_Q.z() = back_data->pose.pose.orientation.z;
    back_Q.w() = back_data->pose.pose.orientation.w;
    back_Q.normalize();

    Qodom = front_Q.slerp(back_scale,back_Q);

}

void FeatureManager::getOdomData(Eigen::Quaterniond &Q,Eigen::Vector3d &P, nav_msgs::Odometry::ConstPtr &odomData) {
    //取旋转
    
    Q.x() = odomData->pose.pose.orientation.x;//todo ?
    Q.y() = odomData->pose.pose.orientation.y;//todo ?
    // Q.x() = 0;
    // Q.y() = 0;
    Q.z() = odomData->pose.pose.orientation.z;
    Q.w() = odomData->pose.pose.orientation.w;
    Q.normalize();


    //取位移
    P << odomData->pose.pose.position.x, odomData->pose.pose.position.y, odomData->pose.pose.position.z;
    // P << odomData->pose.pose.position.x, odomData->pose.pose.position.y, 0;
}

//Rcam == Rwb , Rwc == Rbc
/*
void FeatureManager::transformOdomTCam(Eigen::Matrix3d& Rcam, Eigen::Vector3d& Pcam) {
    Rcam *= Rwc;
    Pcam += Twc;
}
*/

bool FeatureManager::getPoseByWheelOdom(Eigen::Vector3d& Pcam,Eigen::Matrix3d& Rcam,const double curTime) {
    
    if (wheel_odom_buf.empty()) {
        printf("odom data has not push into buf yet! \n");
        return false;
    }

    nav_msgs::Odometry::ConstPtr odomFront;
    nav_msgs::Odometry::ConstPtr odomBack;
    Eigen::Quaterniond odomQ;
    Eigen::Vector3d odomP;


    //wheel到cam旋转矩阵 R_bc
    // Eigen::Matrix3d wheel2Cam;
    // Eigen::Matrix<double,1,3> Ttemp;
    // wheel2Cam << 0, 0, 1, -1, 0, 0, 0, -1, 0;



    if (curTime < wheel_odom_buf.front()->header.stamp.toSec() || wheel_odom_buf.back()->header.stamp.toSec() < curTime) {
        printf("curtime odom data not push into wheel buf! \n ");
        return false;
    }
    else{
        //计时间
        TicToc use_time;

        m_wheel_odom.lock();
        while (wheel_odom_buf.front()->header.stamp.toSec() <= curTime) {
            if (wheel_odom_buf.front()->header.stamp.toSec() == curTime) {
                getOdomData(odomQ,odomP, wheel_odom_buf.front());
                //printf("get odom curtime data: q %f %f %f %f \n",odomQ.x(), odomQ.y(), odomQ.z(), odomQ.w());
                cout << odomP << endl;
                //相机坐标系在world下的位置
                odomQ.normalize();
                Rcam = odomQ.toRotationMatrix();
                Pcam = odomP;

                //Ttemp=Twc.transpose()* Rcam.transpose();
                //Pcam = odomP + Rcam*Toc; 
                //相机坐标系的旋转
                //Rcam *= Roc;
                //将base_link转换到camera

                //transformOdomTCam(Rcam,Pcam);

                //Rcam = odomQ.toRotationMatrix();
                wheel_odom_buf.pop();
                m_wheel_odom.unlock();
                return true;
            }
            odomFront = wheel_odom_buf.front();
            wheel_odom_buf.pop();
        }
        odomBack = wheel_odom_buf.front();//front  < cur < back
        m_wheel_odom.unlock();
        linear_insert(odomQ, odomP, curTime, odomFront, odomBack);
        //printf("get odom curtime data: q %f %f %f %f \n", odomQ.x(), odomQ.y(), odomQ.z(), odomQ.w());
        //cout << odomP << endl;
        //相机坐标系在world下的位置
        Rcam = odomQ.toRotationMatrix();//R_wo
        //Ttemp = Twc.transpose() * Rcam;
        //Ttemp = Twc.transpose() * Rcam.transpose();
        Pcam = odomP;
        //Pcam = odomP + Rcam*Toc;
        //Pcam = odomP + Rcam * Twc;
        //相机坐标系的旋转
        //Rcam *= Roc;//R_wc

        //Rcam = odomQ.toRotationMatrix();
        //耗时
        //printf("get pose data from wheel odom cost %f \n",use_time.toc());
        return true;
    }
}

bool FeatureManager::getRelativePoseByWheel(Eigen::Matrix3d& Rcam, Eigen::Vector3d& Pcam,
    const double prevTime, const double curTime)
{
    

    if (wheel_odom_buf.empty()) {
        printf("wheel_odom_buf is empty! \n");
        return false;
    }

    nav_msgs::Odometry::ConstPtr tempPrevOdom_front;
    nav_msgs::Odometry::ConstPtr tempPrevOdom_back;
    nav_msgs::Odometry::ConstPtr tempCurOdom_front;
    nav_msgs::Odometry::ConstPtr tempCurOdom_back;
    bool curTimeodomFlag = false;

    
    //需要获得的数据
    Eigen::Quaterniond prevTimeQ, curTimeQ;
    Eigen::Vector3d prevTimeP, curTimeP;

    //大于等于，则有curTime的odom数据，curTime的odom数据，可以被prevTime拿到
    //对于队列，开和闭的在于是否取完pop()
    if (curTime <= wheel_odom_buf.back()->header.stamp.toSec()) {
        
        
        if (wheel_odom_buf.front()->header.stamp.toSec() <= prevTime) {
            //有完整数据则加锁
            m_wheel_odom.lock();
            
            //使用at()需要进行边界检查，从0开始
            //如何保证函数使用不越界
            //如何索引
            //索引,使用简单队列就可以了
            //如果相等，则不用插值
            if (wheel_odom_buf.front()->header.stamp.toSec() == prevTime) {
                //直接赋值
                
                getOdomData(prevTimeQ, prevTimeP, wheel_odom_buf.front());
                wheel_odom_buf.pop();
            }
            else {
                while (wheel_odom_buf.front()->header.stamp.toSec() < prevTime) {
                    
                    //printf("pop messages before prevTime in wheel_odom_buf!\n");
                    //取prevtime的插值前数据
                    tempPrevOdom_front = wheel_odom_buf.front();
                    wheel_odom_buf.pop();
                }
                
                //prevtime取数据后，pop好一点，防止odom发布过慢导致的prevtime和curtime取到同一时间数据
                tempPrevOdom_back = wheel_odom_buf.front();
                //进行插值
                linear_insert(prevTimeQ, prevTimeP, prevTime, tempPrevOdom_front, tempPrevOdom_back);//pre之间的数据
            }

            // while (wheel_odom_buf.front()->header.stamp.toSec() <= curTime) //todo  kitti时得注释掉
            {
                
                if (wheel_odom_buf.front()->header.stamp.toSec() == curTime) {
                    //直接赋值
                    getOdomData(curTimeQ, curTimeP, wheel_odom_buf.front());
                    curTimeodomFlag = true;
                    // break;                                               //todo  kitti时得注释掉
                }
                else {
                    tempCurOdom_front = wheel_odom_buf.front();
                    wheel_odom_buf.pop();
                }
            }
            
            //取第一个大于等于curtime的数据
            tempCurOdom_back = wheel_odom_buf.front();
            if (!curTimeodomFlag) {
                
                linear_insert(curTimeQ, curTimeP, curTime, tempCurOdom_front, tempCurOdom_back);
            }
            //不pop，则后一个prevtime可以取到前一个时间段中curtime的数据
            //wheel_odom_buf.pop(); 
            //数据操作完毕，解锁
            m_wheel_odom.unlock();

            
            //插值获得prevTime位姿中的旋转和平移
            Eigen::Matrix3d preR = prevTimeQ.toRotationMatrix();
            Eigen::Matrix3d curR = curTimeQ.toRotationMatrix();

            //计算相对旋转
            Rcam = preR.transpose() * curR;

            //计算相对位移
            // Pcam << preR.transpose() * (curTimeP - prevTimeP);//todo  8.11 改为平移增量转换到i系
            Pcam <<   (preR * curR.transpose() * curTimeP - prevTimeP);

            //输出矩阵
            //printf("preTime is %f,curTime is %f,R is \n", prevTime, curTime);
            //std::cout << Rcam << std::endl;
            /*
            for (auto p = begin(Rcam); p != end(Rcam); ++p) {   //用begin()和end()来替代手工的控制变量
                for (auto q = begin(*p); q != end(*p); ++q) {
                    cout << *q << ' ';
                }
                cout << endl;
            }
            */

            //printf("Pcam is %f,%f,%f \n", Pcam[0], Pcam[1], Pcam[2]);
            return true;
        }
        else {
            //printf("prevTime odom data has not in buf!\n");
            return false;
        }
    }
    else {
        //printf("curTime odom data has not push in buf yet!\n");
        return false;
    }

}
//todo


//todo 2 for horizon generator  11.23没继续写了
void linear_insert(Eigen::Quaterniond& Qodom, Eigen::Vector3d& Podom, Eigen::Vector3d& V, Eigen::Vector3d& a, Eigen::Vector3d& w, 
                     Eigen::Vector3d& Ba, Eigen::Vector3d& Bg,
                     const double sync_time, nav_msgs::Odometry::ConstPtr& front_data, nav_msgs::Odometry::ConstPtr& back_data)
{
    //y = (x1 - x)/(x1 - x0)y0 +(x - x0)/(x1 - x0)y1
    double front_scale = (back_data->header.stamp.toSec() - sync_time) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    double back_scale = (sync_time - front_data->header.stamp.toSec()) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    //线性插值位置
    //只插值x,y和旋转的z,w
    Podom << front_data->pose.pose.position.x * front_scale + back_data->pose.pose.position.x * back_scale,
             front_data->pose.pose.position.y* front_scale + back_data->pose.pose.position.y * back_scale,
             front_data->pose.pose.position.z* front_scale + back_data->pose.pose.position.z * back_scale;
            // 0;

    V << front_data->twist.twist.linear.x * front_scale + back_data->twist.twist.linear.x * back_scale,
         front_data->twist.twist.linear.y * front_scale + back_data->twist.twist.linear.y * back_scale,
         front_data->twist.twist.linear.z * front_scale + back_data->twist.twist.linear.z * back_scale;


    // if(slerp_insert(Qodom, back_scale, front_data, back_data))//todo 自己写的实现
    // {
    //     cout << "slerp_insert" << endl;
    // }
    // else
    // {
    //     //线性插值旋转
        // Qodom.x() = front_data->pose.pose.orientation.x * front_scale + back_data->pose.pose.orientation.x * back_scale;
        // Qodom.y() = front_data->pose.pose.orientation.y * front_scale + back_data->pose.pose.orientation.y * back_scale;
        // // Qodom.x() = 0.0;
        // // Qodom.y() = 0.0;
        // Qodom.z() = front_data->pose.pose.orientation.z * front_scale + back_data->pose.pose.orientation.z * back_scale;
        // Qodom.w() = front_data->pose.pose.orientation.w * front_scale + back_data->pose.pose.orientation.w * back_scale;
        // Qodom.normalize();
    // }


    //todo 函数实现
    Eigen::Quaterniond front_Q;
    front_Q.x() = front_data->pose.pose.orientation.x;
    front_Q.y() = front_data->pose.pose.orientation.y;
    front_Q.z() = front_data->pose.pose.orientation.z;
    front_Q.w() = front_data->pose.pose.orientation.w;
    front_Q.normalize();

    Eigen::Quaterniond back_Q;
    back_Q.x() = back_data->pose.pose.orientation.x;
    back_Q.y() = back_data->pose.pose.orientation.y;
    back_Q.z() = back_data->pose.pose.orientation.z;
    back_Q.w() = back_data->pose.pose.orientation.w;
    back_Q.normalize();

    Qodom = front_Q.slerp(back_scale,back_Q);


}