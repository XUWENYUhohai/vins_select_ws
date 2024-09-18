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

#include "feature_tracker.h"

// return: true 内点， false 边缘点
bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;//! 原1 ， 可以改改看
    int img_x = cvRound(pt.x);//cvRound返回跟参数最接近的整数值，即四舍五入；
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

//todo 下面有了
// double distance(cv::Point2f pt1, cv::Point2f pt2)
// {
//     //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
//     double dx = pt1.x - pt2.x;
//     double dy = pt1.y - pt2.y;
//     return sqrt(dx * dx + dy * dy);
// }

/**
 * 删除集合中status为0的点
*/
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
/**
 * 删除集合中status为0的元素
*/
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
//todo
/**
 * 删除集合中status为0的元素
*/
void reduceVector(vector<double> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//todo
/**
 * 删除集合中status为0的元素
*/
void reduceVector(vector<float> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;

    depth_cam = 0;//todo
}


//TODO: 这里应该可以做动态检测
// 把追踪到的点进行标记
// 设置遮挡部分（鱼眼相机）
// 对检测到的特征点按追踪到的次数排序
// 在mask图像中将追踪到点的地方设置为0，否则为255，目的是为了下面做特征点检测的时候可以选择没有特征点的区域进行检测。
// 在同一区域内，追踪到次数最多的点会被保留，其他的点会被删除??
void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));
    
    //todo 输出图片
    // cv::Mat cur_img_tmp;
    // cv::cvtColor(cur_img, cur_img_tmp, cv::COLOR_GRAY2BGR);

    // prefer to keep features that are tracked for long time// 保存长时间跟踪到的特征点
    vector<pair<pair<int, pair<cv::Point2f, int>> , double>> cnt_pts_id;//todo
    

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])) , scores[i]));//todo
        
    }

// sort 对给定区间的所有元素进行排序，按照点的跟踪次数，从多到少进行排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<pair<int, pair<cv::Point2f, int>> , double> &a, const pair<pair<int, pair<cv::Point2f, int>> , double> &b)//todo
         {
            return a.first.first > b.first.first;//todo
         });

    // sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<pair<int, pair<cv::Point2f, int>> , double> &a, const pair<pair<int, pair<cv::Point2f, int>> , double> &b)//todo
    //     {
    //     return a.second > b.second;//todo
    //     });


    cur_pts.clear();
    ids.clear();
    track_cnt.clear();
    scores.clear();//todo

    // cout << "fprintf best_score" << endl;
    // if(prev_pts.size() > 0)
    //     if(last_score != NULL) fprintf(last_score, "%f\n", cnt_pts_id[cnt_pts_id.size() - 1].second);//todo 3.22   输出最小响应值
    // cout << "fprintf best_score" << endl;
    
    for (auto &it : cnt_pts_id)
    {
        //todo 3.16  掩摸
        // cv::circle(cur_img_tmp, it.first.second.first, 5, cv::Scalar(0,0,0), -1);
        
           //todo
        if (mask.at<uchar>(it.first.second.first) == 255) // at 获取像素点的灰度值或者RGB值，可以通过image.at<uchar>(i,j)的方式轻松获取。检测新建的mask在该点是否为255
        { //将跟踪到的点按照跟踪次数重新排列，并返回到forw_pts，ids，track_cnt
            cur_pts.push_back(it.first.second.first);
            ids.push_back(it.first.second.second);
            track_cnt.push_back(it.first.first);
            scores.push_back(it.second);

            /*在已跟踪到角点的位置上，将mask对应位置上设为0,
            意为在cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
            进行操作时在该点不再重复进行角点检测，这样可以使角点分布更加均匀
            TODO:讲动态的特征点,也设置为这个
            图片，点，半径，颜色为0表示在角点检测在该点不起作用,粗细（-1）表示填充*/
            cv::circle(mask, it.first.second.first, MIN_DIST, 0, -1);

            // cv::circle(mask, it.first.second.first, min_dist, 0, -1);

            //todo 3.16  输出图片
            // string score_text = to_string(it.second);
            // auto scoreColor = cv::Scalar(0, 255*it.second*10, 255*(1 - it.second*10));

            // cv::circle(cur_img_tmp, it.first.second.first, min_dist, scoreColor, 2);//todo 1.5

            
            // cv::circle(cur_img_tmp, it.first.second.first, 5, scoreColor, -1);
            // cv::putText(cur_img_tmp, score_text, it.first.second.first, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, scoreColor);
        }
        //todo
    }

    // if(prev_pts.size() > 0)
    //     if(new_last_score != NULL) fprintf(new_last_score, "%f\n", scores[scores.size() -1]);//todo 3.22   输出new最小响应值

      // TODO:这里应该输出一下mask  看看效果
        // cv::imshow ("mask", mask);      // 用cv::imshow显示图像
        // cv::imshow ("cur_img_tmp", cur_img_tmp);
        // cv::imwrite ("/home/xuwenyu/00/" + to_string(cur_time) + "_mask.png", cur_img_tmp);
        // cv::waitKey (0);                  // 暂停程序,等待一个按键输入
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}


// 对图片进行一系列操作，返回特征点featureFrame。
// 其中还包含了：图像处理、区域mask、检测特征点、计算像素速度等

// ***********************************
//  特征点追踪：
// calcOpticalFlowPyrLK-->反向追踪去除外点-->去除图像边缘的特征点-->通过setMask去除边缘畸变比较大的点-->
// goodFeaturesToTrack 光流跟不够的用新提取的corners角点来补,使其达到每帧最大的特征点数量--->
// 将当前帧的特征点(像素平面 u v)先去畸变，然后映射到归一化坐标系中-->
// 根据前一帧和当前帧的特征点来计算当前帧特征点的速度 vx,vy
// OpenCV 显示特征点与特征追踪的线--->
// 当前帧赋值到上一帧历史记录
// 将每一个特征点打包成featureFrame <归一化坐标的x,y,z 像素坐标u,v 点的速度vx,vy>//todo score lifetime(track_cnt)
// 
//  input : 某一时刻的左图，设置为cur_img, 如果有右图也可输入右图
//  return: featureFrame
// ***********************************

/**
 * 跟踪一帧图像，提取当前帧特征点
 * 1、用前一帧运动估计特征点在当前帧中的位置
 * 2、LK光流跟踪前一帧的特征点，正反向，删除跟丢的点；如果是双目，进行左右匹配，只删右目跟丢的特征点
 * 3、对于前后帧用LK光流跟踪到的匹配特征点，计算基础矩阵，用极线约束进一步剔除outlier点（代码注释掉了）
 * 4、如果特征点不够，剩余的用角点来凑；更新特征点跟踪次数
 * 5、计算特征点归一化相机平面坐标，并计算相对与前一帧移动速度
 * 6、保存当前帧特征点数据（归一化相机平面坐标，像素坐标，归一化相机平面移动速度）
 * 7、展示，左图特征点用颜色区分跟踪次数（红色少，蓝色多），画个箭头指向前一帧特征点位置，如果是双目，右图画个绿色点
*/
//todo 7
map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
//todo
    // std::vector<cv::KeyPoint> keypoint1;
//todo
    
    TicToc t_r;
    cur_time = _cur_time;

// printf("trackimage 1\n");

    //todo 添加了直方图均衡，效果优于origin，odom没加（8.27）
    cv::Mat img , img1;
    ///https://blog.51cto.com/xiaohaiwa/5380121   https://blog.csdn.net/weixin_45930877/article/details/119581282   
    if(EQUALIZE)
    {//这里使用来做图像处理的！！！
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));//直方图均衡
        clahe->apply(_img, img);
    }
    else
        img = _img;


    if(EQUALIZE)
    {//这里使用来做图像处理的！！！
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));//直方图均衡
        clahe->apply(_img1, img1);
    }
    else
        img1 = _img1;
    // todo


    // cur_img = _img;
    cur_img = img;//todo


    

    row = cur_img.rows;
    col = cur_img.cols;

    // cv::Mat rightImg = _img1;
    cv::Mat rightImg = img1;//todo

    cur_pts.clear();

    if (prev_pts.size() > 0)
    {
        
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)// 利用恒速模型对路标点坐标进行了预测
        {
            cur_pts = predict_pts;

            //https://blog.csdn.net/weixin_39068873/article/details/116258938
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)            // 特征点太少，金字塔调整为3层，再跟踪一次
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);//! 原3 
        // reverse check// 反向LK光流计算一次 //从后一帧图像，计算前一帧图像的points，进行额外筛选，提升鲁棒性
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, //! 原1
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)//! 看看能不能改改试试
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }


        // cur_pts_num = (double)cur_pts.size();//todo 3.22输出准确率

        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
            {
                status[i] = 0;
                
                //todo
                // int p_u, p_v;
                // p_u = (int) cur_pts[i].x;
                // p_v = (int) cur_pts[i].y;
                // float grey = cur_img.at<uchar>(p_u,p_v);
                // if (status[i] && grey > 250){
                //     status[i] = 0;
                ////  cout<<grey<<endl;
                // }
            }
                

        


        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(scores,status);//todo
        reduceVector(err,status);//todo 1.6

        // if(true_match_pro != NULL) fprintf(true_match_pro, "%f\n", (double)cur_pts.size() / cur_pts_num);//todo 3.22输出准确率
       
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());


        //todo 1.6
        
        // double avg_err = 0;
        // if(err.size() != 0)
        // {
        //     sort(err.begin(), err.end());
        //     for (auto i : err)
        //     {
        //         avg_err += i;
        //     }

        //     // avg_err /= err.size();
        //     avg_err = err.back() / avg_err;
        //     ROS_WARN_STREAM(" err.front() = " << err.front());
        // }


        //todo 1.5
        // if(cur_pts.size() != 0)
        // {
        //     // min_dist = sqrt((col * row) / (MAX_CNT - static_cast<int>(cur_pts.size())) / M_PI / sqrt(avg_err));
        //     min_dist = sqrt((col * row) / (MAX_CNT - static_cast<int>(cur_pts.size())) / M_PI / sqrt(2));//M_PI //todo max_cnt 100 
        //     // min_dist = sqrt((col * row) / (MAX_CNT - static_cast<int>(cur_pts.size())) / M_PI / 2);
        // }
        // else
        // {
        //     min_dist = MIN_DIST;
        // }
        //todo 1.5

        // cout << "fprintf min_dist" << endl;
        // if(min_dis != NULL) fprintf(min_dis, "%d\n", min_dist);//todo 3.22   输出min_dist
        // if(new_point_num != NULL) fprintf(new_point_num, "%d\n", (MAX_CNT - static_cast<int>(cur_pts.size())));//todo 3.22   输出MAX_CNT - cur_pts.size()
        // cout << "fprintf min_dist" << endl;

    }

    for (auto &n : track_cnt)  // 还在的特征点跟踪次数加1
        n++;

    // cout << " 6 " << endl;

    if (1)
    {   
        //todo 3.16  E矩阵
        // if(prev_pts.size() > 0 && cur_pts.size() > 0)
        // {
        //     cv::cvtColor(cur_img, cur_img_tmp, cv::COLOR_GRAY2BGR);
        //     cv::cvtColor(prev_img, prev_img_tmp, cv::COLOR_GRAY2BGR);
        //     cv::Mat tmp;
        //     cv::vconcat(prev_img, cur_img, tmp);
        //     cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2BGR);

        //     for (size_t i = 0; i < prev_pts.size(); i++)
        //     {
        //         cv::circle(tmp, prev_pts[i], 5, cv::Scalar(255,0,0), -1);
        //         cv::circle(tmp, cv::Point2f(cur_pts[i].x, cur_pts[i].y + cur_img_tmp.rows), 5, cv::Scalar(0,0,255), -1);  
        //         cv::line(tmp, prev_pts[i], cv::Point2f(cur_pts[i].x, cur_pts[i].y + cur_img_tmp.rows), cv::Scalar(0,255,0), 1, 8, 0);
        //     }
            
        // //     cv::imshow("prev_img_tmp", tmp);
        // //     cv::waitKey();
        //     cv::imwrite ("/home/xuwenyu/00/" + to_string(cur_time) + ".png", tmp);
        // }


        // 对于前后帧用LK光流跟踪到的匹配特征点，计算基础矩阵，进一步剔除outlier点
        rejectWithF();//!!!!!!!!!!!!!!!!   todo
        // printf("trackimage 2\n");
        ROS_DEBUG("set mask begins");


        //todo 3.16  E矩阵
        // if(prev_pts.size() > 0 && cur_pts.size() > 0)
        // {
        //     cv::cvtColor(cur_img, cur_img_tmp, cv::COLOR_GRAY2BGR);
        //     cv::cvtColor(prev_img, prev_img_tmp, cv::COLOR_GRAY2BGR);
        //     cv::Mat tmp;
        //     cv::vconcat(prev_img, cur_img, tmp);
        //     cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2BGR);

        //     for (size_t i = 0; i < prev_pts.size(); i++)
        //     {
        //         cv::circle(tmp, prev_pts[i], 5, cv::Scalar(255,0,0), -1);
        //         cv::circle(tmp, cv::Point2f(cur_pts[i].x, cur_pts[i].y + cur_img_tmp.rows), 5, cv::Scalar(0,0,255), -1);  
        //         cv::line(tmp, prev_pts[i], cv::Point2f(cur_pts[i].x, cur_pts[i].y + cur_img_tmp.rows), cv::Scalar(0,255,0), 1, 8, 0);
        //     }
            
        //     // cv::imshow("prev_img_tmp", tmp);
        //     // cv::waitKey();
        //     cv::imwrite ("/home/xuwenyu/00/" + to_string(cur_time) + ".png", tmp);
        // }


        TicToc t_m;
        // 特征点画个圈存mask图，同时特征点集合按跟踪次数从大到小重排序
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());


        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;

        
        // rejectWithF();//todo 8.27改，原本不能使用,好像还是clahe效果提升明显，这个一般般吧（11.11）


            /* goodFeaturesToTrack
            _image：8位或32位浮点型输入图像，单通道
            _corners：保存检测出的角点
            maxCorners：角点数目最大值，如果实际检测的角点超过此值，则只返回前maxCorners个强角点
            qualityLevel：角点的品质因子
            minDistance：对于初选出的角点而言，如果在其周围minDistance范围内存在其他更强角点，则将此角点删除
            _mask：指定感兴趣区，如不需在整幅图上寻找角点，则用此参数指定ROI
            blockSize：计算协方差矩阵时的窗口大小
            useHarrisDetector：指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
            harrisK：Harris角点检测需要的k值 */

            // cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);

            //todo 1.5   https://blog.csdn.net/Sunshine_in_Moon/article/details/45418651  
            cvmodified::goodFeaturesToTrack(cur_img,n_pts,score0,MAX_CNT - cur_pts.size(),0.01,MIN_DIST ,mask,3,false,0.04);
            // cvmodified::goodFeaturesToTrack(cur_img,n_pts,score0,MAX_CNT - cur_pts.size(),0.01,min_dist,mask,3,false,0.04);//TODO 1.5
            ROS_WARN_STREAM("min_dist = " << min_dist);

            //todo 3.16  输出图片  掩摸
            // cv::Mat cur_img_tmp;
            // cv::cvtColor(cur_img,cur_img_tmp, cv::COLOR_GRAY2BGR);
            // for (auto it : n_pts)
            // {
            //     cv::circle(cur_img_tmp, it, 5, cv::Scalar(255,0,0), -1);  
            // }
            // cv::imshow("cur_img", cur_img_tmp);
            // cv::waitKey (0);
            // cv::imwrite ("/home/xuwenyu/00/" + to_string(cur_time) + ".png", cur_img_tmp);


//todo 9.1 orb能够使用但效果不如goodFeaturesToTrack , 可能是goodFeaturesToTrack函数设置了最小距离和角点质量等因素，或者就是本身特征点差距
            // // cv::Ptr<cv::ORB> orb = cv::ORB::create(n_max_cnt);
            // cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();//TODO 1.16
            // // // cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create(n_max_cnt);
            // // // ///cv::FAST(cur_img,keypoint1,30);
            
            
            // // // keypoint1.resize(n_max_cnt);
            // // // // orb->detectAndCompute(cur_img,cv::Mat(),keypoint1,des1);
            // // // orb->detect(cur_img,keypoint1);
            // // orb->detect(cur_img,keypoint1,mask);
            // akaze->detectAndCompute(cur_img, cv::Mat(), keypoint1, des1);//TODO 1.16
            
            // if(!keypoint1.empty())
            // {
            //     keypoint1.resize(300);
            //     cv::KeyPoint::convert(keypoint1, n_pts);
            //     keypoint1.clear();
            //     // cout << "orb num:" <<n_pts.size() << endl;
		    // }
            // else 
            // {
            //     n_pts.clear();
		    // }
            

//todo
        }
        else
            n_pts.clear();

       

        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);

            // scores.push_back(0);//todo        test origincv::goodFeaturesToTrack
        }
        
        //todo
        for (auto & score : score0)
        {
            scores.push_back(score);
        }
        
        //printf("feature cnt after add %d\n", (int)ids.size());
    }
//cur_pts点为图像坐标,投影到相机模型的归一化平面
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);//去畸变
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);//计算在归一化相机坐标系下的速度

    

    if(!_img1.empty() && stereo_cam && !depth_cam)//todo 双目
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right

            /*光流跟踪是在左右两幅图像之间进行cur left ---- cur right
            prevImg	第一幅8位输入图像 或 由buildOpticalFlowPyramid()构造的金字塔。
            nextImg	第二幅与preImg大小和类型相同的输入图像或金字塔。
            prevPts	光流法需要找到的二维点的vector。点坐标必须是单精度浮点数。
            nextPts	可以作为输入，也可以作为输出。包含输入特征在第二幅图像中计算出的新位置的二维点（单精度浮点坐标）的输出vector。当使用OPTFLOW_USE_INITIAL_FLOW 标志时，nextPts的vector必须与input的大小相同。
            status	输出状态vector(类型：unsigned chars)。如果找到了对应特征的流，则将向量的每个元素设置为1；否则，置0。
            err	误差输出vector。vector的每个元素被设置为对应特征的误差，可以在flags参数中设置误差度量的类型；如果没有找到流，则未定义误差（使用status参数来查找此类情况）。
            winSize	每级金字塔的搜索窗口大小。
            maxLevel	基于最大金字塔层次数。如果设置为0，则不使用金字塔（单级）；如果设置为1，则使用两个级别，等等。如果金字塔被传递到input，那么算法使用的级别与金字塔同级别但不大于MaxLevel。
            criteria	指定迭代搜索算法的终止准则（在指定的最大迭代次数标准值（criteria.maxCount）之后，或者当搜索窗口移动小于criteria.epsilon。）
            flags 操作标志，可选参数：
            OPTFLOW_USE_INITIAL_FLOW：使用初始估计，存储在nextPts中；如果未设置标志，则将prevPts复制到nextPts并被视为初始估计。
            OPTFLOW_LK_GET_MIN_EIGENVALS：使用最小本征值作为误差度量（见minEigThreshold描述）；如果未设置标志，则将原始周围的一小部分和移动的点之间的 L1 距离除以窗口中的像素数，作为误差度量。
            minEigThreshold	
            算法所计算的光流方程的2x2标准矩阵的最小本征值（该矩阵称为[Bouguet00]中的空间梯度矩阵）÷ 窗口中的像素数。如果该值小于MinEigThreshold，则过滤掉相应的特征，相应的流也不进行处理。因此可以移除不好的点并提升性能。 */
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);//! 原3 
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);//! 原3 
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }


            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts // 只删右边跟丢的特征点，还是左边也删（to be checked）
            
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            reduceVector(scores,status);//todo
            
            // rejectWithE_stereo();//todo 1.5

            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }//todo
    else if (!_img1.empty() && depth_cam)//todo      depth
    {        
        // printf("trackimage 3\n");
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("trackimage 4\n");
            ids_right = ids;
            cur_right_pts = cur_pts;
            //cur_un_right_pts = cur_un_pts;//todo
            size_t N = cur_pts.size();
            vector<uchar> status(N,0);
            cur_un_right_pts.resize(N);//todo

        //printf("trackimage 4.1\n");
            for(size_t i = 0; i < N; i++)
            {
                int p_u, p_v;
                p_u = (int) cur_pts[i].x;
                p_v = (int) cur_pts[i].y;
                //printf("trackimage 4.2\n");
                //float d = 1.15 * rightImg.ptr<unsigned short >(p_v)[p_u]; //old 参数
                float d = rightImg.ptr<unsigned short >(p_v)[p_u];
                //cout <<d<<endl;
                if(d > 170 && d < 4000)//! 这个距离应该要改(3.30已改)
                {   //printf("trackimage 4.3\n");
                    //cur_un_right_pts[i].x = cur_un_pts[i].x - 100.0 / d;
                    cur_un_right_pts[i].x = d / 1000;//! mm -> m???? 
                    //printf("trackimage 4.3.1\n");
                    cur_un_right_pts[i].y = d / 1000;
                    //printf("trackimage 4.3.1.2\n");
                    //cout <<cur_un_right_pts[i].x<<endl;
                    status[i] = 1;
                    //printf("trackimage 4.3.1\n");
                }
                else//todo 4.4
                {
                    cur_un_right_pts[i].x = 0;
                    cur_un_right_pts[i].y = 0;
                    status[i] = 1;
                }
                //printf("trackimage 4.4\n");
            }
            //printf("trackimage 5\n");
            reduceVector(cur_right_pts, status);
            reduceVector(cur_un_right_pts, status);
            reduceVector(ids_right, status);


            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            reduceVector(scores,status);//todo
//todo 4.4

            // right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
            //printf("trackimage 6\n");
        }
        //prev_un_right_pts_map = cur_un_right_pts_map;
        //printf("track right=depth finsh\n");
    }
//todo




    if(SHOW_TRACK)    // OpenCV 显示特征点与特征追踪的线
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    //printf("draw track\n");
  // 当前帧赋值到上一帧历史记录
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

//todo  11.8
    // find the largest score to use as a normalizer to convert to "probability"
    auto it = std::max_element(scores.begin(), scores.end());
    double eta = (it != scores.end()) ? *it : 1.0;
//todo
    
//（即特征点编号、相机编号（0表示左相机，1表示右相机）、每个特征点参数（归一化相机坐标系坐标、图像坐标（矫正后）、归一化相机坐标系下路标点速度）//todo score(double) lifttime(int)
    map<int, vector<pair<int, Eigen::Matrix<double, 9, 1>>>> featureFrame;//todo 7
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        double pro = scores[i] / eta;//todo 11.7


        Eigen::Matrix<double, 9, 1> xyz_uv_velocity;//todo 7
        // xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, scores[i], (double)track_cnt[i];//todo
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, pro, (double)track_cnt[i];//todo 11.7
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

   

    if (!_img1.empty() && (stereo_cam || depth_cam))//todo
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = 0;         //todo  rgbd:0   stereo: right_pts_velocity[i].x
            velocity_y = 0;         //todo  rgbd:0   stereo: right_pts_velocity[i].y

            double pro = scores[i] / eta;//todo 11.7


            Eigen::Matrix<double, 9, 1> xyz_uv_velocity;//todo 7
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, pro, (double)track_cnt[i];//todo 11.7
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}






//使用F矩阵进行拒绝 删除一些点 涉及到FM_RANSAC,拒绝的是一些光流跟踪错误的点,
// TODO: 这里可能涉及到动态检测，   FOCAL_LENGTH应该要改
//notused
// 通过求解F矩阵过程中的RANSAC 来去除outlier
/**
 * 对于前后帧用LK光流跟踪到的匹配特征点，计算基础矩阵，进一步剔除outlier点
*/
void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {      // 特征点先转到归一化相机平面下，畸变校正，再转回来
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);

            //归一化的投影平面  u = fx*x(distorted)+cx
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;//! best  按照原fxfycxcy来
            // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + 607;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + 185;

            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);

            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + 607;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + 185;

            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        // cout << "col : "   <<  col <<  "  row : " << row  << endl;

        vector<uchar> status;
           // 两帧特征点匹配，算一个最佳的基础矩阵，剔除掉在该基础矩阵变换下匹配较差的匹配点对
        //todo  https://blog.csdn.net/wangerwei86/article/details/7681946
        int size = (int)cur_pts.size();
        cv::Mat p1(size,2,CV_32F) , p2(size,2,CV_32F);
        for (size_t i = 0; i < size; i++)
        {
            // p1.at<float>(i,0) = cur_pts[i].x;
            // p1.at<float>(i,1) = cur_pts[i].y;

            // p2.at<float>(i,0) = prev_pts[i].x;
            // p2.at<float>(i,1) = prev_pts[i].y;

            p1.at<float>(i,0) = un_cur_pts[i].x;
            p1.at<float>(i,1) = un_cur_pts[i].y;

            p2.at<float>(i,0) = un_prev_pts[i].x;
            p2.at<float>(i,1) = un_prev_pts[i].y;
        }
        
        //todo  rh0 < lmeds > 8points   F_THRESHOLD: 0.5 or 1.0   https://blog.csdn.net/u011867581/article/details/43818183
        //todo cv::FM_LMEDS和cv::LMEDS好像差不多
        // cv::findFundamentalMat(p2, p1, cv::LMEDS, F_THRESHOLD, 0.5, status);
        // cv::findFundamentalMat(p2, p1, cv::FM_LMEDS, F_THRESHOLD, 0.5, status);

        //https://blog.csdn.net/u012058778/article/details/90764430
        cv::Mat K = (cv::Mat_<double>(3,3) << 718.856 , 0 , 607.1928 , 0 , 718.856 , 185.2157 , 0 , 0 , 1);
        //https://blog.csdn.net/warningm_dm/article/details/111589774
        //todo rh0 > 8points 感觉上RH0和LMEDS差不多，各有优势在rpe和ape方面    F_THRESHOLD(LMEDS , RH0没用这个参数): 1.0和0.5差不多（rpe和ape各有优势）
        //TODO E比F的rpe更小  

        // !!!!!!!!  best(11.12)  E  cv::FM_LMEDS   0.99   0.5或1.0
        cv::findEssentialMat(p2, p1, K, cv::FM_LMEDS, 0.99 ,F_THRESHOLD, status);//! 0.5   
        // cv::findEssentialMat(p2, p1, K, cv::LMEDS, 0.99 ,F_THRESHOLD, status);//! 0.5   

        // cv::findEssentialMat(p2, p1, K, cv::FM_8POINT, 0.99 ,F_THRESHOLD, status);//! 0.5
        // cv::findEssentialMat(p2, p1, K, cv::LMEDS, 0.5 ,F_THRESHOLD, status);
        

        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(scores,status);//todo


        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}


//todo 1.5
void FeatureTracker::rejectWithE_stereo()
{
        if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_right_pts(cur_right_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {      // 特征点先转到归一化相机平面下，畸变校正，再转回来
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            //归一化的投影平面  u = fx*x(distorted)+cx
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;//! best  按照原fxfycxcy来
            // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + 607;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + 185;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[1]->liftProjective(Eigen::Vector2d(cur_right_pts[i].x, cur_right_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + 607;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + 185;
            un_right_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        // cout << "col : "   <<  col <<  "  row : " << row  << endl;

        vector<uchar> status;
           // 两帧特征点匹配，算一个最佳的基础矩阵，剔除掉在该基础矩阵变换下匹配较差的匹配点对
        //todo  https://blog.csdn.net/wangerwei86/article/details/7681946
        int size = (int)cur_pts.size();
        cv::Mat p1(size,2,CV_32F) , p2(size,2,CV_32F);
        for (size_t i = 0; i < size; i++)
        {
            // p1.at<float>(i,0) = cur_pts[i].x;
            // p1.at<float>(i,1) = cur_pts[i].y;

            // p2.at<float>(i,0) = prev_pts[i].x;
            // p2.at<float>(i,1) = prev_pts[i].y;

            p1.at<float>(i,0) = un_cur_pts[i].x;
            p1.at<float>(i,1) = un_cur_pts[i].y;

            p2.at<float>(i,0) = un_right_pts[i].x;
            p2.at<float>(i,1) = un_right_pts[i].y;
        }
        
        //todo  rh0 < lmeds > 8points   F_THRESHOLD: 0.5 or 1.0   https://blog.csdn.net/u011867581/article/details/43818183
        //todo cv::FM_LMEDS和cv::LMEDS好像差不多
        // cv::findFundamentalMat(p2, p1, cv::LMEDS, F_THRESHOLD, 0.5, status);
        // cv::findFundamentalMat(p2, p1, cv::FM_LMEDS, F_THRESHOLD, 0.5, status);

        //https://blog.csdn.net/u012058778/article/details/90764430
        cv::Mat K = (cv::Mat_<double>(3,3) << 718.856 , 0 , 607.1928 , 0 , 718.856 , 185.2157 , 0 , 0 , 1);
        //https://blog.csdn.net/warningm_dm/article/details/111589774
        //todo rh0 > 8points 感觉上RH0和LMEDS差不多，各有优势在rpe和ape方面    F_THRESHOLD(LMEDS , RH0没用这个参数): 1.0和0.5差不多（rpe和ape各有优势）
        //TODO E比F的rpe更小  

        // !!!!!!!!  best(11.12)  E  cv::FM_LMEDS   0.99   0.5或1.0
        // cv::findEssentialMat(p1, p2, K, cv::FM_LMEDS, 0.99 ,F_THRESHOLD, status);//! 0.5   
        cv::findEssentialMat(p1, p2, K, cv::RHO, 0.99 ,F_THRESHOLD, status);//! 0.5   

        // cv::findEssentialMat(p2, p1, K, cv::FM_8POINT, 0.99 ,F_THRESHOLD, status);//! 0.5
        // cv::findEssentialMat(p2, p1, K, cv::LMEDS, 0.5 ,F_THRESHOLD, status);
        

        int size_a = cur_pts.size();
        reduceVector(cur_right_pts, status);
        reduceVector(ids_right, status);
        // only keep left-right pts // 只删右边跟丢的特征点，还是左边也删（to be checked）
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(cur_un_pts, status);
        reduceVector(pts_velocity, status);
        reduceVector(scores,status);//todo


        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}




void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file,const int depth)//todo
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;

    //todo
    if(depth)
        depth_cam = 1;
    

    //todo

}

/**
 * 对当前帧图像进行畸变校正，展示
 * 
//显示 去畸变和归一化坐标映射的效果
notused
*/
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    // 计算每个像素点校正之后的像素坐标，可能会变成负数或者超出图像边界，所以扩大图像
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

// 将像素座标系下的座标，转换为归一化相机座标系下的座标 即un_pts为归一化相机座标系下的座标。

// 对于像素平面的二维点 去畸变，映射成三维坐标，最后输出三维归一化平面的坐标 x/z y/z  
// 针对于不同相机模型，映射的方式不一样
// cata鱼眼模型  -- 讲二维像素坐标映射到单位球面上 
// Equidistant鱼眼模型
// Pinhole
// PinholeFull 
// scaramuzza
/**
 * 像素点计算归一化相机平面点，带畸变校正
*/
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

// 其为当前帧相对于前一帧 特征点沿x,y方向的像素移动速度
// 根据上一帧图像的特征点和当前帧图像的特征点，来计算点运动的速度
/**
 * 计算当前帧归一化相机平面特征点在x、y方向上的移动速度
 * @param pts 当前帧归一化相机平面特征点
*/
vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}


//标注出特征点 并画出特征追踪的线
//在imTrack图像上画出特征点
/**
 * 展示，左图特征点用颜色区分跟踪次数（红色少，蓝色多），画个箭头指向前一帧特征点位置，如果是双目，右图画个绿色点
 * @param imLeft            当前帧左图
 * @param imRight           当前帧右图
 * @param curLeftIds        当前帧左图特征点id
 * @param curLeftPts        当前帧左图特征点
 * @param curRightPts       当前帧右图特征点
 * @param prevLeftPtsMap    前一帧左图特征点
*/
void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
      // 单目用左图，双目左右图放一起// 图像凭借hconcat（B,C，A）; // 等同于A=[B  C]
    if (!imRight.empty() && stereo_cam && !depth_cam)//todo
        cv::hconcat(imLeft, imRight, imTrack);//https://blog.csdn.net/guanguanboy/article/details/106802689
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);    // 左图特征点画个圈，红色跟踪次数少，蓝色跟踪次数多
    }
    if (!imRight.empty() && stereo_cam && !depth_cam)//todo
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;//如果有右图，把右图上点移动到合并矩阵的右边去
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //画出左右目的匹配直线 curLeftPtsTrackRight找不到啊！
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);// 左图特征点画个箭头指向前一帧特征点位置
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));

    
    //todo 3.24
    // cv::Mat track = imTrack(cv::Range(0, cur_img.rows), cv::Range(0, cur_img.cols));
    // cv::imwrite("/home/xuwenyu/00/" + to_string(cur_time) + "_track.png", track);
}


//把上一帧3d点预测到归一化平面，预测方法好像就是直接把3D点投影下来。
// 填充predict_pts
/**
 * 用前一帧运动估计特征点在当前帧中的位置
*/
void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())// 若该路标点进行了预测
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);//)3d点计算像素点，可带畸变
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);  //未预测，采用路标点在上一帧图像位置的坐标
    }
}


/**
 * 指定outlier点，并删除
*/
void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    reduceVector(scores,status);//todo
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}

cv::Mat FeatureTracker::getSelectImage()
{
    cv::Mat track = imTrack(cv::Range(0, cur_img.rows), cv::Range(0, cur_img.cols));
    // cv::Mat tmp;
    // cv::cvtColor(cur_img, tmp, CV_GRAY2RGB);
    // cv::vconcat(tmp, track, tmp);//https://blog.csdn.net/guanguanboy/article/details/106802689

    // cv::imshow("track_img", track);
    // cv::waitKey(2);
    // cv::imwrite("/home/xuwenyu/00/" + to_string(cur_time) + "_track.png", track);
    return track;
}