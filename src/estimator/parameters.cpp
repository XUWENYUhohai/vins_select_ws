/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;//最小视差，//滑动窗口中,是删除最老帧,还是倒数第二针,是通过新帧和倒数第二帧的视差决定,也就是这个最小像素值,还需要除以焦距
double ACC_N, ACC_W;
double GYR_N, GYR_W;

// body to camera 相对外参
std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;//最大求解器迭代时间（毫秒），以确保实时
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;// 外部参数校准开关
int ESTIMATE_TD;// 校准时间的开关
int ROLLING_SHUTTER; // 这个并没有赋值！！
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;//定义VIO输出的路径
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;// 相片的行和列
double TD; // 输入的相机和imu时间的差值
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;// 这个也没有赋值
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;// 这个并没有赋值！！
std::vector<std::string> CAM_NAMES;
int MAX_CNT;//max feature number in feature tracking
int MIN_DIST;//min distance between two features 
double F_THRESHOLD;// ransac threshold (pixel)
int SHOW_TRACK;//publish tracking image as topic 显示特征点与特征追踪的线
int FLOW_BACK;// perform forward and backward optical flow to improve feature tracking accuracy反向光流追踪



//todo ps：新的变量已在parameters.h中添加为外部变量
int DEPTH;
//add odom topic
int USE_ODOM;
std::string ODOM_TOPIC;

int MIN_CONTAIN_FRAME;
int EQUALIZE = 1;//直方图均衡
//odom与camera的外参
Eigen::Matrix3d Roc;
Eigen::Vector3d Toc;
//运行模式参数
string AGV_MODEL;
int KEYFRAMEWINDOWSIZE;

int LASER;
//add laser topic
std::string LASER_TOPIC;
 //laser与camera的外参
Eigen::Matrix3d Rlc;
Eigen::Vector3d Tlc;
//todo




/**
 * ROS参数读取
*/
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    //    getParam(键,存储结果的变量)
    //         存在,返回 true,且将值赋值给参数2
    //         若果键不存在，那么返回值为 false，且不为参数2赋值
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

/**
 * YAML配置读取
*/
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");//// 以只读的权限打开参数文件,用于判断文件是否存在，真正的读取在cv::FileStorage
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();//中止程序执行，提供有用信息，提示是在那个文件中失败的
        return;          
    }
    fclose(fh);//打开文件之后一定记得关闭

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ); // XML / YAML文件存储类，封装了将数据写入文件或从文件读取数据所需的所有信息。
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    //读取对应名称的参数，在编写自己的参数文件时可以参照
    //图像的话题
    fsSettings["image0_topic"] >> IMAGE0_TOPIC;//字符型变量用的是>>
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;



    //todo
    USE_ODOM = fsSettings["odom"];
    if(USE_ODOM)
    {
        fsSettings["odom_topic"] >> ODOM_TOPIC;
    }
    
    // fsSettings["laser_topic"] >> LASER_TOPIC;
    //todo
    



    MAX_CNT = fsSettings["max_cnt"];//特征跟踪中的最大特征数
    MIN_DIST = fsSettings["min_dist"];//两个特征点最小的距离
    F_THRESHOLD = fsSettings["F_threshold"]; //ransac 阈值 (pixel)
    SHOW_TRACK = fsSettings["show_track"];    //是否将跟踪图像发布为主题
    FLOW_BACK = fsSettings["flow_back"];// 是否执行正向和反向光流以提高特征跟踪精度

    MULTIPLE_THREAD = fsSettings["multiple_thread"]; //是否开启多线程

    USE_IMU = fsSettings["imu"];//是否使用IMU
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
         //IMU内参：
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];  //重力加速度大小，根据地区修改
    }

    SOLVER_TIME = fsSettings["max_solver_time"];//求解器最大迭代时间单位ms
    NUM_ITERATIONS = fsSettings["max_num_iterations"];   //求解器最大求解次数
    MIN_PARALLAX = fsSettings["keyframe_parallax"]; //关键帧选择阈值（视差像素）
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);//定义一个ofstream的流，以输出方式打开，内存到文件
    fout.close();


     /**
     * 是否进行相机和IMU在线标定
     * 0：不进行标定有一个准确的外参
     * 1：进行标定，初步猜测外在参数，将围绕最初的外参，对外参进行优化。
     * 2：进行标定，没有外参，完全依靠VINS系统的在线标定
     */
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);//讲opencv中的Mat格式转化为eigen中的matrix格式
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 



    //todo
    //读取base_link 到camera的转换关系   T_bc或T_oc
    // cv::Mat cv_T_oc;
    // fsSettings["odom_T_cam"] >> cv_T_oc;
    // Eigen::Matrix4d T_oc;
    // cv::cv2eigen(cv_T_oc, T_oc);
    // Roc<<T_oc.block<3, 3>(0, 0);
    // Toc<<T_oc.block<3, 1>(0, 3);

    // cv::Mat cv_T_lc;
    // fsSettings["laser_T_cam"] >> cv_T_lc;
    // Eigen::Matrix4d T_lc;
    // cv::cv2eigen(cv_T_lc, T_lc);
    // Rlc<<T_lc.block<3, 3>(0, 0);
    // Tlc<<T_lc.block<3, 1>(0, 3);
    //todo
    



    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }

   // 为寻找相机参数文件所在的位置做准备
    int pn = config_file.find_last_of('/');//https://blog.csdn.net/zhangxiao93/article/details/54381613/ ，'\\'==\ ,  https://blog.csdn.net/xibi199011/article/details/124397873
    std::string configPath = config_file.substr(0, pn);// 返回0到pn子字符
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }




    //todo
    DEPTH = fsSettings["depth"];
    // if(DEPTH == 1)
    // {
    //     cv::Mat cv_T;
    //     fsSettings["body_T_cam1"] >> cv_T;
    //     Eigen::Matrix4d T;
    //     cv::cv2eigen(cv_T, T);
    //     RIC.push_back(T.block<3, 3>(0, 0));
    //     TIC.push_back(T.block<3, 1>(0, 3));
    //     //NUM_OF_CAM++;
    // }
    //todo



    INIT_DEPTH = 5.0;//初始深度
    BIAS_ACC_THRESHOLD = 0.1;//acc偏置的阈值？
    BIAS_GYR_THRESHOLD = 0.1;


    //是否在线估计时间
    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }
   //释放文件读取类
    fsSettings.release();
}
