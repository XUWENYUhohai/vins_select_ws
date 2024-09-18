#pragma once

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"

//nanoflann是一个c++11标准库，用于构建具有不同拓扑（R2，R3（点云），SO(2)和SO(3)（2D和3D旋转组））的KD树。nanoflann不需要编译或安装。你只需要#include <nanoflann.hpp>在你的代码中
#include "/home/xuwenyu/vins_stereo_ws/src/VINS-Fusion/vins_estimator/thridparty/nanoflann/nanoflann.hpp"
#include "../utility/utility.h"
#include "../utility/state_defs.h"
#include "../utility/horizon_generator.h"

// #include "../estimator/feature_manager.h"
// #include "../estimator/estimator.h"

#include "../estimator/parameters.h"
#include "../estimator/feature_manager.h"

#include <random>//todo

class FeatureSelector
{
public:
    FeatureSelector(FeatureManager * f_manager, camodocal::CameraPtr& m_camera, Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[]);
    ~FeatureSelector() = default;

    void setPara(double acc_var, double acc_bias_var, int max_num, int init_threshold);

  /**
   * @brief         Select the most informative subset of features to track
   *
   * @param[inout]  image   A set of calibrated pixels, indexed by feature id
   * @param[in]     header  The header (with timestamp) of the corresponding image
   * @param[in]     num_odom  The number of Odom measurements between the prev frame and now
   * 
   * @return        <historic_ids, new_ids> of features
   */
  std::pair<std::vector<int>, std::vector<int>> select(image_t& image, const double& header, int num_odom, int stereo, cv::Mat& img);//todo 3.24

  //todo 12.13 all in weight(prob * lifeTime * delta_ells) 
  std::pair<std::vector<int>, std::vector<int>> allIn(image_t& image, const double& header, int num_odom, int stereo);
  
  std::pair<std::vector<int>, std::vector<int>> roulette(image_t& image, const double& header, int num_odom, int stereo);
  //todo 

  /**
   * @brief      Provides the (yet-to-be-corrected) pose estimate
   *             for frame k+1, calculated via Odom propagation.
   *
   * @param[in]  time_stamp  Image clock stamp of frame k+1
   * @param[in]  P     Position at time k+1        (P_WB)
   * @param[in]  Q     Orientation at time k+1     (Q_WB)
   * @param[in]  V     Linear velocity at time k+1 (V_WB)
   * @param[in]  a     Linear accel at time k+1    (a_WB)
   * @param[in]  w     Angular vel at time k+1     (w_WB)
   * @param[in]  Ba    Accel bias at time k+1      (in sensor frame)
   */
  void setNextStateFromOdomPropagation(double time_stamp, const Eigen::Vector3d& pre_P, const Eigen::Quaterniond& pre_Q, 
                                       const Eigen::Vector3d& predict_P, const Eigen::Quaterniond& predict_Q);

  cv::Mat getSelectImg();//todo 3.24


private:

  cv::Mat select_img;//todo 3.24
  double select_time;//todo 3.24

  // ROS stuff
  ros::NodeHandle nh_;

  camodocal::CameraPtr m_camera_; ///< geometric camera model

   FeatureManager * f_manager_ptr_;
  // FeatureManager f_manager_;

  const Eigen::Matrix3d * Rs_;
  const Eigen::Vector3d * Ps_; 

  /**
   * @brief This is the largest feature_id from the previous frame.
   *        In other words, every feature_id that is larger than
   *        this id is considered a new feature to be selected.
   */
  int last_feature_id_ = 0;

  // feature ids that have been selected and passed to the backend
  std::vector<int> tracked_features_;

  bool first_image_ = true;//< All image features are added on first image
  int init_threshold_;//< while not initialized, keep at least this many features

  int max_num_;//< how many features should be maintained?

  // Odom / Camera parameters. TODO: Check if these are/should be discrete
  double acc_var_ = 0.01;
  double acc_bias_var_ = 0.0001;

  // which metric to use
  typedef enum {log_det, min_eig} metric;//todo 论文里log det和信息矩阵最小特征值
  metric metric_ = log_det;

  std::unique_ptr<HorizonGenerator> hg_;

  //state
  state_t state_k_;///< state of last frame, k (from backend)
  state_t state_k1_;///< state of current frame, k+1 (from odom prop)
  Eigen::Vector3d ak1_;///< latest accel measurement, k+1 (from odom)
  Eigen::Vector3d wk1_;///< latest ang. vel. measurement, k+1 (from odom)

  /**
   * @brief Dataset adapter for nanoflann
   */
  struct PointCloud
  {
    PointCloud(const std::vector<std::pair<double, double>>& dataset) : pts(dataset){};
    const std::vector<std::pair<double, double>>& pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const {return pts.size();}

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    // 返回给定索引处的点的指定维度的值
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
      if(dim == 0) return pts[idx].first;
      else /*if (dim == 1)*/ return pts[idx].second;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    // 估计数据集的边界框
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

  };

  // nanoflann kdtree for guessing depth from existing landmarks
  // 创建kd树
  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud,2 /*dim*/> kd_tree_;//https://blog.csdn.net/qq_45108743/article/details/130932742
  std::unique_ptr<kd_tree_> ptr_kd_tree_;

  /**
   * @brief      Split features on the provided key
   *
   * @param[in]  k          feature_id to split on (k will not be in image_new)
   * @param      image      Feature set to split
   * @param      image_new  Any features with id after k
   */
  void splitFeatureId(int k, image_t& image, image_t& image_new);

  /**
   * @brief      Generate a future state horizon from k+1 to k+H
   *
   * @param[in]  header             The header of the current image frame (k+1)
   * @param[in]  num_odom           Num Odom measurements from prev to current frame
   * @param[in]  delta              Sampling period of Odom
   * @param[in]  delta_frame         Sampling period of image frames
   *
   * @return     a state horizon that includes the current state, [xk xk+1:k+H]
   */
  // state_horizon_t generateFutureHorizon(const std_msgs::Header& header, int num_odom, double delta, double delta_frame);
  state_horizon_t generateFutureHorizon();

  /**
   * @brief      Calculate the expected info gain from selection of the l'th feature
   *
   * @param[in]  image          Feature data in this image
   * @param[in]  x_kkH          States over horizon
   *
   * @return     delta_ells (information matrix for each feature in image)
   */
  std::map<int, omega_horizon_t> calcInfoFromFeature(const image_t& image, const state_horizon_t& x_kkH);


  //todo 12.20
  std::map<int, omega_horizon_t> calcInfoFromRightFeature(const image_t& image, const state_horizon_t& x_kkH);

  /**
   * @brief      Check if the pixel location is within the field of view
   *
   * @param[in]  p     The 2-vector with pixels (u,v)
   *
   * @return     True if the pixels are within the FOV
   */
  bool inFOV(const Eigen::Vector2d& p);

  /**
   * @brief      Initialize the nanoflann kd tree used to guess
   *             depths of new features based on their neighbors.
   *
   * @return     A vector of depths (of PGO features) to be used
   *             return the depth once a neighbor has been found
   *             (see findNNDepth).
   */
  std::vector<double> initKDTree();

  //todo 12.20
  std::vector<double> initRightKDTree();


  /**
   * @brief      Guess the depth of a new feature bearing vector by
   *             finding the nearest neighbor in the point cloud
   *             currently maintained by VINS-Mono and using its depth.
   *
   * @param[in]  depths  Depths of VINS-Mono features returned by iniKDTree
   * @param[in]  x       the normalized image plane x direction (calib pixel)
   * @param[in]  y       the normalized image plane y direction (calib pixel)
   *
   * @return     the guessed depth of the new feature
   */
  double findNNDepth(const std::vector<double>& depths, double x, double y);

  /**
   * @brief      Calculate the expected info gain from robot motion
   *
   * @param[in]  x_kkH           The horizon (rotations are needed)
   * @param[in]  num_odom        Num Odom measurements between frames
   * @param[in]  delta           Sampling period of Odom
   *
   * @return     Returns OmegaOdom (bar) from equation (15)
   */
  omega_horizon_t calcInfoFromRobotMotion(const state_horizon_t& x_kkH, double num_odom, double delta);

  /**
   * @brief      Create Ablk and OmegaOdom (no bar, eq 15) using pairs
   *             of consecutive frames in the horizon.
   *
   * @param[in]  Qi                 Orientation at frame i
   * @param[in]  Qj                 Orientation at frame j
   * @param[in]  num_odom  Num Oodm measurements between frames
   * @param[in]  delta           Sampling period of Odom
   *
   * @return     OmegaOdom (no bar) and Ablk for given frame pair
   */
  std::pair<omega_t, ablk_t> createLinearOdomMatrices(const Eigen::Quaterniond& Qi, const Eigen::Quaterniond& Qj, double num_odom, double delta);

  /**
   * @brief         Add OmegaPrior (from back end) to odom prop information
   *
   * @param[inout]  Omega  The omega from odom prop (Omegaodom)
   */
  void addOmegaPrior(Eigen::Ref<omega_horizon_t> Omega);//https://blog.csdn.net/Darlingqiang/article/details/132042184
  
  /**
   * @brief      Run lazy and greedy selection of features
   *
   * @param[inout] subset           Subset of features to add our selection to
   * @param[in]    image            All of the features from the current image
   * @param[in]    kappa            The number of features to try and select
   * @param[in]    Omega_kkH        Information from robot motion
   * @param[in]    Delta_ells       New features and their corresponding info
   * @param[in]    Delta_used_ells  Currently tracked features and their info  
   * 
   * @return       Feature ids of new features that were selected
   */
  std::vector<int> selectSubset(image_t& subset, const image_t& image, int kappa, const omega_horizon_t& Omega_kkH, 
                                const std::map<int, omega_horizon_t>& Delta_ells, const std::map<int, omega_horizon_t>& Delta_used_ells);


  /**
   * @brief      Calculate and sort upper bounds of logDet metric when adding
   *             each feature to the current subset independently of the other
   *
   * @param[in]  Omega       Information from robot motion and current tracks
   * @param[in]  OmegaS      The information content of the current subset
   * @param[in]  Delta_ells  New features and their corresponding information
   * @param[in]  blackList   Feature ids that have already been selected
   * @param[in]  image       All of the features from the current image
   *
   * @return     A (desc) sorted map of <UB（upper bounds）, feature id>
   */
  std::map<double, int, std::greater<double>> sortLogDetUB(const omega_horizon_t& Omega, const omega_horizon_t& OmegaS,
                                                           const std::map<int, omega_horizon_t>& Delta_ells, const std::vector<int>& blackList, 
                                                           const image_t& image);
};