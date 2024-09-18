#include "feature_select.h"

FeatureSelector::FeatureSelector(FeatureManager * f_manager, camodocal::CameraPtr& m_camera, Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[])
                                 : m_camera_(m_camera), Rs_(Rs), Ps_(Ps)
{
  // create future state horizon generator / manager
  f_manager_ptr_ = f_manager;
  hg_ = std::unique_ptr<HorizonGenerator>(new HorizonGenerator());
}

// ----------------------------------------------------------------------------

void FeatureSelector::setPara(double acc_var, double acc_bias_var, int max_num, int init_threshold)
{
  acc_var_ = acc_var;
  acc_bias_var_ = acc_bias_var;
  max_num_ = max_num;
  init_threshold_ = init_threshold;
}

// ----------------------------------------------------------------------------

void FeatureSelector::setNextStateFromOdomPropagation(double time_stamp, const Eigen::Vector3d& pre_P, const Eigen::Quaterniond& pre_Q, 
                                     const Eigen::Vector3d& predict_P, const Eigen::Quaterniond& predict_Q)
{
  // State of previous frame (at the end of the fixed-lag window)
  state_k_.first.coeffRef(xTIMESTAMP) = time_stamp;
  state_k_.first.segment<3>(xPOS) = pre_P;
  state_k_.second = pre_Q;

  // set the propagated-forward state of the predict frame
  //   state_k1_.first.coeffRef(xTIMESTAMP) = time_stamp + ？; 
  state_k1_.first.segment<3>(xPOS) = predict_P;
  state_k1_.second = predict_Q;
}

// ----------------------------------------------------------------------------

std::pair<std::vector<int>, std::vector<int>> FeatureSelector::select(image_t& image, const double& header, int num_odom, int stereo, cv::Mat& img)
{

  // select_img = img;//todo 3.24
  // cv::cvtColor(select_img, select_img, CV_GRAY2RGB);
  // select_time =  header;


  // for (auto & tmp_image : image)
  // {
  //   cv::Point2f point(tmp_image.second[0].second.coeff(3), tmp_image.second[0].second.coeff(4));
  //   cv::circle(select_img, point, 2, cv::Scalar(255, 0, 0), 2);
  // }
  


  //
  // Timing information
  //

  // frame time of previous image
  static double frame_time_k = header;

  // time difference between last frame and current frame
  double delta_frame = header - frame_time_k;

  // calculate the IMU sampling rate of the last frame-to-frame meas set
  double delta_odom = delta_frame / num_odom;

  //
  // Decide which features are new and which are already being used
  //

  // remove new features from image and put into image_new.
  // Image will only contain features that are currently being tracked.
  // TODO: Is this true? Does VINS-Mono use *all* features given to it?
  image_t image_new;
  splitFeatureId(last_feature_id_, image, image_new);//第一帧时image_new = image   image.zero()//todo  3.24

  if(!image_new.empty()) last_feature_id_ = image_new.crbegin()->first;//todo

  // cout << "last_feature_id_ : " << last_feature_id_ << endl;//todo
  // cout << "image_new.size() : " << image_new.size() << endl;//todo


  // the subset of features to pass to VINS-Mono back end
  image_t subset;//todo

  // add in previously tracked features  //todo 这里隐密表示了life time的作用


  // for (auto fid : tracked_features_)
  // {// attempt to retrieve this feature from image
  //   auto feature = image.find(fid);
  //   if (feature != image.end())
  //   {
  //     subset[fid] = feature->second;//todo 12.4 可不可以去掉
  //   }
  //   //todo NOTE: We are not removing not found features because they could pop up again (i.e., loop-closure (?), missed detections, etc.)
  // }
  

  // cout << "subset.size : " << subset.size() << endl;//todo

  // ROS_WARN_STREAM("Feature subset initialized with " << subset.size() << " out"
                  // " of " << trackedFeatures_.size() << " known features");

  //
  // Future State Generation
  //

  // We will need to know the state at each frame in the horizon, k:k+H.
  // Note that this includes the current optimized state, xk
  auto state_kkH = generateFutureHorizon();
  // hg_->visualize(header, state_kkH);

  // for (auto & state : state_kkH)
  // {
  //   cout << "xVector = " << state.first << endl;
  //   cout << "Q = " << state.second.toRotationMatrix() << endl;
  // }
  


  //
  // Anticipation: Compute the Expected Information over the Horizon
  //

  // Calculate the information content from motion over the horizon (eq 15)
  auto omega_kkH = calcInfoFromRobotMotion(state_kkH, num_odom, delta_odom);

  


  // Add in prior information to OmegaIMU_kkH (eq 16)
  addOmegaPrior(omega_kkH);

  // cout << "omega_kkH.logdet : " << Utility::logdet(omega_kkH, true) << endl;//todo
  // cout << "omega_kkH : " << omega_kkH << endl;//todo

  // Calculate the information content of each of the new features
  auto delta_ells = calcInfoFromFeature(image_new, state_kkH);//todo
  // auto delta_ells = calcInfoFromFeature(image, state_kkH);//todo

  //todo 12.20
  if (stereo)
  {
    auto delta_ells_right = calcInfoFromRightFeature(image_new, state_kkH);//todo
    // auto delta_ells_right = calcInfoFromRightFeature(image, state_kkH);

    for(auto & delta : delta_ells)
    {
      if(delta_ells_right.find(delta.first) != delta_ells_right.end())
      {
        delta.second += delta_ells_right[delta.first];
      }
    }
  }
  //todo 12.20

  // cout << "delta_ells : " << delta_ells.size() << endl;//todo
  // for (size_t i = 0; i <  delta_ells.size(); i++)
  // {
    // cout << "delta_ells.logdet : " << Utility::logdet(delta_ells[i], true) << endl;//todo
  // }
  

  // Calculate the information content of each of the currently used features
  std::map<int, omega_horizon_t> delta_used_ells;
  delta_used_ells = calcInfoFromFeature(subset, state_kkH);//todo

  //
  // Attention: Select a subset of features that maximizes expected information
  //
  
  // We would like to only track N features total (including currently tracked
  // features). Therefore, we will calculate how many of the new features should
  // be selected (kappa).
  int kappa = std::max(0, max_num_ - static_cast<int>(subset.size()));

  // so we can keep track of the new features we chose
  std::vector<int> selectedIds;
  selectedIds.reserve(kappa);

  // Only select features if VINS-Mono is initialized
  if(!first_image_)
  {
    selectedIds = selectSubset(subset, image_new, kappa, omega_kkH, delta_ells, delta_used_ells);//subset也在此时更新了 //todo
    // selectedIds = selectSubset(subset, image, kappa, omega_kkH, delta_ells, delta_used_ells);//subset也在此时更新了

  }
  else
  {
    subset.swap(image_new);//todo
    // subset.swap(image);
    for(const auto& fpair : subset) tracked_features_.push_back(fpair.first);
    first_image_ = false;
  }
  
  // cout << "kappa.size : " << kappa << endl;//todo
  // cout << "selectedIds.size : " << selectedIds.size() << endl;//todo
  // cout << "tracked_features_.size : " << tracked_features_.size() << endl;//todo
  // cout << "Rs[] = " << Rs_[9] << endl;
  // cout << "m_camera_>imageWidth = " << m_camera_->imageWidth() << endl;
  // cout << "RIC[0] = " << TIC[1] << endl;

  // return best features to use for VINS-Mono
  //todo 实际返回了subset
  image.swap(subset);

  // keep track of which features have been passed to the back end. If we see
  // these features again, we need to let them through unharrassed.
  tracked_features_.insert(tracked_features_.end(), selectedIds.begin(), selectedIds.end());
  

  frame_time_k = header;

  return std::make_pair(tracked_features_, selectedIds);//std::make_pair可以不用<>，std::pair需要<>
}



// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------
void FeatureSelector::splitFeatureId(int k, image_t& image, image_t& image_new)
{
  // pick up after feature_id k
  auto it = image.upper_bound(k);
  bool found = (it != image.end());

  // if found, copy new features to image_new and remove from image
  if (found) {
    image_new.insert(it, image.end());
    image.erase(it, image.end());
  }
}

// ----------------------------------------------------------------------------

state_horizon_t FeatureSelector::generateFutureHorizon()
{
  // generate the horizon based on the requested scheme
  return hg_->groundTruth(state_k_,state_k1_);
}

// ----------------------------------------------------------------------------

omega_horizon_t FeatureSelector::calcInfoFromRobotMotion(const state_horizon_t& x_kkH, double num_odom, double delta)
{
  // ** Build the large information matrix over the horizon (eq 15).
  //
  // There is a sparse structure to the information matrix that we can exploit.
  // We can calculate the horizon info. matrix in blocks. Notice that each
  // pair of consecutive frames in the horizon create four 9x9 sub-blocks.
  // For example, for a horizon of H=3, the first pair of images (h=1) creates
  // a large information matrix like the following (each block is 9x9):
  //
  //         |------------------------------------
  //         | At*Ω*A |  At*Ω  |    0   |    0   |
  //         |------------------------------------
  //         |   Ω*A  |    Ω   |    0   |    0   |
  //         |------------------------------------
  //         |    0   |    0   |    0   |    0   |
  //         |------------------------------------
  //         |    0   |    0   |    0   |    0   |
  //         |------------------------------------
  //
  // The four non-zero sub-blocks shift along the diagonal as we loop through
  // the horizon (i.e., for h=2 there are zeros on all the edges and for h=3
  // the Ω is in the bottom-right corner). Note that the Ai matrix must be
  // recomputed for each h. The Ω matrix is calculated as the inverse of
  // the covariance in equation (52) and characterizes the noise in a
  // preintegrated set of IMU measurements using the linear IMU model.

  // NOTE: We are even more clever and only calculate the upper-triangular
  // and then transpose since this is a symmetric PSD matrix

  omega_horizon_t omega_kkH = omega_horizon_t::Zero();

  for (int h = 1; h <= HORIZON; h++)// for consecutive frames in horizon
  {
    // convenience: frames (i, j) are a consecutive pair in horizon
    const auto& Qi = x_kkH[h - 1].second;
    const auto& Qj = x_kkH[h].second;

    // Create Ablk and Ω as explained in the appendix
    auto mats = createLinearOdomMatrices(Qi, Qj, num_odom, delta);

    // convenience: select sub-blocks to add to, based on h
    Eigen::Ref<omega_t> block1 = omega_kkH.block<STATE_SIZE, STATE_SIZE>((h - 1) * STATE_SIZE, (h - 1) * STATE_SIZE);
    Eigen::Ref<omega_t> block2 = omega_kkH.block<STATE_SIZE, STATE_SIZE>((h - 1) * STATE_SIZE, h * STATE_SIZE);
    Eigen::Ref<omega_t> block3 = omega_kkH.block<STATE_SIZE, STATE_SIZE>(h * STATE_SIZE, (h - 1) * STATE_SIZE);
    Eigen::Ref<omega_t> block4 = omega_kkH.block<STATE_SIZE, STATE_SIZE>(h * STATE_SIZE, h * STATE_SIZE);

    // At*Ω*A (top-left sub-block)
    block1 += mats.second.transpose() * mats.first * mats.second;

    // At*Ω (top-right sub-block)
    auto tmp = mats.second.transpose() * mats.first;
    block2 += tmp;
    // Ω*A (bottom-left sub-block)
    block3 += tmp.transpose();
    // Ω (bottom-right sub-block)
    block4 += mats.first;
  }
  
  return omega_kkH;
}

// ----------------------------------------------------------------------------

std::pair<omega_t, ablk_t> FeatureSelector::createLinearOdomMatrices(const Eigen::Quaterniond& Qi, const Eigen::Quaterniond& Qj, double num_odom, double delta)
{
  //
  // "Pre-integrate" future IMU measurements over horizon
  //

  // helper matrices, equations (47) and (48)
  Eigen::Matrix3d Nij = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Mij = Eigen::Matrix3d::Zero();

  // initialize block coefficients
  double CCt_11 = 0;
  double CCt_12 = 0;

  // This is an ODOM-rate for loop
  for (int i = 0; i < num_odom; i++)
  {
    // slerp from Qi toward Qj by where we are in between the frames
    // (this will never slerp all the way to Qj)
    auto q = Qi.slerp(i/static_cast<double>(num_odom), Qj);
    // auto q = Qi.inverse() * Qj;

    // so many indices...
    double jkh = (num_odom - i - 0.5);
    Nij += jkh * q.toRotationMatrix();
    Mij += q.toRotationMatrix();

    // entries of CCt
    CCt_11 += jkh*jkh;
    CCt_12 += jkh;
  }
  
  // powers of IMU sampling period
  const double delta_2 = delta * delta;
  const double delta_3 = delta * delta * delta;
  const double delta_4 = delta * delta * delta * delta;

  //
  // Build cov(eta^imu_ij) -- see equation (52)
  //

  // NOTE: In paper, bottom right entry of CCt should have (j-k), not (j-k-1).
  //todo 原先有乘num_odom应该是错的
  omega_t cov = omega_t::Zero();
  cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * CCt_11 * delta_4 * acc_var_;
  cov.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * CCt_12 * delta_3 * acc_var_;
  cov.block<3, 3>(3, 0) = cov.block<3, 3>(0, 3);
  cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * num_odom * delta_2 * acc_var_;
  cov.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * acc_bias_var_;

  //
  // Build Ablk -- see equation (50)
  //

  Nij *= delta_2;
  Mij *= delta;

  ablk_t ablk = - ablk_t::Identity();
  ablk.block<3, 3>(0, 3) = - Eigen::Matrix3d::Identity() * num_odom * delta;
  ablk.block<3, 3>(0, 6) = Nij;
  ablk.block<3, 3>(3, 6) = Mij;

  // cout << "delta = " << delta << endl;
  // cout << "cov = " << cov << endl;
  // cout << "ablk = " << ablk << endl;


  return std::pair<omega_t, ablk_t>(cov.inverse(), ablk);
}

// ----------------------------------------------------------------------------

void FeatureSelector::addOmegaPrior(Eigen::Ref<omega_horizon_t> Omega)
{
  // upper-left sub-block -- use identity for now to keep det(OmegakkH) > 0
  // Add the prior to (upper-left) OmegaIMU (input)
  Omega.block<STATE_SIZE, STATE_SIZE>(0, 0) += omega_t::Identity();
}

// ----------------------------------------------------------------------------
std::map<int, omega_horizon_t> FeatureSelector::calcInfoFromFeature(const image_t& image, const state_horizon_t& x_kkH)
{
  std::map<int, omega_horizon_t> delta_ells;

  // convenience: (yet-to-be-corrected) transformation
  // of camera frame w.r.t world frame at time k+1
  const auto& t_wc_k1 = state_k1_.first.segment<3>(xPOS) + state_k1_.second * TIC[0];
  const auto& q_wc_k1 = state_k1_.second * RIC[0];
  // 11.28
  // const auto& t_wc_k = state_k_.first.segment<3>(xPOS) + state_k_.second * TIC[0];
  // const auto& q_wc_k = state_k_.second * RIC[0];

  auto depthsByIdx = initKDTree();//将vins中的特征点转为time (k+1)的相机归一化坐标系下的点

  // cout << "depthsByIdx.size : " << depthsByIdx.size() << endl;//todo

  for (const auto& fpair : image)//todo image time == header == k + 1
  {
    // there is only one camera, so we expect only one vector per feature //todo 就算变成双目好像也不要紧
    constexpr int c = 0;

    // extract feature id and nip vector from obnoxious data structure
    int feature_id = fpair.first;
    Eigen::Vector3d feature = fpair.second[c].second.head<3>();// calibrated [u v 1] 这里应该是归一化坐标点

    // scale bearing vector by depth
    double d = findNNDepth(depthsByIdx, feature.coeff(0), feature.coeff(1));

    // cout << "d = " << d << endl;//todo
    if(d < 0) continue;                                     //! 11.30 先这样判断，直接跳过此点，测试后看效果是否改回原算法
    feature = feature * d;//todo 与原算法有变化11.28

    // Estimated position of the landmark w.r.t the world frame  世界点
    auto pell = t_wc_k1 + q_wc_k1 * feature;

    //
    // Forward-simulate the feature bearing vector over the horizon
    //
    
    // keep count of how many camera poses could see this landmark.
    // We know it's visible in at least the current (k+1) frame.
    int numVisible = 1;

    // for storing the necessary blocks for the Delta_ell information matrix
    // NOTE: We delay computation for h=k+1 until absolutely necessary.
    Eigen::Matrix<double, 3, 3*HORIZON> Ch; // Ch == BtB_h in report
    Ch.setZero();

    // Also sum up the Ch blocks for EtE;
    Eigen::Matrix3d EtE = Eigen::Matrix3d::Zero();

    // NOTE: start forward-simulating the landmark projection from k+2
    // (we have the frame k+1 projection, since that's where it came from)
    for (int h = 2; h <= HORIZON; h++)
    {
      // convenience: future camera frame (k+h) w.r.t world frame
      const auto& t_wc_h = x_kkH[h].first.segment<3>(xPOS) + x_kkH[h].second * TIC[0];
      const auto& q_wc_h = x_kkH[h].second * RIC[0];

      // create bearing vector of feature w.r.t camera pose h
      Eigen::Vector3d uell = q_wc_h.inverse() * (pell - t_wc_h);

      // TODO: Maybe flip the problem so we don't have to do this every looped-loop
      // project to pixels so we can perform visibility check
      Eigen::Vector2d pixels;
      m_camera_->spaceToPlane(uell, pixels);

      // If not visible from this pose, skip
      if (!inFOV(pixels)) continue;

      // Calculate sub-block of Delta_ell (zero-indexing)
      Eigen::Matrix3d Bh = Utility::skewSymmetric(uell.normalized()) * (q_wc_h * RIC[0]).transpose();

      Ch.block<3, 3>(0, 3 * (h - 1)) = Bh.transpose() * Bh;
      // Sum up block for EtE
      EtE += Ch.block<3, 3>(0, 3 * (h - 1));

      ++numVisible;
    }

    // If we don't expect to be able to triangulate a point
    // then it is not useful. By not putting this feature in
    // the output map, we are effectively getting rid of it now.
    if (numVisible == 1) continue;

    // Since the feature can be triangulated, we now do the computation that
    // we put off before forward-simulating the landmark projection:
    // we calculate Ch for h=k+1 (the frame where the feature was detected)
    Eigen::Matrix3d Bh = Utility::skewSymmetric(feature.normalized()) * (q_wc_k1 * RIC[0]).transpose();
    
    Ch.block<3, 3>(0, 0) = Bh.transpose() * Bh;

    // add information to EtE
    EtE += Ch.block<3, 3>(0, 0);

    // Compute landmark covariance (should be invertible)
    Eigen::Matrix3d W = EtE.inverse(); 

    //
    // Build Delta_ell for this Feature (see support_files/report)
    //

    omega_horizon_t Delta_ell = omega_horizon_t::Zero();

    // col-wise for efficiency
    for (int j = 1; j <= HORIZON; j++)
    {
      // for convenience
      Eigen::Ref<Eigen::Matrix3d> Cj = Ch.block<3, 3>(0, 3*(j-1));

      for (int i = j; i <= HORIZON; i++)// NOTE: i=j for lower triangle  下三角   ？但下面不是上下三角都有吗
      {
        // for convenience
        Eigen::Ref<Eigen::Matrix3d> Ci = Ch.block<3, 3>(0, 3*(i-1));
        Eigen::Matrix3d Dij = Ci*W*Cj.transpose();
        if (i == j)
        {
          // diagonal
          Delta_ell.block<3, 3>(9*i, 9*j) = Ci - Dij;
        }
        else
        {
          // lower triangle
          Delta_ell.block<3, 3>(9*i, 9*j) = -Dij;

          // upper triangle
          Delta_ell.block<3, 3>(9*j, 9*i) = -Dij.transpose();
        }
      }
    }
  
  // Store this information matrix with its associated feature ID
  delta_ells[feature_id] = Delta_ell;
  }
  return delta_ells;
}

//todo 12.20
// ----------------------------------------------------------------------------
std::map<int, omega_horizon_t> FeatureSelector::calcInfoFromRightFeature(const image_t& image, const state_horizon_t& x_kkH)
{
  std::map<int, omega_horizon_t> delta_ells;

  // convenience: (yet-to-be-corrected) transformation
  // of camera frame w.r.t world frame at time k+1
  const auto& t_wc_k1 = state_k1_.first.segment<3>(xPOS) + state_k1_.second * TIC[1];
  const auto& q_wc_k1 = state_k1_.second * RIC[1];
  // 11.28
  // const auto& t_wc_k = state_k_.first.segment<3>(xPOS) + state_k_.second * TIC[0];
  // const auto& q_wc_k = state_k_.second * RIC[0];

  auto depthsByIdx = initRightKDTree();//将vins中的特征点转为time (k+1)的相机归一化坐标系下的点

  // cout << "depthsByIdx.size : " << depthsByIdx.size() << endl;//todo

  for (const auto& fpair : image)//todo image time == header == k + 1
  {
    // there is only one camera, so we expect only one vector per feature //todo 就算变成双目好像也不要紧
    constexpr int c = 1;//todo

    // extract feature id and nip vector from obnoxious data structure
    int feature_id = fpair.first;
    Eigen::Vector3d feature = fpair.second[c].second.head<3>();// calibrated [u v 1] 这里应该是归一化坐标点

    // scale bearing vector by depth
    double d = findNNDepth(depthsByIdx, feature.coeff(0), feature.coeff(1));

    // cout << "d = " << d << endl;//todo
    if(d < 0) continue;                                     //! 11.30 先这样判断，直接跳过此点，测试后看效果是否改回原算法
    feature = feature * d;//todo 与原算法有变化11.28

    // Estimated position of the landmark w.r.t the world frame  世界点
    auto pell = t_wc_k1 + q_wc_k1 * feature;

    //
    // Forward-simulate the feature bearing vector over the horizon
    //
    
    // keep count of how many camera poses could see this landmark.
    // We know it's visible in at least the current (k+1) frame.
    int numVisible = 1;

    // for storing the necessary blocks for the Delta_ell information matrix
    // NOTE: We delay computation for h=k+1 until absolutely necessary.
    Eigen::Matrix<double, 3, 3*HORIZON> Ch; // Ch == BtB_h in report
    Ch.setZero();

    // Also sum up the Ch blocks for EtE;
    Eigen::Matrix3d EtE = Eigen::Matrix3d::Zero();

    // NOTE: start forward-simulating the landmark projection from k+2
    // (we have the frame k+1 projection, since that's where it came from)
    for (int h = 2; h <= HORIZON; h++)
    {
      // convenience: future camera frame (k+h) w.r.t world frame
      const auto& t_wc_h = x_kkH[h].first.segment<3>(xPOS) + x_kkH[h].second * TIC[1];
      const auto& q_wc_h = x_kkH[h].second * RIC[1];

      // create bearing vector of feature w.r.t camera pose h
      Eigen::Vector3d uell = q_wc_h.inverse() * (pell - t_wc_h);

      // TODO: Maybe flip the problem so we don't have to do this every looped-loop
      // project to pixels so we can perform visibility check
      Eigen::Vector2d pixels;
      m_camera_->spaceToPlane(uell, pixels);

      // If not visible from this pose, skip
      if (!inFOV(pixels)) continue;

      // Calculate sub-block of Delta_ell (zero-indexing)
      Eigen::Matrix3d Bh = Utility::skewSymmetric(uell.normalized()) * (q_wc_h * RIC[1]).transpose();

      Ch.block<3, 3>(0, 3 * (h - 1)) = Bh.transpose() * Bh;
      // Sum up block for EtE
      EtE += Ch.block<3, 3>(0, 3 * (h - 1));

      ++numVisible;
    }

    // If we don't expect to be able to triangulate a point
    // then it is not useful. By not putting this feature in
    // the output map, we are effectively getting rid of it now.
    if (numVisible == 1) continue;

    // Since the feature can be triangulated, we now do the computation that
    // we put off before forward-simulating the landmark projection:
    // we calculate Ch for h=k+1 (the frame where the feature was detected)
    Eigen::Matrix3d Bh = Utility::skewSymmetric(feature.normalized()) * (q_wc_k1 * RIC[1]).transpose();
    
    Ch.block<3, 3>(0, 0) = Bh.transpose() * Bh;

    // add information to EtE
    EtE += Ch.block<3, 3>(0, 0);

    // Compute landmark covariance (should be invertible)
    Eigen::Matrix3d W = EtE.inverse(); 

    //
    // Build Delta_ell for this Feature (see support_files/report)
    //

    omega_horizon_t Delta_ell = omega_horizon_t::Zero();

    // col-wise for efficiency
    for (int j = 1; j <= HORIZON; j++)
    {
      // for convenience
      Eigen::Ref<Eigen::Matrix3d> Cj = Ch.block<3, 3>(0, 3*(j-1));

      for (int i = j; i <= HORIZON; i++)// NOTE: i=j for lower triangle  下三角   ？但下面不是上下三角都有吗
      {
        // for convenience
        Eigen::Ref<Eigen::Matrix3d> Ci = Ch.block<3, 3>(0, 3*(i-1));
        Eigen::Matrix3d Dij = Ci*W*Cj.transpose();
        if (i == j)
        {
          // diagonal
          Delta_ell.block<3, 3>(9*i, 9*j) = Ci - Dij;
        }
        else
        {
          // lower triangle
          Delta_ell.block<3, 3>(9*i, 9*j) = -Dij;

          // upper triangle
          Delta_ell.block<3, 3>(9*j, 9*i) = -Dij.transpose();
        }
      }
    }
  
  // Store this information matrix with its associated feature ID
  delta_ells[feature_id] = Delta_ell;
  }
  return delta_ells;
}

// ----------------------------------------------------------------------------

std::vector<double> FeatureSelector::initKDTree()
{
  // setup dataset
  static std::vector<std::pair<double, double>> dataset;
  dataset.clear();
  dataset.reserve(f_manager_ptr_->feature.size());

  //
  // Build the point cloud of bearing vectors w.r.t camera frame k+1
  //
  
  // we want a vector of depths that match the ordering of the dataset
  // for lookup after the knn have been found
  std::vector<double> depths;
  depths.reserve(f_manager_ptr_->feature.size());

  // cout << "f_manager_.feature.size()  "   << f_manager_ptr_->feature.size() << endl;

  // copied from visualization.cpp, pubPointCloud
  for (const auto& it_per_id : f_manager_ptr_->feature)
  {
    // ignore features if they haven't been around for a while or they're not stable
    int used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) continue;
    if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1) continue;//todo 不能大于window_size的3/4没搞懂

    // TODO: Why 0th frame?
    int s_f = it_per_id.start_frame;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i = Rs_[s_f] * (RIC[0] * pts_i + TIC[0]) + Ps_[s_f];

    // w_pts_i is the position of the landmark w.r.t. the world
    // transform it so that it is the pos of the landmark w.r.t camera frame at x_k（k+1）
    Eigen::Vector3d p_IL_k1 = state_k1_.second.inverse() * (w_pts_i - state_k1_.first.segment<3>(xPOS));//todo  k+1
    Eigen::Vector3d p_CL_k1 = RIC[0].inverse() * (p_IL_k1 - TIC[0]);

    // project back to nip of the camera at time k(k+1)
    pts_i = p_CL_k1 / p_CL_k1.coeff(2);
    double x = pts_i.coeff(0), y = pts_i.coeff(1);

    dataset.push_back(std::make_pair(x, y)); //将vins中的特征点转为time k(k+1)的相机归一化坐标系下的点
    depths.push_back(p_CL_k1.coeff(2));//todo 这里改成了k+1时刻的深度
  }
  
  // point cloud adapter for currently tracked landmarks in PGO
  // keep as static because kdtree uses a reference to the cloud.
  // Note that this works because dataset is also static
  static PointCloud cloud(dataset);//不太懂

  // create the kd-tree and index the data
  ptr_kd_tree_.reset(new kd_tree_(2/* dim */, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
  ptr_kd_tree_->buildIndex();

  // these are depths of PGO features by index of the dataset
  return depths;
}

// ----------------------------------------------------------------------------
//todo 12.20
std::vector<double> FeatureSelector::initRightKDTree()
{
  // setup dataset
  static std::vector<std::pair<double, double>> dataset;
  dataset.clear();
  dataset.reserve(f_manager_ptr_->feature.size());

  //
  // Build the point cloud of bearing vectors w.r.t camera frame k+1
  //
  
  // we want a vector of depths that match the ordering of the dataset
  // for lookup after the knn have been found
  std::vector<double> depths;
  depths.reserve(f_manager_ptr_->feature.size());

  // cout << "f_manager_.feature.size()  "   << f_manager_ptr_->feature.size() << endl;

  // copied from visualization.cpp, pubPointCloud
  for (const auto& it_per_id : f_manager_ptr_->feature)
  {
    // ignore features if they haven't been around for a while or they're not stable
    int used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) continue;
    if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1) continue;//todo 不能大于window_size的3/4没搞懂

    // TODO: Why 0th frame?
    int s_f = it_per_id.start_frame;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i = Rs_[s_f] * (RIC[1] * pts_i + TIC[1]) + Ps_[s_f];

    // w_pts_i is the position of the landmark w.r.t. the world
    // transform it so that it is the pos of the landmark w.r.t camera frame at x_k（k+1）
    Eigen::Vector3d p_IL_k1 = state_k1_.second.inverse() * (w_pts_i - state_k1_.first.segment<3>(xPOS));//todo  k+1
    Eigen::Vector3d p_CL_k1 = RIC[1].inverse() * (p_IL_k1 - TIC[1]);

    // project back to nip of the camera at time k(k+1)
    pts_i = p_CL_k1 / p_CL_k1.coeff(2);
    double x = pts_i.coeff(0), y = pts_i.coeff(1);

    dataset.push_back(std::make_pair(x, y)); //将vins中的特征点转为time k(k+1)的相机归一化坐标系下的点
    depths.push_back(p_CL_k1.coeff(2));//todo 这里改成了k+1时刻的深度
  }
  
  // point cloud adapter for currently tracked landmarks in PGO
  // keep as static because kdtree uses a reference to the cloud.
  // Note that this works because dataset is also static
  static PointCloud cloud(dataset);//不太懂

  // create the kd-tree and index the data
  ptr_kd_tree_.reset(new kd_tree_(2/* dim */, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
  ptr_kd_tree_->buildIndex();

  // these are depths of PGO features by index of the dataset
  return depths;
}

// ----------------------------------------------------------------------------

double FeatureSelector::findNNDepth(const std::vector<double>& depths, double x, double y)
{
  // The point cloud and the query are expected to be in the normalized image
  // plane (nip) of the camera at time k+1 (the frame the feature was detected in)

  // If this happens, then the back end is initializing
  if (depths.size() == 0)
  {
    // cout << "select findNNDepth == -1" << endl;
    return -1.0;
  }

  // build query
  double query_pt[2] = { x, y };

  // do a knn search
  // 使用距离度量标准：
  // L1 （曼哈顿）
  // L2 （欧几里得，赞成SSE2优化）。
  // L2_Simple （欧几里得，用于像点云这样的低维数据集）。
  // SO2 （用于旋转组SO2）。
  // SO3 （欧几里得，对于旋转组SO3）。
// 原文链接：https://blog.csdn.net/u013019296/article/details/109377104
  // TODO: Considering avg multiple neighbors?
  const size_t num_results = 1;//选择n个最近关键点
  size_t ret_index = 0;//结果的index
  double out_dist_sqr;//结果的L2距离（）
  nanoflann::KNNResultSet<double> resultSet(num_results);
  resultSet.init(&ret_index, &out_dist_sqr);
  ptr_kd_tree_->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

  return depths[ret_index];
}

// ----------------------------------------------------------------------------

bool FeatureSelector::inFOV(const Eigen::Vector2d& p)
{
  constexpr int border = 0; // TODO: Could be good to have a border here  这里为0，如果需要检查靠近边缘的话可以增加
  int u = std::round(p.coeff(0));
  int v = std::round(p.coeff(1));
  return (border <= u && u < m_camera_->imageWidth() - border) && 
         (border <= v && v < m_camera_->imageHeight() - border);
}

// ----------------------------------------------------------------------------

std::vector<int> FeatureSelector::selectSubset(image_t& subset, const image_t& image, int kappa, const omega_horizon_t& Omega_kkH, 
                                const std::map<int, omega_horizon_t>& Delta_ells, const std::map<int, omega_horizon_t>& Delta_used_ells)
{
  // Combine motion information with information from features that are already
  // being used in the VINS-Mono optimization backend
  omega_horizon_t Omega = Omega_kkH;

  for (const auto& Delta : Delta_used_ells) 
  {
    Omega += Delta.second;
  }

  // blacklist of already selected features (by id)
  std::vector<int> blacklist;
  blacklist.reserve(kappa);

  // combined information of subset
  omega_horizon_t OmegaS = omega_horizon_t::Zero();

  // select the indices of the best features
  for(int i =0; i < kappa; i++)
  {
    // compute upper bounds in form of <UB, featureId> descending by UB
    auto up_bounds = sortLogDetUB(Omega_kkH, OmegaS, Delta_ells, blacklist, image);

    // cout << "up_bounds : " << up_bounds.size() << endl;//todo

    
    // initialize the best cost function value and feature ID to worst case
    double fMax = -1.0;
    // double fMax = - INT_MAX;
    int lMax = -1;

    // iterating through upperBounds in descending order, check each feature
    for (const auto& fpair : up_bounds)
    {
      int f_id = fpair.second;
      double ub = fpair.first;

      if(ub < fMax) break;

      // convenience: the information matrix corresponding to this feature
      const auto& Delta_ell = Delta_ells.at(f_id);

      // find probability of this feature being tracked
      double p = image.at(f_id)[0].second.coeff(fPROB);
      double l = image.at(f_id)[0].second.coeff(5);//!  l  效果不好12.4

      // calculate logdet efficiently
      double f_value = Utility::logdet(Omega + OmegaS + Delta_ell, true);//! 11.30 true和false还需要测试
      // double f_value = (Omega + OmegaS + p * Delta_ell).diagonal().array().log().sum();
      //todo
      // int row_index, col_index;
      // Eigen::EigenSolver<omega_horizon_t> es(Omega + OmegaS + p * Delta_ell);
      // // if (es.pseudoEigenvalueMatrix().minCoeff(&row_index, &col_index) < 0) ROS_ERROR_STREAM("(Omega + OmegaS + p * Delta_ell) min eigen value < 0!");
      // double omega_eigen_value = es.pseudoEigenvalueMatrix().minCoeff(&row_index, &col_index);
      // // if (omega_eigen_value < 0) continue;
      // double f_value = omega_eigen_value;
      //todo
      // nan check
      // if (std::isnan(f_value)) ROS_ERROR_STREAM("logdet returned nan!");//! p

      if(std::isnan(f_value)) continue;
      if(f_value <= 0) continue;

      if(f_value > fMax)//! p
      {
        fMax = f_value;   //! p
        lMax = f_id;
      }
    }

    // if lMax == -1 there was likely a nan (probably because roundoff error
    // caused det(M) < 0). I guess there just won't be a feature this iter.
    if (lMax > -1) 
    {
      // Accumulate combined feature information in subset
      double p = image.at(lMax)[0].second.coeff(fPROB);
      double l = image.at(lMax)[0].second.coeff(5);//! l
      OmegaS += Delta_ells.at(lMax);

      // add feature that returns the most information to the subset
      subset[lMax] = image.at(lMax);
      // mark as used
      blacklist.push_back(lMax);
      //todo 3.24
      // cv::Point2f point(image.at(lMax)[0].second.coeff(3), image.at(lMax)[0].second.coeff(4));
      // cv::circle(select_img, point, 2, cv::Scalar(0, 255, 0), 2);
      // string select_text = to_string((fMax - 310) * 100);
      // // auto scoreColor = cv::Scalar(0, (255 - (fMax - 310) * 100), (fMax - 310) * 100);
      // cv::putText(select_img, select_text, point, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
      // // cv::putText(select_img, select_text, point, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, scoreColor);
      //todo
    }
  }
  //todo 3.24
  // cv::imshow("select_img", select_img);
  // cv::waitKey(2);
  // cv::imwrite("/home/xuwenyu/00/test/" + to_string(select_time) + "_select.png", select_img);
  //todo 3.24
  return blacklist;
}

cv::Mat FeatureSelector::getSelectImg()
{
  return select_img;
}

// ----------------------------------------------------------------------------

std::map<double, int, std::greater<double>> FeatureSelector::sortLogDetUB(const omega_horizon_t& Omega, const omega_horizon_t& OmegaS,
                                                           const std::map<int, omega_horizon_t>& Delta_ells, const std::vector<int>& blackList, 
                                                           const image_t& image)
{
  // returns a descending sorted map with upper bound as the first key,
  // and feature id as the value for all features in image
  std::map<double, int, std::greater<double>> UBs;// <UB, featureId> descending by UB

    // Partially create argument to UB function (see eq 9). The only thing
  // missing from this is the additive and expected information from the
  // l-th feature. Each is added one at a time (independently) to calc UB
  const omega_horizon_t M = Omega + OmegaS;


  // Find the upper bound of adding each Delta_ell to M independently
  for(const auto& fpair : Delta_ells)
  {
    int f_id = fpair.first;

    // if a feature was already selected, do not calc UB. Not including it
    // in the UBs prevents it from being selected again.
    bool in_blackList = std::find(blackList.begin(), blackList.end(), f_id) != blackList.end();

    if(in_blackList) continue;

    // find probability of this feature being tracked
    double p = image.at(f_id)[0].second.coeff(fPROB);
    double l = image.at(f_id)[0].second.coeff(5);//!  l
    // double u_vel = image.at(f_id)[0].second.coeff(fuVEL);//!  uVEL
    // double v_vel = image.at(f_id)[0].second.coeff(fuVEL);//!  vVEL
    // if(u_vel >= 10 || v_vel >= 10) continue;//todo VEL 12.4

    // construct the argument to the logdetUB function
    omega_horizon_t A = M + Delta_ells.at(f_id);//todo 这里用到了pro，可不可以再把life time加入(12.4)


    //todo
    // calculate upper bound (eq 32)
    // int row_index, col_index;
    // Eigen::EigenSolver<omega_horizon_t> es(M);
    // if (es.pseudoEigenvalueMatrix().minCoeff(&row_index, &col_index) < 0) ROS_ERROR_STREAM("M min eigen value < 0!");
    // double M_eigen_value = es.pseudoEigenvalueMatrix().minCoeff(&row_index, &col_index);
    // if (M_eigen_value < 0) continue;
    // double ub = M_eigen_value + (p * Delta_ells.at(f_id) * es.pseudoEigenvectors().block<STATE_SIZE*(HORIZON+1), 1>(0, col_index)).norm();
    //todo
    // calculate upper bound (eq 29)
    double ub = A.diagonal().array().log().sum();//这里计算的是实际成本的上界，更为快速

    if(std::isnan(ub)) continue;
    if(ub <= 0) continue;

    // store in map for automatic sorting (desc) and easy lookup
    UBs[ub] = f_id;        //!   p
  }

  return UBs;
} 

// ----------------------------------------------------------------------------

//todo 12.13 all in weight(prob * lifeTime * delta_ells) 
std::pair<std::vector<int>, std::vector<int>> FeatureSelector::allIn(image_t& image, const double& header, int num_odom, int stereo)
{

  static double frame_time_k = header;

  double delta_frame = header - frame_time_k;

  double delta_odom = delta_frame / num_odom;

  image_t image_new;
  splitFeatureId(last_feature_id_, image, image_new);//第一帧时image_new = image   image.zero()//todo

  if(!image_new.empty()) last_feature_id_ = image_new.crbegin()->first;//todo

  auto state_kkH = generateFutureHorizon();

  auto omega_kkH = calcInfoFromRobotMotion(state_kkH, num_odom, delta_odom);

  addOmegaPrior(omega_kkH);

  auto delta_ells = calcInfoFromFeature(image_new, state_kkH);

  //todo 12.20
  if(stereo)
  {
    auto delta_ells_right = calcInfoFromRightFeature(image_new, state_kkH);

    for(auto & delta : delta_ells)
    {
      if(delta_ells_right.find(delta.first) != delta_ells_right.end())
      {
        delta.second += delta_ells_right[delta.first];
      }
    }
  }
  //todo 12.20

  cout << "delta_ells : " << delta_ells.size() << endl;//todo

  // so we can keep track of the new features we chose
  std::vector<int> selectedIds;
  selectedIds.reserve(max_num_);

  std::map<double, int, std::greater<double>> UBs;

  // uniform_real_distribution<double> u(0, 1);
	// default_random_engine e;

  for (const auto& fpair : delta_ells)
  {
    int f_id = fpair.first;

    double p = image_new.at(f_id)[0].second.coeff(fPROB);
    double l = image_new.at(f_id)[0].second.coeff(5);//!  l

    // omega_horizon_t A = omega_kkH + l * p * delta_ells.at(f_id);//todo
    omega_horizon_t A = omega_kkH + l * p * delta_ells.at(f_id);//todo



    // double ub = A.diagonal().array().log().sum();//这里计算的是实际成本的上界，更为快速
    double ub = Utility::logdet(A, true);//todo 与up bounds差不多
    // if(std::isnan(ub)) continue;
    // double ub = l * p;//todo

    cout << "ub = " << ub << endl;
    // cout << "u(e) = " << u(e) << endl;

    UBs[ub] = f_id;
  }

  int weight_max = 1;
  for(auto i : UBs)
  {
    if(weight_max == max_num_) break;
    int fid = i.second;
    selectedIds.push_back(fid);
    weight_max++;
    
  }
  
  cout << "selectedIds.size : " << selectedIds.size() << endl;//todo
  cout << "tracked_features_.size : " << tracked_features_.size() << endl;//todo

  tracked_features_.insert(tracked_features_.end(), selectedIds.begin(), selectedIds.end());
  

  frame_time_k = header;

  return std::make_pair(tracked_features_, selectedIds);
}

// ----------------------------------------------------------------------------

std::pair<std::vector<int>, std::vector<int>> FeatureSelector::roulette(image_t& image, const double& header, int num_odom, int stereo)
{

  static double frame_time_k = header;

  double delta_frame = header - frame_time_k;

  double delta_odom = delta_frame / num_odom;

  image_t image_new = image;
  splitFeatureId(last_feature_id_, image, image_new);//第一帧时image_new = image   image.zero()//todo

  if(!image_new.empty()) last_feature_id_ = image_new.crbegin()->first;//todo

  auto state_kkH = generateFutureHorizon();

  auto omega_kkH = calcInfoFromRobotMotion(state_kkH, num_odom, delta_odom);

  addOmegaPrior(omega_kkH);

  auto delta_ells = calcInfoFromFeature(image_new, state_kkH);
  
  cout << "delta_ells : " << delta_ells.size() << endl;//todo

  // so we can keep track of the new features we chose
  std::vector<int> selectedIds;
  selectedIds.reserve(max_num_);

  
  std::map<double, int, std::greater<double>> UBs;
  std::map<double, int, std::greater<double>> UBs_right;
  double sum = 0;
  double sum_right = 0;

  //todo https://blog.csdn.net/xinwenhuayu/article/details/99708993
  uniform_real_distribution<double> u(0, 1);
	default_random_engine e;

  for (const auto& fpair : delta_ells)
  {
    int f_id = fpair.first;

    double p = image_new.at(f_id)[0].second.coeff(fPROB);
    double l = image_new.at(f_id)[0].second.coeff(5);//!  l

    omega_horizon_t A = omega_kkH + l * p * delta_ells.at(f_id);//todo
    // omega_horizon_t A = omega_kkH + delta_ells.at(f_id);
    
    // double ub = A.diagonal().array().log().sum();//这里计算的是实际成本的上界，更为快速
    double ub = Utility::logdet(A, true);//todo 与up bounds差不多

    if(std::isnan(ub)) continue;
    if(ub <= 0) continue;
    // cout << "ub = " << ub << endl;

    // double ub = l * p;//todo

    UBs[ub] = f_id;
    sum += ub;

  }

  if(stereo)
  {
    auto delta_ells_right = calcInfoFromRightFeature(image_new, state_kkH);//todo 

    for (const auto& fpair : delta_ells_right)
    {
      int f_id = fpair.first;

      double p = image_new.at(f_id)[0].second.coeff(fPROB);
      double l = image_new.at(f_id)[0].second.coeff(5);//!  l

      omega_horizon_t A = omega_kkH + l * p * delta_ells_right.at(f_id);//todo
      // omega_horizon_t A = omega_kkH + delta_ells.at(f_id);
      
      // double ub = A.diagonal().array().log().sum();//这里计算的是实际成本的上界，更为快速
      double ub = Utility::logdet(A, true);//todo 与up bounds差不多

      if(std::isnan(ub)) continue;
      if(ub <= 0) continue;
      // cout << "ub2 = " << ub << endl;

      // double ub = l * p;//todo

      UBs_right[ub] = f_id;
      sum_right += ub;
    }
  }


  cout << "sum = " << sum << endl;
  cout << "sum_right = " << sum_right << endl;

  if(sum > sum_right)
  {
    if(UBs.size() <= 30) return std::make_pair(tracked_features_, selectedIds);//todo  12.19

    double weight_max = 1;
    double n = (UBs.size() <= max_num_) ? (UBs.size() - 30) : max_num_;
    while(weight_max < n)
    {

      double c = u(e);
      for(auto i : UBs)
      {
        double occ = i.first / sum;
        // cout << "occ = " << occ << endl;
        if(c >= occ)
        {
          selectedIds.push_back(i.second);
          UBs.erase(i.first);
          weight_max++;
          break;
        }
        else
        {
          continue;
        }
      }
    }
  }
  else
  {
    if(UBs_right.size() <= 30) return std::make_pair(tracked_features_, selectedIds);//todo  12.19

    double weight_max = 1;
    double n = (UBs_right.size() <= max_num_) ? (UBs_right.size() - 30) : max_num_;
    while(weight_max < n)
    {

      double c = u(e);
      for(auto i : UBs_right)
      {
        double occ = i.first / sum_right;
        // cout << "occ = " << occ << endl;
        if(c >= occ)
        {
          selectedIds.push_back(i.second);
          UBs_right.erase(i.first);
          weight_max++;
          break;
        }
        else
        {
          continue;
        }
      }
    }
  }

  cout << "selectedIds.size : " << selectedIds.size() << endl;//todo
  cout << "tracked_features_.size : " << tracked_features_.size() << endl;//todo

  tracked_features_.insert(tracked_features_.end(), selectedIds.begin(), selectedIds.end());
  
  frame_time_k = header;

  return std::make_pair(tracked_features_, selectedIds);
}
