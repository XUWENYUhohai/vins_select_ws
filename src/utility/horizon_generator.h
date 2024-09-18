#pragma once

#include "state_defs.h"
#include "utility.h"

#include <Eigen/Dense>
#include <ros/ros.h>
// #include <std_msgs/Header.h>
#include <nav_msgs/Path.h>

#include "../estimator/parameters.h"

class HorizonGenerator
{
public:
  HorizonGenerator();
  ~HorizonGenerator() = default;//https://blog.csdn.net/woshizuopie/article/details/125609904    https://blog.csdn.net/weixin_43598042/article/details/125316549
    

  /**
  * @brief      Generate horizon using constant acceleration IMU model
  *
  * @param[in]  state_0            The state of the prev frame (k)
  * @param[in]  state_1            The state of the current (yet-to-be-corrected) frame (k+1)
  * @param[in]  a                  Latest IMU acceleration measurement
  * @param[in]  w                  Latest IMU angular vel measurement
  * @param[in]  nrImuMeasurements  Number of IMU measurements from prev frame to now
  * @param[in]  delta             Sampling period of IMU measurements
  *
  * @return     a state horizon
  */
  state_horizon_t odom(const state_t& state_0, const state_t& state_1, const Eigen::Vector3d& a, const Eigen::Vector3d& w, const int OdomMeasurements, const double delta);
  //const和&的问题     ：  https://blog.csdn.net/qq_52572621/article/details/128228173


  //todo odom use for this
  /**
 * @brief      Generate horizon from ground truth data
 *
 * @param[in]  state_0     The state of the previous frame (k)
 * @param[in]  state_1     The state of the current (yet-to-be-corrected) frame (k+1)
 * @param[in]  deltaFrame  time between previous two img frames (secs)
 *
 * @return     a state horizon
 */
  state_horizon_t groundTruth(const state_t& state_0, const state_t& state_1);


  /**
  * @brief      Visualize a state propagated over a horizon
  *
  * @param[in]  header  The header message from the frame k (start)
  * @param[in]  x_kkH   The state horizon to visualize
  */
  void visualize(const std_msgs::Header& header, const state_horizon_t& x_kkH);

private:
  // ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher pub_horizon_;

  // extrinsic parameters: camera frame w.r.t imu frame
  // Eigen::Quaterniond q_IC_;
  // Eigen::Vector3d t_IC_;

};