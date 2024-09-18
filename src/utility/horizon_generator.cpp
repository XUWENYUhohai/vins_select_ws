#include "horizon_generator.h"

HorizonGenerator::HorizonGenerator()
{
    // pub_horizon_ = nh_.advertise<nav_msgs::Path>("horizon", 10);
}

//todo no use
state_horizon_t HorizonGenerator::odom(const state_t& state_0, const state_t& state_1, const Eigen::Vector3d& a, 
                                        const Eigen::Vector3d& w, const int OdomMeasurements, const double delta)
{
    state_horizon_t state_kkH;

    // initialize with the the last frame's pose (tail of optimizer window)
    state_kkH[0] = state_0;

    // Additionally, since we already have an IMU propagated estimate
    // (yet-to-be-corrected) of the current frame, we will start there.
    state_kkH[1] = state_1;

    // let's just assume constant bias over the horizon    
    auto Ba = state_kkH[0].first.segment<3>(xB_A);

    // we also assume a constant angular velocity during the horizon
    auto Q = Utility::deltaQ(w * delta);

    for(int h = 2; h <= HORIZON; h++)// NOTE: we already have k+1.
    {
        // use the prev frame state to initialize the k+h frame state
        state_kkH[h] = state_kkH[h-1];

        for (int i = 0; i < OdomMeasurements; i++)
        {
            // propagate attitude with incremental IMU update
            state_kkH[h].second = state_1.second * Q;//机器人坐标系转到世界系

            // Convenience: quat from world to current IMU-rate body pose
            const auto& q_hi = state_kkH[h].second;

            // vi, eq (11)
            // state_kkH[h].first.segment<3>(xVEL) += (gravity + q_hi * (a - Ba)) * delta;
            state_kkH[h].first.segment<3>(xVEL) += (q_hi * (a - Ba)) * delta;

            // ti, second equality in eq (12)
            // state_kkH[h].first.segment<3>(xPOS) += state_kkH[h].first.segment<3>(xVEL) * delta + 0.5 * (gravity + q_hi * (a - Ba)) * delta * delta;
            state_kkH[h].first.segment<3>(xPOS) += state_kkH[h].first.segment<3>(xVEL) * delta + 0.5 * (q_hi * (a - Ba)) * delta * delta;
        }
    }

    return state_kkH;
}


state_horizon_t HorizonGenerator::groundTruth(const state_t& state_0, const state_t& state_1)
{
    state_horizon_t state_kkH;

    // initialize state horizon structure with [xk]. The rest are future states.
    state_kkH[0].first = state_0.first;
    state_kkH[0].second = state_0.second;
    // state_kkH[0] = state_0;

    state_kkH[1].first = state_1.first;
    state_kkH[1].second = state_1.second;

    // Ground truth orientation of frame k+h w.r.t. orientation of frame k+h-1
    auto rel_q = state_kkH[0].second.inverse() * state_kkH[1].second;
    // Ground truth position of frame k+h w.r.t. position of frame k+h-1
    auto rel_p = state_kkH[1].second.inverse() * (state_kkH[1].first.segment<3>(xPOS) - state_kkH[0].first.segment<3>(xPOS));

    // predict pose of camera for frames k to k+H
    for(int h = 2; h <= HORIZON; h++)
    {
        // "predict" where the current frame in the horizon (k+h)
        // will be by applying this relative rotation to
        // the previous frame (k+h-1)
        state_kkH[h].first.segment<3>(xPOS) = state_kkH[h - 1].first.segment<3>(xPOS) + state_kkH[h - 1].second * rel_p;
        state_kkH[h].second = state_kkH[h - 1].second * rel_q;
    }

    return state_kkH;
}

// ----------------------------------------------------------------------------

void HorizonGenerator::visualize(const std_msgs::Header& header, const state_horizon_t& x_kkH)
{
    // Don't waste cycles unnecessarily
  if (pub_horizon_.getNumSubscribers() == 0) return;

  nav_msgs::Path path;
  path.header = header;
  path.header.frame_id = "world";

  // include the current state, xk (i.e., h=0)
  for (int h = 0; h < HORIZON; h++)
  {
    // for convenience
    const auto x_h = x_kkH[h].first;
    const auto q_h = x_kkH[h].second;

    // Compose world-to-imu estimate with imu-to-cam extrinsic transform  //todo 这里应该是从 惯导在世界系下坐标 转到 相机在世界系下坐标
    Eigen::Vector3d P = x_h.segment<3>(xPOS) + q_h * TIC[0];
    Eigen::Quaterniond Q = q_h * Eigen::Quaterniond(RIC[0]);

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = P.x();
    pose.pose.position.y = P.y();
    pose.pose.position.z = P.z();
    pose.pose.orientation.w = Q.w();
    pose.pose.orientation.x = Q.x();
    pose.pose.orientation.y = Q.y();
    pose.pose.orientation.z = Q.z();

    path.poses.push_back(pose);
  }

  pub_horizon_.publish(path);
}