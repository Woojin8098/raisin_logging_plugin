// Copyright (c) 2025 Raion Robotics Inc.
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef RAISIN_LOGGING_PLUGIN_HPP_
#define RAISIN_LOGGING_PLUGIN_HPP_

#include "raisin_plugin/plugin.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "raisin_interfaces/msg/joint_states.hpp"
#include "raisin_util/raisin_msg_buffer.hpp"
#include "raisin_interfaces/msg/pose.hpp"

namespace raisin
{

namespace plugin
{

using JointStatesBuffer = raisin::util::MsgBuffer<raisin_interfaces::msg::JointStates>;

class LoggingPlugin : public Plugin, Node
{

public:
  LoggingPlugin(
    raisim::World & world, raisim::RaisimServer & server,
    raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource);
  ~LoggingPlugin();
  bool init() final;
  bool advance() final;
  bool reset() final;
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStatesCallback(const raisin_interfaces::msg::JointStates::SharedPtr msg);
  void viconCallback(const raisin_interfaces::msg::Pose::SharedPtr msg);

private:
  int tick_elapsed = 0;

  bool profileInitialized_{false};
  size_t imuCallbackProfile_{0};

  std::mutex imuMutex_;
  std::shared_ptr<raisin::Subscriber<raisin::sensor_msgs::msg::Imu>> imuSubscriber_;
  Eigen::Matrix<double, 3, 1> measuredAngularVelocity_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> measuredLinearAcceleration_ = Eigen::Matrix<double, 3, 1>::Zero();

  std::mutex jointStatesMutex_;
  std::shared_ptr<raisin::Subscriber<raisin_interfaces::msg::JointStates>> jointStatesSubscriber_;
  Eigen::Matrix<double, 12, 1> jointPosition_ = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> jointVelocity_ = Eigen::Matrix<double, 12, 1>::Zero();

  Eigen::VectorXd posTarget_ = Eigen::VectorXd::Zero(19);
  Eigen::VectorXd velTarget_ = Eigen::VectorXd::Zero(18);
  Eigen::Matrix<double, 12, 1> jointTargetPrev_ = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> generalizedForce_ = Eigen::Matrix<double, 12, 1>::Zero();

  std::shared_ptr<raisin::Subscriber<raisin_interfaces::msg::Pose>> viconSubscriber_;
  Eigen::Matrix<double, 3, 1> viconPos = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 4, 1> viconQuat = Eigen::Matrix<double, 4, 1>::Zero();
};

} // namespace plugin

} // namespace raisin

#endif // RAISIN_LOGGING_PLUGIN_HPP_
