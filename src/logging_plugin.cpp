// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#include "raisin_logging_plugin/logging_plugin.hpp"

namespace raisin
{

namespace plugin
{

LoggingPlugin::LoggingPlugin(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
: Node("raisin_logging_plugin", globalResource.network), Plugin(world, server, worldSim, serverSim, globalResource)
{
  pluginType_ = PluginType::CUSTOM;

  imuSubscriber_ = createSubscriber<raisin::sensor_msgs::msg::Imu>("base_imu/imu", nullptr,
    [this](const raisin::sensor_msgs::msg::Imu::SharedPtr msg) { this->imuCallback(msg); });

  jointStatesSubscriber_ = createSubscriber<raisin_interfaces::msg::JointStates>("joint_states", nullptr,
    [this](const raisin_interfaces::msg::JointStates::SharedPtr msg) { this->jointStatesCallback(msg); });

  auto connections =
        raisin::GlobalResource::getInstance().network->getConnection("raisin_vicon_node");
      

  viconSubscriber_ = createSubscriber<raisin_interfaces::msg::Pose>("base_link", connections,
    [this](const raisin_interfaces::msg::Pose::SharedPtr msg) { this->viconCallback(msg); });
}

LoggingPlugin::~LoggingPlugin()
{
  cleanupResources();
}

bool LoggingPlugin::init()
{
  if (!profileInitialized_) {
    imuCallbackProfile_ = Profiler::getInstance().registerMeasurement("logging/imu_callback");
    profileInitialized_ = true;
  }

  logIdx_ = dataLogger_.initializeAnotherDataGroup(
      "logging_gt",
      "tick", tick_elapsed,
      "angVel", measuredAngularVelocity_,
      "linAcc", measuredLinearAcceleration_,
      "jointAng", jointPosition_,
      "jointVel", jointVelocity_,
      "prevTarget", jointTargetPrev_,
      "genForce", generalizedForce_,
      "viconPos", viconPos,
      "viconQuat", viconQuat
      );
  return true;
}

bool LoggingPlugin::advance()
{
  robotHub_->lockMutex();
  robotHub_->getPdTarget(posTarget_, velTarget_);
  generalizedForce_ = robotHub_->getGeneralizedForce().e().tail(12);
  robotHub_->unlockMutex();

  dataLogger_.append(
      logIdx_,
      tick_elapsed++,
      measuredAngularVelocity_,
      measuredLinearAcceleration_,
      jointPosition_,
      jointVelocity_,
      jointTargetPrev_,
      generalizedForce_,
      viconPos,
      viconQuat
      );

  jointTargetPrev_ = posTarget_.tail(12);

  return true;
}

bool LoggingPlugin::reset()
{
  tick_elapsed = 0;
  return true;
}

void LoggingPlugin::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  ScopeTimer timer(&Profiler::getInstance(), imuCallbackProfile_);

  if (!msg) { return; }

  const double currentImuTime = toSeconds(msg->header.stamp);
  if (!std::isfinite(currentImuTime) || currentImuTime <= 0.0) { return; }

  std::scoped_lock lock(imuMutex_);
  measuredAngularVelocity_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  measuredLinearAcceleration_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
}

void LoggingPlugin::jointStatesCallback(const raisin_interfaces::msg::JointStates::SharedPtr msg) {
  std::scoped_lock lock(jointStatesMutex_);
  for (int i = 0; i < 12; ++i) {
    jointPosition_(i) = msg->joint_states[i].joint_position;
    jointVelocity_(i) = msg->joint_states[i].joint_velocity;
  }
}

void LoggingPlugin::viconCallback(const raisin_interfaces::msg::Pose::SharedPtr msg) {
  viconPos[0] = msg->position[0];
  viconPos[1] = msg->position[1];
  viconPos[2] = msg->position[2];
  viconQuat[0] = msg->quaternion[0];
  viconQuat[1] = msg->quaternion[1];
  viconQuat[2] = msg->quaternion[2];
  viconQuat[3] = msg->quaternion[3];
}

extern "C" Plugin * create(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
{
  return new LoggingPlugin(world, server, worldSim, serverSim, globalResource);
}

extern "C" void destroy(LoggingPlugin * p)
{
  delete p;
}

} // namespace plugin

} // namespace raisin
