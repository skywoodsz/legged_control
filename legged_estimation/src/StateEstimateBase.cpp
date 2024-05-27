//
// Created by qiayuan on 2021/11/15.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged {
using namespace legged_robot;

StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      pinocchioBodyInterface_(pinocchioInterface_),
      info_(std::move(info)),
      eeKinematics_(eeKinematics.clone()),
      eeBodyKinematics_(eeKinematics.clone()),
      rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum)) {
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));

  imu_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(nh, "/dog/imu_data", 10));

  leg_pub_.reset(new realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>(nh, "/dog/leg_state", 10));
}

void StateEstimateBase::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel) {
  rbdState_.segment<3>(0) = zyx;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom) {
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    lastPub_ = time;
    if (odomPub_->trylock()) {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}

void StateEstimateBase::updateBodyKinematics(){
  const auto& model = pinocchioBodyInterface_.getModel();
  auto& data = pinocchioBodyInterface_.getData();

  size_t actuatedDofNum = info_.actuatedDofNum;

  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3),
      rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);
}

void StateEstimateBase::publishImuMsgs(const ros::Time& time, const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, 
    const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance, 
    const matrix3_t& linearAccelCovariance){
  if(imu_pub_->trylock()){
    imu_pub_->msg_.header.frame_id = "unitree_imu";
    imu_pub_->msg_.header.stamp = time;

    imu_pub_->msg_.angular_velocity.x = angularVelLocal(0);
    imu_pub_->msg_.angular_velocity.y = angularVelLocal(1);
    imu_pub_->msg_.angular_velocity.z = angularVelLocal(2);
    imu_pub_->msg_.linear_acceleration.x = linearAccelLocal(0);
    imu_pub_->msg_.linear_acceleration.y = linearAccelLocal(1);
    imu_pub_->msg_.linear_acceleration.z = linearAccelLocal(2);
    imu_pub_->msg_.orientation.x = quat.x();
    imu_pub_->msg_.orientation.y = quat.y();
    imu_pub_->msg_.orientation.z = quat.z();
    imu_pub_->msg_.orientation.w = quat.w();

    //TODO: publish covariance

    imu_pub_->unlockAndPublish();
  }
}

void StateEstimateBase::publishLegStateMsgs(const ros::Time& time, vector_t optimizedInput){
  updateBodyKinematics();
  eeBodyKinematics_->setPinocchioInterface(pinocchioBodyInterface_);
  std::vector<vector3_t> eePos = eeBodyKinematics_->getPosition(vector_t());
  std::vector<vector3_t> eeVel = eeBodyKinematics_->getVelocity(vector_t(), vector_t());

  if(leg_pub_->trylock()){
    leg_pub_->msg_.header.frame_id = "base";
    leg_pub_->msg_.header.stamp = time;

    for (size_t i = 0; i < 4; i++)
    {
      leg_pub_->msg_.foot_contact[i].z = contactFlag_[i];
      leg_pub_->msg_.foot_force[i].x = optimizedInput[i*3];
      leg_pub_->msg_.foot_force[i].y = optimizedInput[i*3+1];
      leg_pub_->msg_.foot_force[i].z = optimizedInput[i*3+2];
      leg_pub_->msg_.bfoot_pos[i].x = eePos[i][0];
      leg_pub_->msg_.bfoot_pos[i].y = eePos[i][1];
      leg_pub_->msg_.bfoot_pos[i].z = eePos[i][2];
      leg_pub_->msg_.bfoot_vel[i].x = eeVel[i][0];
      leg_pub_->msg_.bfoot_vel[i].y = eeVel[i][1];
      leg_pub_->msg_.bfoot_vel[i].z = eeVel[i][2];
    }
    leg_pub_->unlockAndPublish();
  }
}

}  // namespace legged
