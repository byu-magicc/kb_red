#include "ekf/ekf.h"

namespace ekf
{

EKF::EKF() :
  nh_(""),
  nh_private_("~")
{
  // retrieve parameters from ROS parameter server
  common::rosImportScalar<int>(nh_, "car_length", car_length, 0.179);
  common::rosImportScalar<int>(nh_, "rmin_left", rmin_left, 0.7);
  common::rosImportScalar<int>(nh_, "rmin_right", rmin_right, 0.7);
  t_prev_ = 0;

  // set up ROS subscribers
  message_filters::Subscriber<Image> cmd_sub(nh, "/safety_pilot", 1);
  message_filters::Subscriber<Image> enc_sub(nh, "/encoder", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cmd_sub, enc_sub);
  sync.registerCallback(std::bind(&propCallback, _1, _2));

  // set up ROS publishers
  state_pub_ = nh_.advertise<State>("state", 1);

  // state, covariance, process/input noise covariance, measurement covariance
  x_.setZero();
  P_ = 1e-3*Eigen::MatrixXd::Identity(8,8);
  Qu_ = 1e-3*Eigen::MatrixXd::Identity(8,8)
  Qx_ = 1e-3*Eigen::MatrixXd::Identity(8,8);
  B_.setZero();
  B_(7,0) = 1;
  B_(3,1) = 1;
}


void propCallback(const Command& msg, const Encoder& msg)
{
  // get time step
  t_now = msg.header.stamp.toSec();
  double dt = t_now - t_prev_;
  t_prev_ = t_now;
}


// void update()
// {
//   //
// }


} // namespace ekf
