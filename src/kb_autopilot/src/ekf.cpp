#include "ekf/ekf.h"

namespace ekf
{

EKF::EKF() :
  nh_(""),
  nh_private_("~"),
  cmd_sub_(nh_, "/safety_pilot", 1),
  enc_sub_(nh_, "/encoder", 1),
  sync_(MySyncPolicy(10), cmd_sub_, enc_sub_)
{
  // retrieve parameters from ROS parameter server
  rosImportMatrix<double>(nh_, "x0", x_);
  rosImportMatrix<double>(nh_, "P0", P_);
  rosImportMatrix<double>(nh_, "Qx", Qx_);
  rosImportMatrix<double>(nh_, "Qu", Qu_);
  rosImportMatrix<double>(nh_, "R_pose", R_pose_);
  rosImportScalar<double>(nh_, "R_v", R_v_, 1);
  rosImportMatrix<double>(nh_, "lambda", lambda_);
  Eigen::Matrix<double,8,1> onevec; onevec.setOnes();
  Lambda_ = onevec*lambda_.transpose()+lambda_*onevec.transpose()-lambda_*lambda_.transpose();

  // other parameters and constants
  t_prev_ = 0;
  B_.setZero();
  B_(7,0) = 1;
  B_(3,1) = 1;
  H_v_.setZero();
  H_v_(3) = 1;
  H_pose_.setZero();
  H_pose_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);

  // set up ROS subscribers
  sync_.registerCallback(std::bind(&EKF::propCallback, this, std::placeholders::_1, std::placeholders::_2));
  pose_sub_ = nh_.subscribe("/pose", 1, &EKF::update, this);

  // set up ROS publishers
  state_pub_ = nh_.advertise<kb_autopilot::State>("/ekf_state", 1);
}


void EKF::propCallback(const kb_utils::Servo_CommandConstPtr& servo_msg, const kb_utils::EncoderConstPtr& encoder_msg)
{
  // get time step
  double t_now = servo_msg->header.stamp.toSec();
  double dt = t_now - t_prev_;
  t_prev_ = t_now;

  // collect steering signal and measure velocity
  double s = servo_msg->steer; // actually comes in as an integer
  double v_meas = encoder_msg->vel;

  // unpack state
  double pn  = x_(0);
  double pe  = x_(1);
  double psi = x_(2);
  double v   = x_(3);
  double bv  = x_(4);
  double L   = x_(5);
  double a   = x_(6);
  double bs  = x_(7);

  // first update the velocity estimate with its measurement
  Eigen::Matrix<double,8,1> K = P_*H_v_.transpose()/(H_v_*P_*H_v_.transpose()+R_v_);
  x_ += lambda_.cwiseProduct(K*(v_meas - v));
  P_ -= Lambda_.cwiseProduct(K*H_v_*P_);

  // state kinematics
  Eigen::Matrix<double, 8, 1> f;
  f.setZero();
  f(0) = (v+bv)*cos(psi);
  f(1) = (v+bv)*sin(psi);
  f(2) = (v+bv)/L*tan(a*s+bs);

  // covariance kinematics
  Eigen::Matrix<double, 8, 8> A;
  A.setZero();
  A(0,2) = -(v+bv)*sin(psi);
  A(0,3) = cos(psi);
  A(0,4) = cos(psi);
  A(1,2) = (v+bv)*cos(psi);
  A(1,3) = sin(psi);
  A(1,4) = sin(psi);
  A(2,3) = tan(a*s+bs)/L;
  A(2,4) = tan(a*s+bs)/L;
  A(2,5) = -(v+bv)/L/L*tan(a*s+bs);
  A(2,6) = s*(v+bv)/L/cos(a*s+bs)/cos(a*s+bs);
  A(2,7) = (v+bv)/L/cos(a*s+bs)/cos(a*s+bs);

  // propagate the state
  x_ += f*dt;
  P_ = (A*P_+P_*A.transpose()+B_*Qu_*B_.transpose()+Qx_)*dt;

  // publish the current state
  publishState();
}


void EKF::update(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  // unpack measurement and estimated measurement
  Eigen::Vector3d z(msg->vector.x,msg->vector.y,msg->vector.z);
  Eigen::Vector3d hx = x_.segment<3>(0);

  // apply update
  Eigen::Matrix<double,8,3> K = P_*H_pose_.transpose()*(H_pose_*P_*H_pose_.transpose()+R_pose_).inverse();
  x_ += lambda_.cwiseProduct(K*(z-hx));
  P_ -= Lambda_.cwiseProduct(K*H_pose_*P_);
}


void EKF::publishState()
{
  kb_autopilot::State msg;
  msg.p_north = x_(0); // north position (m)
  msg.p_east =  x_(1); // east position (m)
  msg.psi =     x_(2); // unwrapped yaw angle (rad)
  msg.u =       x_(3); // body fixed forward velocity (m/s)
  msg.b_u =     x_(4); // velocity bias
  msg.L =       x_(5); // length between axles on the vehicle
  msg.a =       x_(6); // servo to steering angle scale
  msg.b_s =     x_(7); // servo steering angle bias
  msg.psi_deg = x_(2)*180/M_PI; // unwrapped yaw angle (deg)
  state_pub_.publish(msg);
}


} // namespace ekf
