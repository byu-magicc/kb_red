#include "vserv/vserv.h"

namespace vserv
{

VSERV::VSERV() :
  nh_(""),
  nh_private_("~")
{
  // retrieve parameters from ROS parameter server
  common::rosImportScalar<double>(nh_, "fx", fx_);
  common::rosImportMatrix<double>(nh_, "P0", P_);

  // other parameters and constants
  t_prev_ = 0;

  // set up ROS subscribers
  image_sub_ = nh_.subscribe("/image_raw", 1, &VSERV::imageCallback, this);

  // set up ROS publishers
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist", 1);
}


void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // convert message data into OpenCV type cv::Mat
  cv::Mat img;
  try
  {
    img = cv_bridge::toCvCopy(data, "mono8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", data->encoding.c_str());
  }
}


void EKF::publishState(double u)
{
  kb_autopilot::State msg;
  msg.p_north = x_(0); // north position (m)
  msg.p_east =  x_(1); // east position (m)
  msg.psi =     x_(2); // unwrapped yaw angle (rad)
  msg.b_r =     x_(3); // heading rate bias
  msg.b_u =     x_(4); // velocity bias
  msg.u =       u;     // body fixed forward velocity (m/s)
  
  msg.psi_deg = wrapAngle(x_(2))*180/M_PI; // unwrapped yaw angle (deg)
  state_pub_.publish(msg);
}


} // namespace ekf
