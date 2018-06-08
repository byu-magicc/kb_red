#ifndef VSERV_H
#define VSERV_H

#include <iostream>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <geometry_msgs/TwistStamped.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

// Jerel's stuff
#include "common_ros/common_ros.h"
#include "common_cpp/common.h"


namespace vserv
{

class VSERV
{

public:

  VSERV();

private:

  // ROS
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber image_sub_;
  ros::Publisher twist_pub_;

  // additional variables
  double t_prev_;
  cv::Ptr<cv::Feature2D> detector_, descriptor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  std::vector<cv::KeyPoint> kf_pts_; // keframe points
  cv::Mat kf_img_, kf_desc_; // keframe image and descriptors

  // functions
  void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg);

};


} // namespace vserv

#endif // VSERV_H
