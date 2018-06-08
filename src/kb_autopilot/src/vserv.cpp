#include "vserv/vserv.h"

namespace vserv
{

VSERV::VSERV() :
  nh_(""),
  nh_private_("~")
{
  // retrieve parameters from ROS parameter server
  // common::rosImportScalar<double>(nh_, "fx", fx_);
  // common::rosImportMatrix<double>(nh_, "P0", P_);

  // other parameters and constants
  t_prev_ = 0;
  new_keyframe_ = true;
  klt_win_size_ = cv::Size(31,31);
  klt_max_level_ = 2;
  klt_term_crit_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

  // instantiate feature detector
  detector_ = cv::GFTTDetector::create(1000,0.01,30,3,false,0.04);

  // set up ROS subscribers
  image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &VSERV::imageCallback, this);

  // set up ROS publishers
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist", 1);
}


void VSERV::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
  // convert message data into OpenCV type cv::Mat
  cv::Mat img;
  try
  {
    img = cv_bridge::toCvCopy(msg, "mono8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    // ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    ROS_ERROR("cv_bridge couldn't convert image message to OpenCV.");
  }

  // TODO: need to populate keyframe image at some point
  // just use previous image for testing
  if (new_keyframe_) {
    img.copyTo(kf_img_);
    std::vector<cv::KeyPoint> kfpts;
    detector_->detect(kf_img_,kfpts,cv::noArray());
    cv::KeyPoint::convert(kfpts,kf_pts_);
    new_keyframe_ = false;
  }

  // calculate the optical flow
  std::vector<cv::Point2f> pts;
  std::vector<uchar> flow_inlier;
  cv::calcOpticalFlowPyrLK(kf_img_, img, kf_pts_, pts, flow_inlier, cv::noArray(), klt_win_size_, klt_max_level_, klt_term_crit_);

  // collect the matched features and their indices
  std::vector<cv::Point2f> matches1,matches2;
  std::vector<int> klt_matches_idx;
  for (int i = 0; i < flow_inlier.size(); i++)
  {
    if (flow_inlier[i])
    {
      matches1.push_back(pts[i]);
      matches2.push_back(kf_pts_[i]);
    }
  }

  // filter the bad matches
  cv::Mat inlier_mask, homography, F;
  std::vector<cv::Point2f> good_matches1, good_matches2;
  if(matches1.size() >= 8) {
    // homography = cv::findHomography(matches1, matches2, CV_RANSAC, 2, inlier_mask);
    F = cv::findFundamentalMat(matches1, matches2, CV_FM_RANSAC, 3, 0.99, inlier_mask);
    for(unsigned i = 0; i < matches1.size(); i++) {
      if(inlier_mask.at<uchar>(i)) {
        good_matches1.push_back(matches1[i]);
        good_matches2.push_back(matches2[i]);
      }
    }

    // create new keyframe if too few inliers
    if (good_matches1.size() < 30) {
      new_keyframe_ = true;
    }

    // build image jacobian
    
    // compute pixel velocities
    
    // calculate camera velocities

    // build and publish the twist
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = msg->header.stamp;
    twist_msg.twist.linear.x = 0;
    twist_msg.twist.linear.y = 0;
    twist_msg.twist.linear.z = 0;
    twist_msg.twist.angular.x = 0;
    twist_msg.twist.angular.y = 0;
    twist_msg.twist.angular.z = 0;
    twist_pub_.publish(twist_msg);

    // convert image to color
    cv::Mat img_plot = img.clone();
    cv::cvtColor(img, img_plot, cv::COLOR_GRAY2BGR, 3);
    for (int i = 0; i < good_matches1.size(); i++)
    {
      cv::circle(img_plot, good_matches1[i], 5, cv::Scalar(0,255,0), 1);
    }
    cv::namedWindow("Tracked Features", CV_WINDOW_AUTOSIZE);
    cv::imshow("Tracked Features", img_plot);
    cv::waitKey(1);
  }
  else {
    ROS_WARN("Too few matches for fundamental matrix. New keyframe!\n");
    new_keyframe_ = true;
  }
}


} // namespace ekf
