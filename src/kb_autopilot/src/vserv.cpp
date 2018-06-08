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

  // instantiate feature detector
  detector_ = cv::GFTTDetector::create(1000,0.001,30,3,false,0.04);
  descriptor_ = cv::ORB::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

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
    ROS_ERROR("Couldn't convert image.");
  }

  // TODO: need to populate keyframe image at some point
  // just use previous image for testing
  if (kf_img_.empty()) {
    img.copyTo(kf_img_);
    detector_->detect(kf_img_,kf_pts_,cv::noArray());
    descriptor_->compute(kf_img_,kf_pts_,kf_desc_);
  }

  // collect keypoints and descriptors of current image
  std::vector<cv::KeyPoint> pts;
  cv::Mat desc;
  detector_->detect(img,pts,cv::noArray());
  descriptor_->compute(img,pts,desc);

  // find matches between current image and keyframe image
  std::vector<cv::DMatch> matches;
  std::vector<cv::KeyPoint> matched1, matched2;
  matcher_->match(kf_desc_, desc, matches);
  for (int i = 0; i < kf_desc_.rows; i++) {
    if (matches[i].distance <= 10.0) {
      matched1.push_back(kf_pts_[matches[i].queryIdx]);
      matched2.push_back(    pts[matches[i].trainIdx]);
    }
  }

  // // find matches between current image and keyframe image
  // std::vector< std::vector<cv::DMatch> > matches;
  // std::vector<cv::KeyPoint> matched1, matched2;
  // matcher_->knnMatch(kf_desc_, desc, matches, 2);
  // for(unsigned i = 0; i < matches.size(); i++) {
  //   if(matches[i][0].distance < 0.8 * matches[i][1].distance) {
  //     matched1.push_back(kf_pts_[matches[i][0].queryIdx]);
  //     matched2.push_back(    pts[matches[i][0].trainIdx]);
  //   }
  // }

  // filter the bad matches
  cv::Mat inlier_mask, homography;
  std::vector<cv::KeyPoint> inliers1, inliers2;
  std::vector<cv::Point2f> matched1_pt2f, matched2_pt2f;
  std::vector<cv::DMatch> inlier_matches;
  if(matched1.size() >= 4) {
    cv::KeyPoint::convert(matched1, matched1_pt2f);
    cv::KeyPoint::convert(matched2, matched2_pt2f);
    homography = cv::findHomography(matched1_pt2f, matched2_pt2f, CV_RANSAC, 5, inlier_mask);
    for(unsigned i = 0; i < matched1.size(); i++) {
      if(inlier_mask.at<uchar>(i)) {
        int new_i = static_cast<int>(inliers1.size());
        inliers1.push_back(matched1[i]);
        inliers2.push_back(matched2[i]);
        inlier_matches.push_back(cv::DMatch(new_i, new_i, 0));
      }
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

    cv::Mat img_draw;
    cv::drawMatches(img,pts,kf_img_,kf_pts_,inlier_matches,img_draw,cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::namedWindow("matches", CV_WINDOW_AUTOSIZE);
    cv::imshow("matches", img_draw);
    cv::waitKey(1);
  }
  else {
    ROS_WARN("Too few matches for twist calculation.\n");
  }

  // store info. of current image for next iteration
  img.copyTo(kf_img_);
  detector_->detect(kf_img_,kf_pts_,cv::noArray());
  descriptor_->compute(kf_img_,kf_pts_,kf_desc_);
}


} // namespace ekf
