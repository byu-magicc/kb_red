//
//  learn_node.cpp
//  
//
//  Created by Timothy Morris on 14/04/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include "relative_nav/Keyframe.h"
#include "relative_nav/Edge.h"
#include "relative_nav/NodeInfo.h"
#include "relative_nav/FilterState.h"
#include "relative_nav/GPS.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include "kb_autopilot/State.h"

class KeyframeFactory
{
  public:
    KeyframeFactory();
    void tick();
    ros::NodeHandle nh_;

  private:

    image_transport::ImageTransport it_;

    // publishers and subscribers
    image_transport::Subscriber rgb_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber ins_sub_;

    ros::Publisher kf_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher edge_pub_;
    ros::Publisher node_pub_;
    ros::Publisher is_flying_pub_;
    ros::Publisher relative_state_pub_;

    cv::Mat rgb_, depth_;
    cv_bridge::CvImagePtr cv_ptr_;
    
    relative_nav::Keyframe kf_msg;
    relative_nav::Edge edge_msg;
    relative_nav::FilterState relative_state_msg;
    relative_nav::NodeInfo node_msg;
    nav_msgs::Odometry kf_odom;
    nav_msgs::Odometry cur_odom;
    relative_nav::GPS cur_gps;
    
    bool got_rgb_;
    bool got_depth_;

    double min_dist_kf;
    bool is_flying_;

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void poseCallback(const nav_msgs::Odometry& msg);
    void stateCallback(const kb_autopilot::StateConstPtr& msg);
    void insCallback(const nav_msgs::OdometryConstPtr& msg);

    nav_msgs::Odometry odomDiff(nav_msgs::Odometry o1, nav_msgs::Odometry o2);
};

KeyframeFactory::KeyframeFactory()
  : nh_(ros::NodeHandle()), it_(nh_)
{
  rgb_sub_ = it_.subscribe("/camera/color/image_raw", 1, &KeyframeFactory::rgbCallback, this);
  depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &KeyframeFactory::depthCallback, this);
  pose_sub_ = nh_.subscribe("pose", 1, &KeyframeFactory::poseCallback, this);
  state_sub_ = nh_.subscribe("state", 1, &KeyframeFactory::stateCallback, this);

  kf_pub_ = nh_.advertise<relative_nav::Keyframe>("keyframe", 1);
  edge_pub_ = nh_.advertise<relative_nav::Edge>("edge", 1);
  node_pub_ = nh_.advertise<relative_nav::NodeInfo>("node", 1);
  gps_pub_ = nh_.advertise<relative_nav::GPS>("gps", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1, true);
  relative_state_pub_ = nh_.advertise<relative_nav::FilterState>("relative_state", 1);

  // Grab min distance of param
  nh_.param<double>("minDistKF", min_dist_kf, 0.5); 


  // Initialize first kf_pose
  kf_odom.pose.pose.position.x = 0;
  kf_odom.pose.pose.position.y = 0;
  kf_odom.pose.pose.position.z = 0;

  kf_odom.pose.pose.orientation.w = 1;
  kf_odom.pose.pose.orientation.x = 0;
  kf_odom.pose.pose.orientation.y = 0;
  kf_odom.pose.pose.orientation.z = 0;
  
  cur_gps.header.stamp = ros::Time::now();
  is_flying_ = false;
  got_rgb_ = false;
  got_depth_ = false;
}

void KeyframeFactory::insCallback(const nav_msgs::OdometryConstPtr &msg)
{
  cur_gps.header = msg->header;
  cur_gps.utm = msg->pose.pose.position;
  cur_gps.hAcc = 2.0;  
}

void KeyframeFactory::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    got_rgb_ = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[kf_factory %d] cv_bridge error: %s", __LINE__, e.what());  
    got_rgb_ = false;
  }
  rgb_ = cv_ptr_->image;
//  kf_msg.rgb = *msg;
}

void KeyframeFactory::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    got_depth_ = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[kf_factory %d] cv_bridge error: %s", __LINE__, e.what());  
    got_depth_ = false;
  }
  depth_ = cv_ptr_->image;
  
//  kf_msg.depth = *msg;
}

void KeyframeFactory::stateCallback(const kb_autopilot::StateConstPtr& msg)
{
  // Create an odometry message and call the pose callback
  nav_msgs::Odometry odom;
  odom.header = msg->header;
  odom.pose.pose.position.x = msg->p_north;
  odom.pose.pose.position.y = msg->p_east;
  odom.pose.pose.position.z = 0;
  
  double cs = std::cos(msg->psi/2.0);
  double ss = std::sin(msg->psi/2.0);
  
  odom.pose.pose.orientation.w = cs;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = ss;
  
  odom.twist.twist.linear.x = msg->u;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  poseCallback(odom);  
}

void KeyframeFactory::poseCallback(const nav_msgs::Odometry& msg)
{
  cur_odom = msg;
}

nav_msgs::Odometry KeyframeFactory::odomDiff(nav_msgs::Odometry o1, nav_msgs::Odometry o2)
{
  nav_msgs::Odometry d_o;
  geometry_msgs::Point p1, p2;
  geometry_msgs::Quaternion q1, q2;

  p1 = o1.pose.pose.position; p2 = o2.pose.pose.position;
  q2 = o1.pose.pose.orientation; q1 = o2.pose.pose.orientation;

  // Position difference
  d_o.pose.pose.position.x = p1.x - p2.x;
  d_o.pose.pose.position.y = p1.y - p2.y;
  d_o.pose.pose.position.z = p1.z - p2.z;

  // Orientation difference
  d_o.pose.pose.orientation.w = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
  d_o.pose.pose.orientation.x = -q1.w*q2.x + q1.x*q2.w - q1.y*q2.z + q1.z*q2.y;
  d_o.pose.pose.orientation.y = -q1.w*q2.y + q1.x*q2.z + q1.y*q2.w - q1.z*q2.x;
  d_o.pose.pose.orientation.z = -q1.w*q2.z - q1.x*q2.y + q1.y*q2.x + q1.z*q2.w;

  double norm = sqrt(pow(d_o.pose.pose.orientation.w,2) + pow(d_o.pose.pose.orientation.x,2) +
                     pow(d_o.pose.pose.orientation.y,2) + pow(d_o.pose.pose.orientation.z,2));

  d_o.pose.pose.orientation.w /= norm; 
  d_o.pose.pose.orientation.x /= norm; 
  d_o.pose.pose.orientation.y /= norm; 
  d_o.pose.pose.orientation.z /= norm; 

  return d_o;

}

void KeyframeFactory::tick()
{
  static int kf_id = 0;

  float d;
  nav_msgs::Odometry d_o;
  d = pow(cur_odom.pose.pose.position.x - kf_odom.pose.pose.position.x ,2) +
      pow(cur_odom.pose.pose.position.y - kf_odom.pose.pose.position.y ,2); 
  d = sqrt(d);

  // Use some bogus covariance, because Jerel's estimator is not relative
  for (int i = 0; i < 36; i += 7)
  {
    kf_odom.pose.covariance[i] = 1;
    cur_odom.pose.covariance[i] = 1;
  }

  // If we need to declare a new keyframe
  if (d > min_dist_kf && got_rgb_ && got_depth_)
  {
    // If this is the first keyframe, then tell the backend we are flying
    if (!is_flying_)
    {
      ROS_INFO("[kf_factory %d] FIRST KEYFRAME", __LINE__);
      is_flying_ = true;
      std_msgs::Bool bool_msg;
      bool_msg.data = true;
      is_flying_pub_.publish(bool_msg);
    }
    
    
    // Calculate the pose difference between current and last keyframe
    d_o = odomDiff(cur_odom, kf_odom);
    // Pack up edge message
    edge_msg.header.stamp = ros::Time::now();
    edge_msg.from_node_id = kf_id;
    edge_msg.to_node_id = kf_id + 1;
    edge_msg.transform.translation.x = d_o.pose.pose.position.x;
    edge_msg.transform.translation.y = d_o.pose.pose.position.y;
    edge_msg.transform.translation.z = d_o.pose.pose.position.z;
    edge_msg.transform.rotation = d_o.pose.pose.orientation;
    edge_msg.covariance = cur_odom.pose.covariance;
    // Publish edge
    edge_pub_.publish(edge_msg);
    
    // Pack up keyframe
    kf_msg.keyframe_id = kf_id;
    try
    {
      kf_msg.header.stamp = ros::Time::now();
      cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, rgb_).toImageMsg(kf_msg.rgb);
      cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, depth_).toImageMsg(kf_msg.depth);
      ROS_INFO("T1");
  
      // Publish keyframe
      kf_pub_.publish(kf_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[kf_factory %d] cv_bridge exception: %s", __LINE__, e.what());
    }

    // Increment keyframe
    kf_odom = cur_odom;
    kf_id++;
    
    // If we have a new GPS message, then publish it
    if ((ros::Time::now() - cur_gps.header.stamp).toSec() < 0.1)
    {
      gps_pub_.publish(cur_gps);
    }
  }
  
  // Pack up relative state
  d_o = odomDiff(cur_odom, kf_odom);
  relative_state_msg.header.stamp = ros::Time::now();
  relative_state_msg.node_id = kf_id;
  relative_state_msg.transform.translation.x = d_o.pose.pose.position.x;
  relative_state_msg.transform.translation.y = d_o.pose.pose.position.y;
  relative_state_msg.transform.translation.z = d_o.pose.pose.position.z;
  relative_state_msg.transform.rotation = d_o.pose.pose.orientation;
  relative_state_msg.velocity = cur_odom.twist.twist.linear;
  // Publish relative state
  relative_state_pub_.publish(relative_state_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyframe_factory_node");
  KeyframeFactory kf_factory;
	
	double sampleRate_;
	kf_factory.nh_.param<double>("sampleRate", sampleRate_, 10); 
	ros::Rate r(sampleRate_);
	
	ROS_INFO_STREAM("Keyframe sampling rate set to: " << sampleRate_ << "Hz");
	while (ros::ok())
	{
		ros::spinOnce();
    kf_factory.tick();
		r.sleep();
	}
	
	return 0;
}
