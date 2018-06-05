#ifndef EKF_H
#define EKF_H

#include <iostream>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "kb_utils/Encoder.h"
#include "kb_utils/Servo_Command.h"

typedef message_filters::sync_policies::ApproximateTime<kb_utils::Servo_Command, kb_utils::Encoder> MySyncPolicy;


namespace ekf
{

class EKF
{

public:

  EKF();

private:

  // ROS
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher state_pub_;

  // EKF arrays
  Eigen::Matrix<double, 8, 1> x_;
  Eigen::Matrix<double, 8, 8> P_, Qx_;
  Eigen::Matrix<double, 8, 2> B_;
  Eigen::Matrix<double, 2, 2> Qu_;

  // additional variables
  double t_prev_;

  // functions
  void propCallback(const kb_utils::Servo_CommandConstPtr& servo_msg, const kb_utils::EncoderConstPtr& encoder_msg);
  void update();

};

template <typename T1, typename T2>
void rosImportScalar(ros::NodeHandle nh, std::string param, T2& value, T1 default_value)
{
  // get scalar from ROS parameter server
  if (!nh.getParam(param, value))
  {
    ROS_WARN("Could not find %s/%s on the server.", nh.getNamespace().c_str(), param.c_str());
    value = default_value;
  }
}

template <typename T1, typename T2>
void rosImportMatrix(ros::NodeHandle nh, std::string param, Eigen::MatrixBase<T2>& mat)
{
  // get array from ROS parameter server
  std::vector<T1> vec;
  if (!nh.getParam(param, vec))
  {
    ROS_WARN("Could not find %s/%s on the server. Set to zeros.", nh.getNamespace().c_str(), param.c_str());
    mat.setZero();
    return;
  }
 
  // ensure imported array has correct number of values then populate the matrix
  ROS_ASSERT_MSG(vec.size() == mat.rows()*mat.cols(), "Param %s/%s is the wrong size", nh.getNamespace().c_str(),param.c_str());
  for (unsigned i = 0; i < mat.rows(); i++)
    for (unsigned j = 0; j < mat.cols(); j++)
      mat(i,j) = vec[mat.cols()*i+j];
}

} // namespace ekf

#endif // EKF_H
