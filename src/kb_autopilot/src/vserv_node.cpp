#include "vserv/vserv.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vserv_node"); // start ROS node
  vserv::EKF vserv_object; // start callbacks
  ros::spin();
  return 0;
}