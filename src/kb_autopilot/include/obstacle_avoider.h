#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include <ros/ros.h>
#include <kb_autopilot/State.h>
#include <kb_autopilot/Controller_Commands.h>
#include <sensor_msgs/Image.h>

namespace kb_autopilot
{

class obstacle_avoider
{
public:
  obstacle_avoider();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber controller_commands_sub_;
  ros::Subscriber depth_image_sub_;

  ros::Publisher avoid_commands_pub_;

  void vehicle_state_callback(const kb_autopilot::StateConstPtr &msg);
  void controller_commands_callback(const kb_autopilot::Controller_CommandsConstPtr &msg);
  void depth_callback(const sensor_msgs::ImagePtr &msg);

  double update_rate_;
  double p_n_;
  double p_e_;
  bool state_init_;

  bool to_close_;
  bool turn_left_;
  bool turn_right_;
};

} // end namespace

#endif // OBSTACLE_AVOIDER_H
