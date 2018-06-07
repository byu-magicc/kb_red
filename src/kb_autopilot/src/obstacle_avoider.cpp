#include "obstacle_avoider.h"

namespace kb_autopilot
{
obstacle_avoider::obstacle_avoider():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~")),
    state_init_(false),
    turn_left_(false),
    turn_right_(false),
    to_close_(false)
{
    vehicle_state_sub_ = nh_.subscribe<kb_autopilot::State>("state", 1, &obstacle_avoider::vehicle_state_callback, this);
    controller_commands_sub_ = nh_.subscribe<kb_autopilot::Controller_Commands>("controller_commands", 1,
                        &obstacle_avoider::controller_commands_callback, this);
    avoid_commands_pub_ = nh_.advertise<kb_autopilot::Controller_Commands>("avoid_commands", 1);

    nh_private_.param<double>("depth_update_rate", update_rate_, 3.0);
}

void obstacle_avoider::vehicle_state_callback(const kb_autopilot::StateConstPtr &msg)
{
  p_n_ = msg->p_north;               /** position north */
  p_e_ = msg->p_east;               /** position east */

  state_init_ = true;
}

void obstacle_avoider::controller_commands_callback(const kb_autopilot::Controller_CommandsConstPtr &msgi)
{
    kb_autopilot::Controller_Commands msg = *msgi;
    if(!state_init_ || std::sqrt(p_n_*p_n_ + p_e_*p_e_) < 10)  // if close to brigham, don't avoid anything!
    {
        avoid_commands_pub_.publish(msg);
        return;
    }
    else
    {
        if(turn_left_)
            msg.psi_c -= 60.0*M_PI/180.0;
        if(turn_right_)
            msg.psi_c += 60.0*M_PI/180.0;
        if(to_close_) // if close to obstacle, backup! (and keep turning)
        {
            msg.u_c = -1;
            msg.psi_c *= -1;
        }
    }
    avoid_commands_pub_.publish(msg);
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_avoider");
  kb_autopilot::obstacle_avoider *oa = new kb_autopilot::obstacle_avoider();

  ros::spin();

  return 0;
}
