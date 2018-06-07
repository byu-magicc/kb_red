#include <ros/ros.h>
#include <kb_autopilot/Waypoint.h>

#define num_waypoints 3

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_path_planner");

  ros::NodeHandle nh_;
  ros::Publisher waypointPublisher = nh_.advertise<kb_autopilot::Waypoint>("waypoint_path", 10);

  float u = 1.5;
  float wps[3*num_waypoints] =
  {
    200, 0, u,
    0, 200, u,
    200, 200, u,
  };

  for (int i(0); i < num_waypoints; i++)
  {
    ros::Duration(0.5).sleep();

    kb_autopilot::Waypoint new_waypoint;

    new_waypoint.w[0] = wps[i*3 + 0];
    new_waypoint.w[1] = wps[i*3 + 1];
    new_waypoint.u_d = wps[i*3 + 2];

    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    waypointPublisher.publish(new_waypoint);
  }
  ros::Duration(1.5).sleep();

  return 0;
}
