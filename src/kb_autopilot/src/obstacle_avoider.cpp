#include "obstacle_avoider.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

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
    depth_image_sub_ = nh_.subscribe("depth", 1, &obstacle_avoider::depth_callback, this);
    avoid_commands_pub_ = nh_.advertise<kb_autopilot::Controller_Commands>("avoid_commands", 1);

    nh_private_.param<double>("depth_update_rate", update_rate_, 3.0);
}

void obstacle_avoider::vehicle_state_callback(const kb_autopilot::StateConstPtr &msg)
{
  p_n_ = msg->p_north;               /** position north */
  p_e_ = msg->p_east;               /** position east */

  state_init_ = true;
}

void obstacle_avoider::depth_callback(const sensor_msgs::ImagePtr &msg)
{
    if ((ros::Time::now() - last_update_).toSec() < 1.0/update_rate_)
        return;
    last_update_ = ros::Time::now();


    cv_bridge::CvImagePtr cv_dpth_ptr;
    try
    {
        cv_dpth_ptr = cv_bridge::toCvCopy(*msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    int rows = cv_dpth_ptr->image.rows;
    int cols = cv_dpth_ptr->image.cols;
    cv::Mat cv_dpth = cv_dpth_ptr->image(cv::Rect(cols/4,rows/4,cols/2,rows/4));

    cv::Mat color;
    cv::cvtColor(cv_dpth_ptr->image, color, cv::COLOR_GRAY2BGR);

    cv::rectangle(color, cv::Rect(cols/4,rows/4,cols/2,rows/4), cv::Scalar(150,0,0));

    cv::imshow("here",color);
    cv::waitKey(1);

    double min(65535), max(0);
    double total(0);
    for(int i(0);i<cv_dpth.rows;i++)
    {
        for(int j(0);j<cv_dpth.cols;j++)
        {
            float d = cv_dpth.at<float>(i,j);
            if(d > 0.01) // greater then min
            {
                if(d > max)
                    max = d;
                if(d < min)
                    min = d;
                if(d > 1000 && d < 2000)
                {
                    if(j < cv_dpth.cols/2) //right side
                        total += 1000/d;
                    else                //left side
                        total -= 1000/d;
                }
            }

        }
    }

    if (total > 1000)
        turn_right_ = true;
    else
        turn_right_ = false;

    if (total < -1000)
        turn_left_ = true;
    else
        turn_left_ = false;

    if(min < 1000)
        to_close_ = true;
    else
        to_close_ = false;
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
