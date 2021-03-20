#ifndef VELOCITYS_MOOTHER_H
#define VELOCITYS_MOOTHER_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <mutex>

using namespace std;

class VelocitySmoother
{
public:
    VelocitySmoother();

    void controlTimerCallback(const ros::TimerEvent &event);

    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);

private:
    void calculateVelocity();
    inline void increaseSpeed(double &target, double &current, double accel, double decel);
    inline void limitRange(double &value, double min_value, double max_value);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer control_timer_;
    ros::Subscriber raw_vel_sub_;
    ros::Publisher smooth_vel_pub_;

    //
    double delta_time_;
    ros::Time target_stamp_;
    geometry_msgs::Twist target_vel_;
    geometry_msgs::Twist current_vel_;
    std::mutex vel_mutex_;

    // Parameters
    bool verbose_;
    double frequency_;
    double velocity_timeout_;
    double speed_max_x_;
    double speed_min_x_;
    double speed_max_y_;
    double speed_min_y_;
    double speed_lim_w_;
    double accel_lim_x_;
    double accel_lim_y_;
    double accel_lim_w_;
    double decel_lim_x_;
    double decel_lim_y_;
    double decel_lim_w_;
    double decel_factor_;
    //
};

#endif // VELOCITYSMOOTHER_H
