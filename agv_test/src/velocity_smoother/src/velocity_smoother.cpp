#include "velocity_smoother.h"

VelocitySmoother::VelocitySmoother()
    : nh_(),
      nh_private_("~")
{
    nh_private_.param<bool>("verbose", verbose_, false);
    nh_private_.param<double>("frequency", frequency_, 20.0);
    nh_private_.param<double>("velocity_timeout", velocity_timeout_, 0);
    nh_private_.param<double>("speed_max_x", speed_max_x_, 1.0);
    nh_private_.param<double>("speed_min_x", speed_min_x_, -1.0);
    nh_private_.param<double>("speed_max_y", speed_max_y_, 1.0);
    nh_private_.param<double>("speed_min_y", speed_min_y_, -1.0);
    nh_private_.param<double>("speed_lim_w", speed_lim_w_, 1.0);
    nh_private_.param<double>("accel_lim_x", accel_lim_x_, 1.0);
    nh_private_.param<double>("accel_lim_y", accel_lim_y_, 1.0);
    nh_private_.param<double>("accel_lim_w", accel_lim_w_, 1.0);
    nh_private_.param<double>("decel_factor", decel_factor_, 1.5);
    decel_lim_x_ = accel_lim_x_ * decel_factor_;
    decel_lim_y_ = accel_lim_y_ * decel_factor_;
    decel_lim_w_ = accel_lim_w_ * decel_factor_;

    //
    delta_time_ = 1.0 / frequency_;
    target_stamp_ = ros::Time::now();
    target_vel_.linear.x = target_vel_.linear.y = target_vel_.linear.z = 0;
    target_vel_.angular.x = target_vel_.angular.y = target_vel_.angular.z = 0;
    current_vel_.linear.x = current_vel_.linear.y = current_vel_.linear.z = 0;
    current_vel_.angular.x = current_vel_.angular.y = current_vel_.angular.z = 0;

    //
    raw_vel_sub_  = nh_.subscribe("raw_cmd_vel", 1, &VelocitySmoother::velocityCallback, this);
    smooth_vel_pub_  = nh_.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

    control_timer_ = nh_.createTimer( ros::Duration(delta_time_), &VelocitySmoother::controlTimerCallback, this );

}

void VelocitySmoother::controlTimerCallback(const ros::TimerEvent &event)
{
    vel_mutex_.lock();
    calculateVelocity();
    vel_mutex_.unlock();
    // Publish
    smooth_vel_pub_.publish(current_vel_);
}

void VelocitySmoother::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
//    ROS_INFO_STREAM("Velocity command.");
    vel_mutex_.lock();
    target_stamp_ = ros::Time::now();
    target_vel_.linear.x = msg->linear.x;
    target_vel_.linear.y = msg->linear.y;
    target_vel_.angular.z = msg->angular.z;
    //
    //
    limitRange(target_vel_.linear.x, -speed_max_x_, speed_max_x_);
    limitRange(target_vel_.linear.y, -speed_max_y_, speed_max_y_);
    limitRange(target_vel_.angular.z, -speed_lim_w_, speed_lim_w_);
    vel_mutex_.unlock();
}

void VelocitySmoother::calculateVelocity()
{
    if(velocity_timeout_ > 0 && (ros::Time::now()-target_stamp_).toSec() > velocity_timeout_)
    {
        target_vel_.linear.x = 0;
        target_vel_.linear.y = 0;
        target_vel_.angular.z = 0;
    }
    //
    increaseSpeed(target_vel_.linear.x, current_vel_.linear.x, accel_lim_x_*delta_time_, decel_lim_x_*delta_time_);
    increaseSpeed(target_vel_.linear.y, current_vel_.linear.y, accel_lim_y_*delta_time_, decel_lim_x_*delta_time_);
    increaseSpeed(target_vel_.angular.z, current_vel_.angular.z, accel_lim_w_*delta_time_, decel_lim_x_*delta_time_);
    //
    limitRange(current_vel_.linear.x, speed_min_x_, speed_max_x_);
    limitRange(current_vel_.linear.y, speed_min_y_, speed_max_y_);
    limitRange(current_vel_.angular.z, -speed_lim_w_, speed_lim_w_);
}


void VelocitySmoother::increaseSpeed(double &target, double &current, double accel, double decel)
{
    double delta = target - current;
    if(delta * current >= 0)
    {
        if(delta > accel)
            current += accel;
        else if(delta < -accel)
            current -= accel;
        else
            current = target;
    }
    else
    {
        if(delta > decel)
            current += decel;
        else if(delta < -decel)
            current -= decel;
        else
            current = target;
    }

}

void VelocitySmoother::limitRange(double &value, double min_value, double max_value)
{
    if(value < min_value)
        value = min_value;
    if(value > max_value)
        value = max_value;
}
