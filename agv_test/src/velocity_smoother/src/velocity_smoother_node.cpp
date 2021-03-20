#include "velocity_smoother.h"

int main(int argc, char ** argv)
{
    ros::init( argc, argv, "velocity_smoother_node");
    VelocitySmoother vs;
    ros::spin();
    return 0;
}

