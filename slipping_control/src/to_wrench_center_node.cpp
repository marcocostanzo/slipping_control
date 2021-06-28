#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wrench_2_contact");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    return 0;
}
