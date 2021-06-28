#include "ros/ros.h"
#include "slipping_control_msgs/LSCombinedStamped.h"

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "gen_observer_input");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    slipping_control_msgs::LSCombinedStamped msg;

    ros::Publisher pub = nh_public.advertise<slipping_control_msgs::LSCombinedStamped>("/yaskawa/wsg50/ls_combined_fake",1);

    double g0 = 0.03*2;
    double gf = 0.02;
    double y0 = 0.03;
    double duration = 10.0;

    std::cout << "EPS: " << std::numeric_limits<double>::epsilon() << std::endl;

    msg.generalized_force = y0;
    msg.generalized_max_force = gf;

    ros::Time t0 = ros::Time::now();
    ros::Time tf = t0 + ros::Duration(duration);

    while(ros::ok())
    {
        double t = (ros::Time::now() - t0).toSec();

        if(ros::Time::now()<tf)
            msg.generalized_max_force = g0 + (gf - g0)*t/duration;

        double rand_01 = ((double) rand() / (RAND_MAX));
        double rand__1_1 = (rand_01 - 0.5)*2.0; 

        msg.generalized_force = y0*(1 + 0.01*rand__1_1);

        

        pub.publish(msg);

    }




    return 0;
}
