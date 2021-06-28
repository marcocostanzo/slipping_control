/*
    ROS node to fix sign of torque

    Copyright 2019 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ros/ros.h"

#include "std_msgs/Int8.h"
#include "slipping_control_msgs/ContactForcesStamped.h"
#include "slipping_control_msgs/ContactForcesFixStamped.h"
#include "Helper.h"

// #define SIGN_FIX_TAUN_TRIGGER 0.003

// // #define SCHMITT_TRIGGER_MIN 0.0005
// // #define SCHMITT_TRIGGER_MAX 0.003
// // #define SCHMITT_TRIGGER_OUT_WHEN_ACTIVE 0.003

// #define SCHMITT_TRIGGER_MIN 0.0015
// #define SCHMITT_TRIGGER_MAX 0.0015
// #define SCHMITT_TRIGGER_OUT_WHEN_ACTIVE 0.0015

#define SIGN_FIX_TAUN_TRIGGER 0.001
#define SCHMITT_TRIGGER_MIN 0.001
#define SCHMITT_TRIGGER_MAX 0.001
#define SCHMITT_TRIGGER_OUT_WHEN_ACTIVE 0.001

using namespace std;

//TRIGGER FCN
double trigger_fcn( double x_k, double & s_k, double DELTA ){

    if(s_k > 0.0 && x_k < -DELTA ){
        s_k = -1.0;
    } else if( s_k < 0.0 && x_k > DELTA ){
        s_k = 1.0;
    }

    if( fabs(x_k) > DELTA ){
        return x_k;
    } else{
        return DELTA * s_k;
    }

}

ros::Publisher pubContact0;
ros::Publisher pubContact1;

//State vars
double segn_tau0 = -1.0;
double segn_tau1 = 1.0;
double segn_tau = -1.0;
bool schmitt_t_tau0_state = true;
bool schmitt_t_tau1_state = true;
double tau0_abs = 0.0, tau1_abs = 0.0;

double taun0_filtered = 0;
double taun1_filtered = 0;

#include "geometry_msgs/WrenchStamped.h"
void wrench0FilteredCB( const geometry_msgs::WrenchStampedConstPtr& msg ){
    taun0_filtered = msg->wrench.torque.z;
}
void wrench1FilteredCB( const geometry_msgs::WrenchStampedConstPtr& msg ){
    taun1_filtered = msg->wrench.torque.z;
}

void wrench0CB( const slipping_control_msgs::ContactForcesStamped::Ptr& msg ){

    tau0_abs = fabs(msg->forces.taun);

    // Trigger sign tau
    trigger_fcn( msg->forces.taun, segn_tau0, SIGN_FIX_TAUN_TRIGGER );
    // choose sign using abs
    if(tau0_abs >= tau1_abs)
        segn_tau = segn_tau0;
    else
        segn_tau = -segn_tau1;
    
    //fix 2.0? - IDK
    if(segn_tau0 == segn_tau1 && (tau0_abs - tau1_abs) < 0.002 )
        segn_tau = segn_tau0;

    // Trigger on tau value
    double tau0_abs_triggered = SchmittTrigger( 
                                                tau0_abs, 
                                                schmitt_t_tau0_state, 
                                                SCHMITT_TRIGGER_MIN,
                                                SCHMITT_TRIGGER_MAX,
                                                SCHMITT_TRIGGER_OUT_WHEN_ACTIVE
                                                );

    slipping_control_msgs::ContactForcesFixStamped out_msg;
    out_msg.header = msg->header;
    out_msg.forces.fn = msg->forces.fn;
    out_msg.forces.ft = msg->forces.ft;
    // // out_msg.forces.taun = segn_tau*tau0_abs;
    // out_msg.forces.taun = msg->forces.taun;
    // out_msg.forces.taun_fixed = segn_tau*tau0_abs_triggered;
    // out_msg.forces.taun_filtered = taun0_filtered;

    out_msg.forces.taun = -fabs(msg->forces.taun);
    out_msg.forces.taun_fixed = -fabs(tau0_abs_triggered);
    out_msg.forces.taun_filtered = -fabs(taun0_filtered);

    // out_msg.forces.taun = msg->forces.taun;
    // out_msg.forces.taun_fixed = tau0_abs_triggered;
    // out_msg.forces.taun_filtered = taun0_filtered;

    pubContact0.publish( out_msg );

}

void wrench1CB( const slipping_control_msgs::ContactForcesStamped::Ptr& msg ){

    tau1_abs = fabs(msg->forces.taun);

    // Trigger sign tau
    trigger_fcn( msg->forces.taun, segn_tau1, SIGN_FIX_TAUN_TRIGGER );

    // Trigger on tau value
    double tau1_abs_triggered = SchmittTrigger( 
                                                tau1_abs, 
                                                schmitt_t_tau1_state, 
                                                SCHMITT_TRIGGER_MIN,
                                                SCHMITT_TRIGGER_MAX, 
                                                SCHMITT_TRIGGER_OUT_WHEN_ACTIVE
                                                );

    slipping_control_msgs::ContactForcesFixStamped out_msg;
    out_msg.header = msg->header;
    out_msg.forces.fn = msg->forces.fn;
    out_msg.forces.ft = msg->forces.ft;
    // // out_msg.forces.taun = -segn_tau*tau1_abs;
    // out_msg.forces.taun = msg->forces.taun;
    // out_msg.forces.taun_fixed = -segn_tau*tau1_abs_triggered;
    // out_msg.forces.taun_filtered = taun1_filtered;

    out_msg.forces.taun = fabs(msg->forces.taun);
    out_msg.forces.taun_fixed = fabs(tau1_abs_triggered);
    out_msg.forces.taun_filtered = fabs(taun1_filtered);

    pubContact1.publish( out_msg );

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "contact_fix");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    string input_topic0_str;
    nh_private.param("input_topic0" , input_topic0_str, string("contact0/raw") );
    string input_topic1_str;
    nh_private.param("input_topic1" , input_topic1_str, string("contact1/raw") );

    string output_topic0_str;
    nh_private.param("output_topic0" , output_topic0_str, string("contact0/fix") );
    string output_topic1_str;
    nh_private.param("output_topic1" , output_topic1_str, string("contact1/fix") );

    string wrench_filtered_topic0_str;
    nh_private.param("wrench_filtered0_tipic" , wrench_filtered_topic0_str, string("wrench0_filter") );
    string wrench_filtered_topic1_str;
    nh_private.param("wrench_filtered1_tipic" , wrench_filtered_topic1_str, string("wrench1_filter") );

    ros::Subscriber subContact0 = nh_public.subscribe( input_topic0_str, 1, wrench0CB);
    ros::Subscriber subContact1 = nh_public.subscribe( input_topic1_str, 1, wrench1CB);

    ros::Subscriber subWrenchFiltered0 = nh_public.subscribe( wrench_filtered_topic0_str, 1, wrench0FilteredCB);
    ros::Subscriber subWrenchFiltered1 = nh_public.subscribe( wrench_filtered_topic1_str, 1, wrench1FilteredCB);

    pubContact0 = nh_public.advertise<slipping_control_msgs::ContactForcesFixStamped>(output_topic0_str, 1);
    pubContact1 = nh_public.advertise<slipping_control_msgs::ContactForcesFixStamped>(output_topic1_str, 1);
    
    ros::spin();

    return 0;
}