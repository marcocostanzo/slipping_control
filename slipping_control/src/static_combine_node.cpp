/*
    ROS node to combine two static components of normal force (slipping avoidance)

    Copyright 2018-2019 Università della Campania Luigi Vanvitelli

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

#include "slipping_control_common/LSStamped.h"
#include "slipping_control_common/LSCombinedStamped.h"



#define HEADER_PRINT BOLDYELLOW "[Static Combine]: " CRESET 

using namespace std;

/*PARAMS*/

//Gain
double fn_ls_gain, min_fn;

//Messages
slipping_control_common::LSStamped in_msg0;
slipping_control_common::LSStamped in_msg1;
slipping_control_common::LSCombinedStamped out_msg;

//Input topic
string in_topic0("");
string in_topic1("");

//Pub and sub
ros::Publisher pub_static;
ros::Subscriber sub_static0;
ros::Subscriber sub_static1;

/* USER FUN */
void combineStatic();
/**********************/

void static_force_Callback0 (const slipping_control_common::LSStamped msg) {

    in_msg0 = msg;
    combineStatic();
    pub_static.publish(out_msg);

}
void static_force_Callback1 (const slipping_control_common::LSStamped msg) {

    in_msg1 = msg;
    combineStatic();
    pub_static.publish(out_msg);

}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "static_combine");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    //Pub and sub
    string out_topic("");
    nh_private.param("out_topic" , out_topic, string("ls_combined") ); 
    nh_private.param("in_topic0" , in_topic0, string("finger0/ls") );
    nh_private.param("in_topic1" , in_topic1, string("finger1/ls") );

    nh_private.param("fn_ls_gain" , fn_ls_gain, 1.1 );
    nh_private.param("min_fn" , min_fn, 1.0 );

    /******************/

    pub_static = nh_public.advertise<slipping_control_common::LSCombinedStamped>(out_topic, 1);

    sub_static0 = nh_public.subscribe(in_topic0, 1, static_force_Callback0);
    sub_static1 = nh_public.subscribe(in_topic1, 1, static_force_Callback1);

    ros::spin();

    return 0;
}

/* USER FUN IMPL */

void combineStatic(){
    
    if(in_msg0.header.stamp > in_msg1.header.stamp)
        out_msg.header.stamp = in_msg0.header.stamp;
    else
        out_msg.header.stamp = in_msg1.header.stamp;

    out_msg.header.frame_id = "combined_ls";

    out_msg.sigma = (in_msg0.sigma - in_msg1.sigma) / 2.0;

    out_msg.cor_tilde = (in_msg0.cor_tilde - in_msg1.cor_tilde) / 2.0;

    out_msg.cor = (in_msg0.cor - in_msg1.cor) / 2.0;

    out_msg.ft_tilde_ls = (in_msg0.ft_tilde_ls + in_msg1.ft_tilde_ls) / 2.0;

    out_msg.taun_tilde_ls = (in_msg0.taun_tilde_ls - in_msg1.taun_tilde_ls) / 2.0;

    out_msg.ft_ls = in_msg0.ft_ls + in_msg1.ft_ls;

    out_msg.taun_ls = in_msg0.taun_ls - in_msg1.taun_ls;

    out_msg.radius = (in_msg0.radius + in_msg1.radius) / 2.0;

    out_msg.generalized_max_force = in_msg0.generalized_max_force + in_msg1.generalized_max_force;

    out_msg.generalized_max_force0 = in_msg0.generalized_max_force;

    out_msg.generalized_max_force1 = in_msg1.generalized_max_force;

    out_msg.generalized_force = in_msg0.generalized_force - in_msg1.generalized_force;

    out_msg.generalized_force0 = in_msg0.generalized_force;

    out_msg.generalized_force1 = -in_msg1.generalized_force;

    out_msg.fn_ls = fn_ls_gain * (in_msg0.fn_ls + in_msg1.fn_ls);

    out_msg.fn_ls_free_pivot = in_msg0.fn_ls_free_pivot + in_msg1.fn_ls_free_pivot;

    if(out_msg.fn_ls < min_fn)
        out_msg.fn_ls = min_fn;
    
    if(out_msg.fn_ls_free_pivot < min_fn)
        out_msg.fn_ls_free_pivot = min_fn;

}

/*******************/