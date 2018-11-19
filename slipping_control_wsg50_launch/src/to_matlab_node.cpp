/*
    ROS node to convert topic for the matlab GUI

    Copyright 2018 Università della Campania Luigi Vanvitelli

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
#include "std_msgs/Float64MultiArray.h"
#include "slipping_control_common/ContactForcesStamped.h"
#include "slipping_control_common/MaxForcesStamped.h"
#include "slipping_control_common/VirtualCORStamped.h"
#include "sun_utils/MultiVector.h"

using namespace std;

ros::Publisher pub_contact_forces;   
ros::Publisher pub_max_forces;
ros::Publisher pub_virtual_cor;
ros::Publisher pub_kf_measure;

void contact_forces_cb(const slipping_control_common::ContactForcesStamped::ConstPtr& msg) {

    std_msgs::Float64MultiArray outmsg;
    outmsg.data.resize(3);

    outmsg.data[0] = msg->forces.fn;
    outmsg.data[1] = msg->forces.ft;
    outmsg.data[2] = msg->forces.taun;

    pub_contact_forces.publish(outmsg);

}

void max_forces_cb(const slipping_control_common::MaxForcesStamped::ConstPtr& msg) {

    std_msgs::Float64MultiArray outmsg;
    outmsg.data.resize(4);

    outmsg.data[0] = msg->forces.force_frac;
    outmsg.data[1] = msg->forces.ft_max;
    outmsg.data[2] = msg->forces.taun_max;
    outmsg.data[3] = msg->forces.generalized_max_force;

    pub_max_forces.publish(outmsg);

}

void virtual_cor_cb(const slipping_control_common::VirtualCORStamped::ConstPtr& msg) {

    std_msgs::Float64MultiArray outmsg;
    outmsg.data.resize(4);

    outmsg.data[0] = msg->cor.sigma;
    outmsg.data[1] = msg->cor.virtual_radius;
    outmsg.data[2] = msg->cor.virtual_cor_tilde;
    outmsg.data[3] = msg->cor.virtual_cor;

    pub_virtual_cor.publish(outmsg);

}

void kf_measure_cb(const sun_utils::MultiVector::ConstPtr& msg) {

    std_msgs::Float64MultiArray outmsg;
    outmsg.data.resize(2);

    outmsg.data[0] = msg->data[0];
    outmsg.data[1] = msg->data[1];

    pub_virtual_cor.publish(outmsg);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "to_matlab");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    string contact_forces_topic_str("");
    nh_private.param( "contact_forces_topic", contact_forces_topic_str, string("contact_force") );
    string max_forces_topic_str("");
    nh_private.param( "max_forces_topic", max_forces_topic_str, string("max_force") );
    string virtual_cor_topic_str("");
    nh_private.param( "virtual_cor_topic", virtual_cor_topic_str, string("virtual_cor") );
    string kf_measure_topic_str("");
    nh_private.param( "kf_measure_topic", kf_measure_topic_str, string("") );

    ros::Subscriber sub_contact_forces = nh_public.subscribe( contact_forces_topic_str, 1, contact_forces_cb );
    ros::Subscriber sub_max_forces = nh_public.subscribe( max_forces_topic_str, 1, max_forces_cb );
    ros::Subscriber sub_virtual_cor = nh_public.subscribe( virtual_cor_topic_str, 1, virtual_cor_cb );
    
    pub_contact_forces = nh_public.advertise<std_msgs::Float64MultiArray>(contact_forces_topic_str+"/matlab", 1);
    pub_max_forces = nh_public.advertise<std_msgs::Float64MultiArray>(max_forces_topic_str+"/matlab", 1);
    pub_virtual_cor = nh_public.advertise<std_msgs::Float64MultiArray>(virtual_cor_topic_str+"/matlab", 1);
    
    ros::Subscriber sub_kf_measure;
    if(!kf_measure_topic_str.empty()){
        sub_kf_measure = nh_public.subscribe( kf_measure_topic_str, 1, kf_measure_cb );
        pub_kf_measure = nh_public.advertise<std_msgs::Float64MultiArray>(kf_measure_topic_str+"/matlab", 1);
    }

    ros::spin();

    return 0;
}