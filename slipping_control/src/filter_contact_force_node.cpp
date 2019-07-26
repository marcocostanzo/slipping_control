/*
    ROS node to filter Contact Forces

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

#include "slipping_control_msgs/ContactForcesStamped.h"
#include "sun_systems_lib/TF/TF_MIMO_DIAGONAL.h"
#include "sun_systems_lib/TF/TF_FIRST_ORDER_FILTER.h"

using namespace std;
using namespace TooN;

ros::Publisher pubContactFilter;
slipping_control_msgs::ContactForcesStamped msgContactFilter;
Vector<3> contact_vec = Zeros;
Vector<3> contact_vec_filter = Zeros;

//==========TOPICs CALLBKs=========//
void readContact( const slipping_control_msgs::ContactForcesStamped::ConstPtr& msg  ){

    contact_vec[0] = msg->forces.ft;
    contact_vec[1] = msg->forces.taun;
    contact_vec[2] = msg->forces.fn;
    msgContactFilter.header.stamp = ros::Time::now(); //msg->header.stamp;
    msgContactFilter.header.frame_id = msg->header.frame_id;
	
}

//====================================//


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"filter_contact");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** PARAMS ****/
    string str_in_topic = string("");
    nh_private.param("in_topic" , str_in_topic, string("/tactile") );
    string str_out_topic = string("");
    nh_private.param("out_topic" , str_out_topic, str_in_topic + string("/filter") );
    double cut_freq;
    nh_private.param("cut_freq" , cut_freq, 20.0 );
    double Hz;
    nh_private.param("rate" , Hz, 500.0 );
	/************************************/

    /******INIT ROS MSGS**********/
	/********************/

    /*******INIT ROS PUB**********/
	//Float_filter pub
	pubContactFilter = nh_public.advertise<slipping_control_msgs::ContactForcesStamped>( str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
	ros::Subscriber subContact = nh_public.subscribe( str_in_topic , 1, readContact);
    /***************************/

    /******INIT FILTER************/
    TF_MIMO_DIAGONAL filter(    3,
                                TF_FIRST_ORDER_FILTER(cut_freq, 1.0/Hz)
                            );
    /***************************/	

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
	while(ros::ok()){

        contact_vec_filter = filter.apply( contact_vec );

        msgContactFilter.forces.ft = contact_vec_filter[0];
        msgContactFilter.forces.taun = contact_vec_filter[1];
        msgContactFilter.forces.fn = contact_vec_filter[2];

		pubContactFilter.publish( msgContactFilter );
		loop_rate.sleep();
      	ros::spinOnce();
			
	}
    
    /*==============================*/

    return 0;
}