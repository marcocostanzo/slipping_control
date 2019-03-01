/*
    ROS node to filter COR

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

#include "slipping_control_common/VirtualCORStamped.h"
#include "TF_MIMO/TF_MIMO_DIAGONAL.h"
#include "TF_SISO/TF_FIRST_ORDER_FILTER.h"

using namespace std;
using namespace TooN;

ros::Publisher pubCORFilter;
slipping_control_common::VirtualCORStamped msgCORFilter;
Vector<4> cor_vec = Zeros;
Vector<4> cor_vec_filter = Zeros;

//==========TOPICs CALLBKs=========//
void readCOR( const slipping_control_common::VirtualCORStamped::ConstPtr& msg  ){

    cor_vec[0] = msg->cor.sigma;
    cor_vec[1] = msg->cor.virtual_cor_tilde;
    cor_vec[2] = msg->cor.virtual_radius;
    cor_vec[3] = msg->cor.virtual_cor;
    msgCORFilter.header.stamp = msg->header.stamp;
    msgCORFilter.header.frame_id = msg->header.frame_id;
	
}

//====================================//


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"filter_cor");

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
	pubCORFilter = nh_public.advertise<slipping_control_common::VirtualCORStamped>( str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
	ros::Subscriber subFloat = nh_public.subscribe( str_in_topic , 1, readCOR);
    /***************************/

    /******INIT FILTER************/
    TF_MIMO_DIAGONAL filter(    4,
                                TF_FIRST_ORDER_FILTER(cut_freq, 1.0/Hz), 
                                1.0/Hz
                            );
    /***************************/	

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
	while(ros::ok()){

        cor_vec_filter = filter.apply( cor_vec );

        msgCORFilter.cor.sigma = cor_vec_filter[0];
        msgCORFilter.cor.virtual_cor_tilde = cor_vec_filter[1];
        msgCORFilter.cor.virtual_radius = cor_vec_filter[2];
        msgCORFilter.cor.virtual_cor = cor_vec_filter[3];

		pubCORFilter.publish( msgCORFilter );
		loop_rate.sleep();
      	ros::spinOnce();
			
	}
    
    /*==============================*/

    return 0;
}