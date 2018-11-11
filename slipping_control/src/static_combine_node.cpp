/*
    ROS node to combine two static components of normal force (slipping avoidance)

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

#include <std_msgs/Float64.h>
#include "std_srvs/SetBool.h"
#include "slipping_control_common/FnsParams.h"

#include "Helper.h"


#define HEADER_PRINT BOLDYELLOW "[Static Combine]: " CRESET 

using namespace std;

/*PARAMS*/

//Input topic
string in_topic0("");
string in_topic1("");

//Pub and sub
ros::Publisher pub_static;
ros::Subscriber sub_static0;
ros::Subscriber sub_static1;

//SService clients
ros::ServiceClient client_pause0;
ros::ServiceClient client_pause1;
ros::ServiceClient client_rotation0;
ros::ServiceClient client_rotation1;

/* USER FUN */
void combineStatic();
/**********************/

double f_static0;
double f_static1;

void static_force_Callback0 (const std_msgs::Float64::ConstPtr& msg) {

    f_static0 = msg->data;
    combineStatic();

}
void static_force_Callback1 (const std_msgs::Float64::ConstPtr& msg) {

    f_static1 = msg->data;
    combineStatic();

}


/*Pause callback*/
bool pause_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

    std_srvs::SetBool setBoolmsg;
   	setBoolmsg.request.data = req.data;

    bool b_0 = client_pause0.call(setBoolmsg);
    b_0 = b_0 && setBoolmsg.response.success;
    if(!b_0){
        cout <<  HEADER_PRINT RED << "Filed to call set pause on 0" << CRESET << endl;
    }
    bool b_1 = client_pause1.call(setBoolmsg);
    b_1 = b_1 && setBoolmsg.response.success;
    if(!b_1){
        cout <<  HEADER_PRINT RED << "Filed to call set pause on 1" << CRESET << endl;
    }

    res.success = b_0 && b_1;
    return true;

}


bool rotation_callbk(slipping_control_common::FnsParams::Request  &req, 
   		 		slipping_control_common::FnsParams::Response &res){

    slipping_control_common::FnsParams setFnsParams;
   	setFnsParams.request.control_rotation = req.control_rotation;

    bool b_0 = client_rotation0.call(setFnsParams);
    b_0 = b_0 && setFnsParams.response.success;
    if(!b_0){
        cout <<  HEADER_PRINT RED << "Filed to call set rotation on 0" << CRESET << endl;
    }
    bool b_1 = client_rotation1.call(setFnsParams);
    b_1 = b_1 && setFnsParams.response.success;
    if(!b_1){
        cout <<  HEADER_PRINT RED << "Filed to call set rotation on 1" << CRESET << endl;
    }

    res.success = b_0 && b_1;
    return true;

}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "static_combine");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    //Pub and sub
    string out_topic("");
    nh_private.param("out_topic" , out_topic, string("static_force") ); 
    nh_private.param("in_topic0" , in_topic0, string("finger0/static_force") );
    nh_private.param("in_topic1" , in_topic1, string("finger1/static_force") );
    
    //Pause Service
    string pause_service("");
    nh_private.param("pause_service" , pause_service, string("pause") ); 

    //Pause CLient
    string pause_client0_str("");
    nh_private.param("pause_client0" , pause_client0_str, string("finger0/pause") ); 
    string pause_client1_str("");
    nh_private.param("pause_client1" , pause_client1_str, string("finger1/pause") );

    //Control rotation service
    string control_rotation_str("");
    nh_private.param("control_rotation_service" , control_rotation_str, string("control_rotation") );

    //Control rotation client
    string control_rotation_client0_str("");
    nh_private.param("control_rotation_client0" , control_rotation_client0_str, string("finger0/control_rotation") ); 
    string control_rotation_client1_str("");
    nh_private.param("control_rotation_client1" , control_rotation_client1_str, string("finger1/control_rotation") );

    /******************/

    pub_static = nh_public.advertise<std_msgs::Float64>(out_topic, 1);

    sub_static0 = nh_public.subscribe(in_topic0, 1, static_force_Callback0);
    sub_static1 = nh_public.subscribe(in_topic1, 1, static_force_Callback1);

    ros::ServiceServer servicePause = nh_public.advertiseService(pause_service, pause_callbk);

    ros::ServiceServer serviceRotation = nh_public.advertiseService(control_rotation_str, rotation_callbk);

    client_pause0 = nh_public.serviceClient<std_srvs::SetBool>(pause_client0_str);
    client_pause1 = nh_public.serviceClient<std_srvs::SetBool>(pause_client1_str);

    client_rotation0 = nh_public.serviceClient<slipping_control_common::FnsParams>(control_rotation_client0_str);
    client_rotation1 = nh_public.serviceClient<slipping_control_common::FnsParams>(control_rotation_client1_str);

    ros::spin();

    return 0;
}

/* USER FUN IMPL */

void combineStatic(){
    std_msgs::Float64 out;
    out.data = f_static0 + f_static1;
    pub_static.publish(out);
}

/*******************/