/*
    ROS node to calculate the dynamic contribution

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
#include "Helper.h"

#include <TF_SISO/TF_FIRST_ORDER_FILTER.h>

#define HEADER_PRINT BOLDYELLOW "[Dynamic Controller]: " CRESET

using namespace std;

ros::Publisher outPub;
ros::Subscriber in_sub;

ros::ServiceClient client_pause_kf;

string in_topic("");
string out_topic("");

double i_gain = 0.0;
double integrator_dc_gain = 1E2;
double p_gain;
double input_data;
TF_FIRST_ORDER_FILTER* tf_pseudo_integrator;

std_msgs::Float64 fnd;
void readInput_and_pub(const std_msgs::Float64::ConstPtr& msg){

    fnd.data = fabs(msg->data * p_gain);

	outPub.publish(fnd);
}

void readInput(const std_msgs::Float64::ConstPtr& msg){

    input_data = msg->data;

}

/*Pause callback*/
bool paused = true;
bool pause_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

    std_srvs::SetBool setBoolmsg;
   	setBoolmsg.request.data = req.data;
    bool b_kf = client_pause_kf.call(setBoolmsg);

    if(!b_kf || !setBoolmsg.response.success ){
        cout <<  HEADER_PRINT RED << "Filed to " << BOLDRED << ( req.data ? "STOP" : "START" ) << CRESET RED << " Kalman Filter!" << CRESET << endl;
    }

    if(req.data){

        if(i_gain!=0.0){
           tf_pseudo_integrator->reset(); 
        }

        fnd.data = 0.0;
        paused = true;
        outPub.publish(fnd);

        cout << HEADER_PRINT YELLOW "PAUSED!" CRESET << endl;

    } else {

        if(paused){
            if(i_gain!=0.0){
                tf_pseudo_integrator->reset(); 
            }
            cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
        }

        paused = false;

    }

    res.success = true;
	return true;
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "dynamic_controller");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;
     
    nh_private.param("in_topic" , in_topic, string("in_topic") );
    
    nh_private.param("out_topic" , out_topic, string("dyn_force") );

    string pause_service("");
    nh_private.param("pause_service" , pause_service, string("pause") );
    string pause_kf_service("");
    nh_private.param("pause_kf_service" , pause_kf_service, string("pause") );

    nh_private.param("i_gain" , i_gain, 20.0 );
    nh_private.param("p_gain" , p_gain, 20.0 );
    nh_private.param("integrator_dc_gain" , integrator_dc_gain, 1E2 );

    double Hz;
    nh_private.param("hz" , Hz, 500.0 );
    double Ts = 1.0/Hz;

	// Publisher
    if(i_gain!=0.0){
	    in_sub = nh_public.subscribe(in_topic, 1, readInput);
        tf_pseudo_integrator = new TF_FIRST_ORDER_FILTER(i_gain/integrator_dc_gain/2.0/M_PI, 1.0/Hz, integrator_dc_gain);
    }
    else
        in_sub = nh_public.subscribe(in_topic, 1, readInput_and_pub);

	outPub = nh_public.advertise<std_msgs::Float64>( out_topic,1);

    ros::ServiceServer servicePause = nh_public.advertiseService(pause_service, pause_callbk);

    client_pause_kf = nh_public.serviceClient<std_srvs::SetBool>(pause_kf_service);

    if(i_gain!=0.0){

        //Main Loop
        ros::Rate loop_rate(Hz);
        while(ros::ok()){

            ros::spinOnce();
            if(paused){
                loop_rate.sleep();
                continue;
            }
            
            //Apply control
            fnd.data = fabs( tf_pseudo_integrator->apply(input_data) + p_gain * input_data );          
            
            outPub.publish(fnd);

            loop_rate.sleep();

        } //end main loop

    } else {
        ros::spin();
    }

    return 0;
}
