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

#include <std_msgs/Float32.h>
#include "std_srvs/SetBool.h"
#include "Helper.h"

#include <TF_SISO/TF_INTEGRATOR.h>
#include <TF_SISO/TF_FIRST_ORDER_FILTER.h>

#define HEADER_PRINT BOLDYELLOW "[Dynamic Controller]: " CRESET

using namespace std;


ros::NodeHandle * nh;

ros::Publisher outPub;
ros::Subscriber in_sub;

ros::ServiceClient client_pause_kf;

string in_topic("");
string out_topic("");

float control_gain = 0.0;
double input_data;
double input_saturation;
bool b_integral_action;
TF_INTEGRATOR* tf_integrator;
TF_FIRST_ORDER_FILTER* tf_filter;

std_msgs::Float32 fnd;
void readInput_and_pub(const std_msgs::Float32::ConstPtr& msg){

    fnd.data = fabs(msg->data * control_gain);

	outPub.publish(fnd);
}

void readInput(const std_msgs::Float32::ConstPtr& msg){

    if( fabs(msg->data) < input_saturation )
        input_data = 0.0;
    else
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

        if(b_integral_action){
           tf_integrator->reset(); 
           tf_filter->reset();
        }

        fnd.data = 0.0;
        paused = true;
        outPub.publish(fnd);

        cout << HEADER_PRINT YELLOW "PAUSED!" CRESET << endl;

    } else {

        if(paused){
            if(b_integral_action){
                tf_integrator->reset(); 
                tf_filter->reset();
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

    nh = new ros::NodeHandle("~");
     
    nh->param("in_topic" , in_topic, string("in_topic") );
    
    nh->param("out_topic" , out_topic, string("dyn_force") );

    string pause_service("");
    nh->param("pause_service" , pause_service, string("pause") );
    string pause_kf_service("");
    nh->param("pause_kf_service" , pause_kf_service, string("pause") );

    nh->param("controlGain" , control_gain, (float)2000.0 );
    float p_gain;
    nh->param("p_gain" , p_gain, (float)0.1 );

    nh->param("integral_action" , b_integral_action, false );
    double Hz;
    nh->param("hz" , Hz, 500.0 );
    double Ts = 1.0/Hz;
    nh->param("input_saturation" , input_saturation, 0.0 );

    double timer_max;
    double cut_freq;
    double timer_state;
    bool b_use_int = false;
    nh->param("timer_max" , timer_max, 5.0 );
    nh->param("cut_freq" , cut_freq, 0.3 );

	// Publisher
    if(b_integral_action){
	    in_sub = nh->subscribe(in_topic, 1, readInput);
        tf_integrator = new TF_INTEGRATOR(1.0/Hz);
        tf_filter = new TF_FIRST_ORDER_FILTER(cut_freq, 1.0/Hz);
        timer_state = timer_max;
        b_use_int = false;
    }
    else
        in_sub = nh->subscribe(in_topic, 1, readInput_and_pub);

	outPub = nh->advertise<std_msgs::Float32>( out_topic,1);

    ros::ServiceServer servicePause = nh -> advertiseService(pause_service, pause_callbk);

    client_pause_kf = nh -> serviceClient<std_srvs::SetBool>(pause_kf_service);

    control_gain = fabs(control_gain);

    if(b_integral_action){

        //Main Loop
        ros::Rate loop_rate(Hz);
        while(ros::ok()){

            ros::spinOnce();
            if(paused){
                loop_rate.sleep();
                continue;
            }

            //Timer
            if(input_data == 0.0 && timer_state > 0.0){
                timer_state -= Ts;
            } else if(input_data != 0.0) {
                timer_state = timer_max;
            }

            //Switch sys
            if( b_use_int &&  timer_state <= 0.0){ //Switch to filter
            cout << HEADER_PRINT << "TO FILTER!" << endl;
                tf_filter->setState( fnd.data );
                b_use_int = false;
            } else if( !b_use_int && timer_state > 0.0 ){ //Switch to integrator
            cout << HEADER_PRINT << "TO INT!" << endl;
                tf_integrator->setState( fnd.data );
                b_use_int = true;
            }
            
            //Apply correct filter
            if(b_use_int){
                fnd.data = fabs( control_gain * tf_integrator->apply(input_data) + p_gain * input_data );
            } else {
                fnd.data = fabs( tf_filter->apply(0.0) );
            }
            
            
            outPub.publish(fnd);

            loop_rate.sleep();

        } //end main loop

    } else {
        ros::spin();
    }

    return 0;
}
