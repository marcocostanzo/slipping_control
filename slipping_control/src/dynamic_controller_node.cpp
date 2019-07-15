/*
    ROS node to calculate the dynamic contribution

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

#include <sun_ros_msgs/Float64Stamped.h>
#include "std_srvs/SetBool.h"

#include <sun_systems_lib/TF/TF_FIRST_ORDER_FILTER.h>

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[Dynamic Controller]: " CRESET

using namespace std;

ros::Publisher outPub;
ros::Subscriber in_sub;

string in_topic("");
string out_topic("");

double i_gain;
double integrator_dc_gain;
double p_gain;
double input_data;
TF_FIRST_ORDER_FILTER* tf_pseudo_integrator;
bool running = false;

sun_ros_msgs::Float64Stamped fnd;
void readInput_and_pub(const sun_ros_msgs::Float64Stamped::ConstPtr& msg){

    if(!running){
        return;
    }

    fnd.data = fabs(msg->data * p_gain);
    fnd.header = msg->header;

	outPub.publish(fnd);
}

void readInput(const sun_ros_msgs::Float64Stamped::ConstPtr& msg){

    input_data = msg->data;

}

/*Pause callback*/
bool setRunning_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

    if(req.data){

        if(!running){
            input_data = 0.0;
            fnd.data = 0.0;
            if(i_gain!=0.0){
                tf_pseudo_integrator->reset(); 
            }
            cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
        }
        running = true;

    } else{

        if(i_gain!=0.0){
           tf_pseudo_integrator->reset(); 
        }

        fnd.data = 0.0;
        running = false;
        fnd.header.stamp = ros::Time::now();
        outPub.publish(fnd);

        cout << HEADER_PRINT YELLOW "Stopped!" CRESET << endl;

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

    string set_running_service_str("");
    nh_private.param("set_running_service" , set_running_service_str, string("set_running") );

    nh_private.param("i_gain" , i_gain, 50.0 );
    nh_private.param("p_gain" , p_gain, 13.0 );
    nh_private.param("integrator_dc_gain" , integrator_dc_gain, 35.0 );

    double Hz;
    nh_private.param("hz" , Hz, 2000.0 );
    double Ts = 1.0/Hz;

	// Publisher
    if(i_gain!=0.0){
	    in_sub = nh_public.subscribe(in_topic, 1, readInput);
        tf_pseudo_integrator = new TF_FIRST_ORDER_FILTER(i_gain/integrator_dc_gain/2.0/M_PI, 1.0/Hz, integrator_dc_gain);
    }
    else
        in_sub = nh_public.subscribe(in_topic, 1, readInput_and_pub);

	outPub = nh_public.advertise<sun_ros_msgs::Float64Stamped>( out_topic,1);

    ros::ServiceServer serviceSetRunning = nh_public.advertiseService(set_running_service_str, setRunning_callbk);

    if(i_gain!=0.0){

        //Main Loop
        ros::Rate loop_rate(Hz);
        while(ros::ok()){

            ros::spinOnce();
            if(!running){
                loop_rate.sleep();
                continue;
            }
            
            //Apply control  ---> fabs(i+p)  or fabs(i)+fabs(p) ?
            fnd.data = fabs( tf_pseudo_integrator->apply(input_data) + p_gain * input_data );  
            fnd.header.stamp = ros::Time::now();

            /*Security check*/
            if( isnan(fnd.data) || isinf(fnd.data) )
            {
                    cout << HEADER_PRINT << BOLDRED "INF/NAN... EXIT! ..." CRESET << endl;
                    fnd.data = 0.0;
                    outPub.publish(fnd);
                    loop_rate.sleep();
                    exit(-1);
            }
            
            outPub.publish(fnd);

            loop_rate.sleep();

        } //end main loop

    } else {
        ros::spin();
    }

    return 0;
}
