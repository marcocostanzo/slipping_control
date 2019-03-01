/*
    ROS node to compute static component of normal force (slipping avoidance)

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
#include "slipping_control_common/ContactForcesStamped.h"

#include "Helper.h"
#include "learn_algs/learn_algs.h"
#include "slipping_control_common/functions.h"
#include "slipping_control_common/SetMu.h"


#define HEADER_PRINT BOLDYELLOW "[Compute Static]: " CRESET 

using namespace std;
using namespace TooN;

/*PARAMS*/

//Params
LS_INFO ls_info;
double (*ls_function)(double);
double (*diff_ls_function)(double);

//Trigger
bool taun_trigger_state = true;
double TAUN_MIN, TAUN_MAX, TAUN_WHEN_TRIGGER_ACTIVE;

//FindZero -> Fn_min
double GD_GAIN, GD_COST_TOL, FIND_ZERO_LAMBDA;
double DEFAULT_MIN_FN;
int MAX_GD_ITER;

//Security vars
double secure_gain = 1.2;
double MIN_FN_MIN = 1.5;

//Input topic
string in_topic("");

//If the torque has to been taken into account
bool b_use_rotation = true;

//Public Node Handle
ros::NodeHandle* nh_public;

//Pub and sub
ros::Publisher pub_static;
ros::Subscriber sub_wrench;

/* USER FUN */
void startSubscribers();
void stopSubscribers();
/**********************/

std_msgs::Float64 Fn_min;
void contact_force_Callback (const slipping_control_common::ContactForcesStamped::ConstPtr& msg) {

    if(b_use_rotation){

        //Trigger
        double taun_triggered = SchmittTrigger( fabs(msg->forces.taun), taun_trigger_state, TAUN_MIN, TAUN_MAX, TAUN_WHEN_TRIGGER_ACTIVE );

        //Calc min Fn
        bool b_max_iter;
        double initial_point;
        if(msg->forces.fn == 0.0){
            initial_point = DEFAULT_MIN_FN;
        } else{
            initial_point = msg->forces.fn;
        }
        double x = findZero(    initial_point,
                                boost::bind(min_force_Jcst, 
                                            _1, 
                                            ls_function,
                                            msg->forces.ft,
                                            fabs(msg->forces.taun),
                                            ls_info
                                            ), 
                                boost::bind(min_force_gradJ, 
                                            _1, 
                                            diff_ls_function,
                                            msg->forces.ft,
                                            fabs(msg->forces.taun),
                                            ls_info
                                            ), 
                                GD_GAIN, 
                                GD_COST_TOL, 
                                FIND_ZERO_LAMBDA, 
                                MAX_GD_ITER, 
                                b_max_iter );

        if( !isnan(x) && x!=0.0 && !b_max_iter ){
            Fn_min.data = secure_gain*fabs(x);
        }
        //TO DO... min only in this case?
        if(Fn_min.data < MIN_FN_MIN){
            Fn_min.data = MIN_FN_MIN;
        }
        
    } else {
        Fn_min.data = secure_gain*msg->forces.ft/ls_info.mu_;
    }   

    pub_static.publish(Fn_min);

}


/*Pause callback*/
bool paused = true;
bool pause_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

	if(req.data){

        stopSubscribers();
		cout << HEADER_PRINT YELLOW "PAUSED!" CRESET << endl;
        paused = true;

	} else{

        if(paused){
            startSubscribers();
		    cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
        }
        paused = false;
    }

    res.success = true;
	return true;

}


bool rotation_callbk(slipping_control_common::FnsParams::Request  &req, 
   		 		slipping_control_common::FnsParams::Response &res){

    if(req.control_rotation){
        cout << HEADER_PRINT YELLOW "Control Rotation yes!" CRESET << endl;
        b_use_rotation = true;
    } else {
        cout << HEADER_PRINT YELLOW "Control Rotation no!" CRESET << endl;
        b_use_rotation = false;
    }

    res.success = true;
    return true;

}

bool ch_mu_callbk(slipping_control_common::SetMu::Request  &req, 
   		 		slipping_control_common::SetMu::Response &res){

    cout << HEADER_PRINT << "Service Change mu: " << req.mu << endl;

    if(req.mu<0.0){
        cout << HEADER_PRINT << BOLDRED "ERROR!" << endl;
        res.success = false;
        return true;
    }

    ls_info.mu_ = req.mu;
    ls_info.alpha_ = computeAlpha( ls_info );

    cout << HEADER_PRINT << GREEN "Service Change mu OK " << endl;

    res.success = true;
	return true;

}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "compute_static_force");

    ros::NodeHandle nh_private("~");
    nh_public = new ros::NodeHandle();

    //params
    nh_private.param("beta" , ls_info.beta_, 0.002 );
    nh_private.param("gamma" , ls_info.gamma_, 0.3333 );
    nh_private.param("mu" , ls_info.mu_, 0.8 );
    ls_info.alpha_ = computeAlpha( ls_info );
    bool line_approx;
    nh_private.param("line_approx" , line_approx, false );
    if(line_approx){
        ls_function = limitSurface_line;
        diff_ls_function = diff_limitSurface_line;
    } else {
        ls_function = limitSurface_true;
        diff_ls_function = diff_limitSurface_true;
    }

    //Trigger
    nh_private.param("trigger_taun_min" , TAUN_MIN, 0.002 );
    nh_private.param("trigger_taun_max" , TAUN_MAX, 0.010 );
    nh_private.param("trigger_taun_when_trigger_active" , TAUN_WHEN_TRIGGER_ACTIVE, TAUN_MIN );

    //Fzero -> minFn
    nh_private.param("newton_gain" , GD_GAIN, 1.0 );
    nh_private.param("newton_cost_tol" , GD_COST_TOL, 1.0E-6 );
    nh_private.param("newton_max_iter" , MAX_GD_ITER, 150 );
    nh_private.param("newton_lambda" , FIND_ZERO_LAMBDA, 1.0E-10 );
    nh_private.param("default_fn_min" , DEFAULT_MIN_FN, 20.0 );

    //Security vars
    nh_private.param("secure_gain" , secure_gain, 1.2 );
    nh_private.param("min_fn_min" , MIN_FN_MIN, 2.0 );

    //Pub and sub
    string out_topic("");
    nh_private.param("out_topic" , out_topic, string("static_force") ); 
    nh_private.param("in_topic" , in_topic, string("wrench") );
    
    //Pause Service
    string pause_service("");
    nh_private.param("pause_service" , pause_service, string("pause") ); 

    //Control rotation service
    string control_rotation_str("");
    nh_private.param("control_rotation_service" , control_rotation_str, string("control_rotation") );

    /******************/

    pub_static = nh_public->advertise<std_msgs::Float64>(out_topic, 1);

    ros::ServiceServer servicePause = nh_public->advertiseService(pause_service, pause_callbk);

    ros::ServiceServer serviceRotation = nh_public->advertiseService(control_rotation_str, rotation_callbk);
sleep(1); cout << "mu=" << ls_info.mu_ << "secure gain = " << secure_gain << "line_approx" << (line_approx ? "true" : "false") << endl;

        //Server Mu
    string mu_server_str("");
    nh_private.param("service_ch_mu" , mu_server_str, string("change_mu") );
    ros::ServiceServer serviceChMu = nh_public->advertiseService(mu_server_str, ch_mu_callbk);

    ros::spin();

    return 0;
}

/* USER FUN IMPL */
void startSubscribers(){
    sub_wrench = nh_public->subscribe(in_topic, 1, contact_force_Callback);
}
void stopSubscribers(){
    sub_wrench.shutdown();
}

/*******************/