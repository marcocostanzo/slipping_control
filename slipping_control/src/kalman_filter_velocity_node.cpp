/*
    ROS node that implements the Kalman Filter (relative velocity mode)

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
#include "ros/package.h"

#include <std_msgs/Float64.h>
#include "std_srvs/SetBool.h"

#include <Helper.h>
#include <Kalman_Filter/Kalman_Filter_RK4.h>
#include "slipping_control_common/LSCombinedStamped.h"
#include "sun_utils/MultiVectorStamped.h"

#include "slipping_control_common/functions.h"


#define KF_DIM_STATE 3
#define KF_DIM_OUT 2
#define KF_DIM_IN 1

#define HEADER_PRINT BOLDYELLOW "[Kalman Filter(velocity)]: " CRESET 

using namespace std;
using namespace TooN;

VEL_SYSTEM_INFO ss_info;
Kalman_Filter_RK4* kf;

Vector<KF_DIM_IN> input_vector; // u = [ generalizedforce ]
Vector<KF_DIM_OUT> y_kf = Zeros;

//Vars
double MIN_GEN_MAX_FORCE;
bool running = false;

/* USER FUN */
void stopFilter();
void startFilter();
void setInitialConditions();
/**********************/

/*ROS CALLBACK*/

void sub_ls_combined(const slipping_control_common::LSCombinedStamped::ConstPtr& msg)
{

    ss_info.cor = msg->cor;

    if( msg->generalized_max_force0 < MIN_GEN_MAX_FORCE ){
        ss_info.f_max_0 = MIN_GEN_MAX_FORCE;
    } else{
        ss_info.f_max_0 = msg->generalized_max_force0;
    }

    if( msg->generalized_max_force1 < MIN_GEN_MAX_FORCE ){
        ss_info.f_max_1 = MIN_GEN_MAX_FORCE;
    } else{
        ss_info.f_max_1 = msg->generalized_max_force1;
    }

    y_kf[0] = msg->generalized_force0;
    y_kf[1] = msg->generalized_force1;
    //input_vector[0] = msg->generalized_force;
    input_vector[0] = y_kf[0] + y_kf[1];

}

/*Set Running callback*/
bool setRunning_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

	if(req.data){

        if(!running){
            startFilter();
		    cout << HEADER_PRINT GREEN "RE-STARTED!" CRESET << endl;
        }

	} else{
        stopFilter();
		cout << HEADER_PRINT YELLOW "Stopped!" CRESET << endl;
    }

    res.success = true;
	return true;
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "kalman_filter");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    /*PARSAMS*/
    double Hz;
    nh_private.param("hz" , Hz, 500.0 );
    nh_private.param( "Io" , ss_info.Io, 1.4E-3 );
    nh_private.param( "Mo" , ss_info.Mo, 0.35 );
    nh_private.param( "b" , ss_info.b, 0.0 );
    nh_private.param( "beta_o2" , ss_info.beta_o2, 0.07 );
    nh_private.param( "beta_o3" , ss_info.beta_o3, 0.07 );
    nh_private.param( "sigma_02" , ss_info.sigma_02, 1.0E2 );
    nh_private.param( "sigma_03" , ss_info.sigma_03, 1.0E2 );
    nh_private.param( "min_gen_max_force" , MIN_GEN_MAX_FORCE, 0.001 );

    string ls_combined_tipic_str("");
    nh_private.param( "ls_combined_tipic" , ls_combined_tipic_str, string("ls_combined") );

    string extimated_velocity_topic_str("");
    nh_private.param( "extimated_velocity_topic" , extimated_velocity_topic_str, string("extimated_vel") );
    string extimated_state_topic_str("");
    nh_private.param( "extimated_state_topic" , extimated_state_topic_str, string("extimated_state") );
    string extimated_measure_topic_str("");
    nh_private.param( "extimated_measure_topic" , extimated_measure_topic_str, string("extimated_measure") );

    string set_running_service_str("");
    nh_private.param("set_running_service" , set_running_service_str, string("set_running") );
    /******************/

    string base_path("");
    base_path = ros::package::getPath("slipping_control");
    base_path += "/KF_VELOCITY";
    
    string path = base_path + "/W.txt";
    Matrix<KF_DIM_STATE,KF_DIM_STATE> W = readFileM(path, KF_DIM_STATE, KF_DIM_STATE);
    cout << HEADER_PRINT << "W = " << endl << W << endl;
    W = W * pow(1.0/Hz , 2.0);

    path = base_path + "/V.txt";
    Matrix<KF_DIM_OUT,KF_DIM_OUT> V = readFileM(path, KF_DIM_OUT, KF_DIM_OUT);
    cout << HEADER_PRINT << "V = " << endl << V << endl;

    kf = new Kalman_Filter_RK4(   
                            Zeros(KF_DIM_STATE), 
                            W, 
                            KF_DIM_IN, 
                            KF_DIM_OUT, 
                            boost::bind( vel_sys_f_fcn_cont, _1, _2, boost::ref(ss_info) ), 
                            boost::bind( vel_sys_FF_fcn_cont, _1, _2, boost::ref(ss_info) ),
                            boost::bind( vel_sys_h_fcn, _1, _2, boost::ref(ss_info) ),
                            boost::bind( vel_sys_HH_fcn, _1, _2, boost::ref(ss_info) ), 
                            1.0/Hz);

    ros::ServiceServer serviceSetRunning = nh_public.advertiseService(set_running_service_str, setRunning_callbk);

    //Subs
    ros::Subscriber subLSCombined = nh_public.subscribe(ls_combined_tipic_str, 1, sub_ls_combined);
    
    //Pubs
    ros::Publisher pubExtVel = nh_public.advertise<std_msgs::Float64>(extimated_velocity_topic_str, 1);
    ros::Publisher pubExtState = nh_public.advertise<sun_utils::MultiVectorStamped>(extimated_state_topic_str, 1);
    ros::Publisher pubExtMeasure = nh_public.advertise<sun_utils::MultiVectorStamped>(extimated_measure_topic_str, 1);

    //Init Pub Mex
    std_msgs::Float64 ext_vel_msg;
    sun_utils::MultiVectorStamped ext_state_msg;
    ext_state_msg.data.data.resize(KF_DIM_STATE);
    sun_utils::MultiVectorStamped ext_measure_msg;
    ext_measure_msg.data.data.resize(KF_DIM_OUT);
    
    Vector<KF_DIM_OUT> y_hat_k_k1 = Zeros;
    
    ros::Rate loop_rate(Hz);
    while(ros::ok()){

        ros::spinOnce();
        if(!running){
            loop_rate.sleep();
            continue;
        }

        y_hat_k_k1 = kf->apply( y_kf, input_vector,  W, V );

        //Fill Msgs
        ext_vel_msg.data = kf->get_state()[0];
        ext_state_msg.data.data[0] = kf->get_state()[0];
        ext_state_msg.data.data[1] = kf->get_state()[1];
        ext_state_msg.data.data[2] = kf->get_state()[2];
        ext_measure_msg.data.data[0] = y_hat_k_k1[0];
        ext_measure_msg.data.data[1] = y_hat_k_k1[1];
        ext_measure_msg.header.stamp = ros::Time::now();
        ext_state_msg.header.stamp = ext_measure_msg.header.stamp;

        /*Security check*/
        if(     isnan(kf->get_state()[0]) || isnan(kf->get_state()[1]) || isnan(kf->get_state()[2]) )
        {
                cout << HEADER_PRINT << BOLDRED "NANs... EXIT..." CRESET << endl;
                ext_vel_msg.data = 0.0;
                pubExtVel.publish(ext_vel_msg);
                loop_rate.sleep();
                exit(-1);
        }

        //Publish
        pubExtVel.publish(ext_vel_msg);
        pubExtState.publish(ext_state_msg);
        pubExtMeasure.publish(ext_measure_msg);

        loop_rate.sleep();
        loop_rate.reset(); //<== needed?

    }

    return 0;
}

/* USER FUN IMPL */

void stopFilter(){
    kf->reset();
    running = false;
}
void startFilter(){
    kf->reset();
    setInitialConditions();
    running = true;
}

void setInitialConditions(){
    
    kf->reset();
    kf->set_state( makeVector( 0.0, y_kf[0]/ss_info.sigma_02, y_kf[0]/ss_info.sigma_03 ) ) ;
    kf->setPrecInput( input_vector );

}

/*******************/