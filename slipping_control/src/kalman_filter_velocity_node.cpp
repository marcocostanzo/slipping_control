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
#include "slipping_control_common/VirtualCORStamped.h"
#include "slipping_control_common/MaxForcesStamped.h"
#include "sun_utils/MultiVector.h"

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

double MIN_GEN_MAX_FORCE;

/* USER FUN */

void setInitialConditions();
/**********************/

/*ROS CALLBACK*/

//Sub
double cor0, cor1;
void cor0_CB( const slipping_control_common::VirtualCORStamped::ConstPtr& msg ){

    cor0 = msg->cor.virtual_cor;
    ss_info.cor = (cor0+cor1)/2.0;

}

void cor1_CB( const slipping_control_common::VirtualCORStamped::ConstPtr& msg ){

    cor1 = msg->cor.virtual_cor;
    ss_info.cor = (cor0+cor1)/2.0;

}

void max_generalized_force0_CB( const slipping_control_common::MaxForcesStamped::ConstPtr& msg){

    if( msg->forces.generalized_max_force < MIN_GEN_MAX_FORCE ){
        ss_info.f_max_0 = MIN_GEN_MAX_FORCE;
    } else{
        ss_info.f_max_0 = msg->forces.generalized_max_force;
    }

}

void max_generalized_force1_CB( const slipping_control_common::MaxForcesStamped::ConstPtr& msg){
    
    if( msg->forces.generalized_max_force < MIN_GEN_MAX_FORCE ){
        ss_info.f_max_1 = MIN_GEN_MAX_FORCE;
    } else{
        ss_info.f_max_1 = msg->forces.generalized_max_force;
    }

}

void generalized_force0_CB( const std_msgs::Float64::ConstPtr& msg){
    y_kf[0] = msg->data;
    input_vector[0] = y_kf[0] + y_kf[1];
}

void generalized_force1_CB( const std_msgs::Float64::ConstPtr& msg){
    y_kf[1] = msg->data;
    input_vector[0] = y_kf[0] + y_kf[1];
}


/*Pause callback*/
bool paused = true;
void stopFilter(){
    kf->reset();
    paused = true;
}
void startFilter(){
    kf->reset();
    setInitialConditions();
    paused = false;
}
bool pause_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

	if(req.data){

        //stopFilter();
		//cout << HEADER_PRINT YELLOW "PAUSED!" CRESET << endl;

	} else{
        if(paused){
            startFilter();
		    cout << HEADER_PRINT GREEN "RE-STARTED!" CRESET << endl;
        }
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

    string cor0_tipic_str("");
    nh_private.param( "cor0_topic" , cor0_tipic_str, string("finger0/cor") );
    string cor1_tipic_str("");
    nh_private.param( "cor1_topic" , cor1_tipic_str, string("finger1/cor") );
    string max_gen_force0_tipic_str("");
    nh_private.param( "max_gen_force0_topic" , max_gen_force0_tipic_str, string("finger0/max_gen_force") );
    string max_gen_force1_tipic_str("");
    nh_private.param( "max_gen_force1_topic" , max_gen_force1_tipic_str, string("finger1/max_gen_force") );
    string gen_force0_tipic_str("");
    nh_private.param( "gen_force0_topic" , gen_force0_tipic_str, string("finger0/gen_force") );
    string gen_force1_tipic_str("");
    nh_private.param( "gen_force1_topic" , gen_force1_tipic_str, string("finger1/gen_force") );

    string extimated_velocity_topic_str("");
    nh_private.param( "extimated_velocity_topic" , extimated_velocity_topic_str, string("extimated_vel") );
    string extimated_state_topic_str("");
    nh_private.param( "extimated_state_topic" , extimated_state_topic_str, string("extimated_state") );
    string extimated_measure_topic_str("");
    nh_private.param( "extimated_measure_topic" , extimated_measure_topic_str, string("extimated_measure") );

    string pause_service_str("");
    nh_private.param("pause_service" , pause_service_str, string("pause") );
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

    ros::ServiceServer servicePause = nh_public.advertiseService(pause_service_str, pause_callbk);

    //Subs
    ros::Subscriber subCOR0 = nh_public.subscribe(cor0_tipic_str, 1, cor0_CB);
    ros::Subscriber subCOR1 = nh_public.subscribe(cor1_tipic_str, 1, cor1_CB);
    ros::Subscriber subMax_Gen_Force0 = nh_public.subscribe(max_gen_force0_tipic_str, 1, max_generalized_force0_CB);
    ros::Subscriber subMax_Gen_Force1 = nh_public.subscribe(max_gen_force1_tipic_str, 1, max_generalized_force1_CB);
    ros::Subscriber subGen_Force0 = nh_public.subscribe(gen_force0_tipic_str, 1, generalized_force0_CB);
    ros::Subscriber subGen_Force1 = nh_public.subscribe(gen_force1_tipic_str, 1, generalized_force1_CB);
    
    //Pubs
    ros::Publisher pubExtVel = nh_public.advertise<std_msgs::Float64>(extimated_velocity_topic_str, 1);
    ros::Publisher pubExtState = nh_public.advertise<sun_utils::MultiVector>(extimated_state_topic_str, 1);
    ros::Publisher pubExtMeasure = nh_public.advertise<sun_utils::MultiVector>(extimated_measure_topic_str, 1);

    //Init Pub Mex
    std_msgs::Float64 ext_vel_msg;
    sun_utils::MultiVector ext_state_msg;
    ext_state_msg.data.resize(KF_DIM_STATE);
    sun_utils::MultiVector ext_measure_msg;
    ext_measure_msg.data.resize(KF_DIM_OUT);
    
    Vector<KF_DIM_OUT> y_hat_k_k1 = Zeros;
    
    ros::Rate loop_rate(Hz);
    while(ros::ok()){

        ros::spinOnce();
        if(paused){
            loop_rate.sleep();
            continue;
        }

        y_hat_k_k1 = kf->apply( y_kf, input_vector,  W, V );

        //Fill Msgs
        ext_vel_msg.data = kf->get_state()[0];
        ext_state_msg.data[0] = kf->get_state()[0];
        ext_state_msg.data[1] = kf->get_state()[1];
        ext_state_msg.data[2] = kf->get_state()[2];
        ext_measure_msg.data[0] = y_hat_k_k1[0];
        ext_measure_msg.data[1] = y_hat_k_k1[1];

        /*Security check*/
        if(     isnan(kf->get_state()[0]) || isnan(kf->get_state()[1]) || isnan(kf->get_state()[2]) ){
                cout << HEADER_PRINT << BOLDRED "NANs... EXIT..." CRESET << endl;
                exit(-1);
            }

        //Publish
        pubExtVel.publish(ext_vel_msg);
        pubExtState.publish(ext_state_msg);
        pubExtMeasure.publish(ext_measure_msg);

        loop_rate.sleep();

    }

    return 0;
}

/* USER FUN IMPL */
void setInitialConditions(){
    
    kf->reset();
    kf->set_state( makeVector( 0.0, y_kf[0]/ss_info.sigma_02, y_kf[0]/ss_info.sigma_03 ) ) ;

}


/*******************/