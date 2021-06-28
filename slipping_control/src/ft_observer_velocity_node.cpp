/*
    ROS node that implements the Observer for the relative velocity 1DOF Luenberger

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
#include "ros/package.h"

#include <sun_ros_msgs/Float64Stamped.h>
#include "std_srvs/SetBool.h"

#include <Helper.h>
#include "slipping_control_msgs/LSCombinedStamped.h"
#include "sun_ros_msgs/MultiVectorStamped.h"

#include "slipping_control_common/functions.h"

#include "sun_systems_lib/Discretization/RK4.h"
#include "sun_systems_lib/Continuous/Continuous_System.h"
#include "sun_systems_lib/Observers/Observer_SS_Incapsuler.h"
#include "sun_systems_lib/Continuous/Continuous_Luenberger_Observer.h"


#define OBS_DIM_STATE 2
#define OBS_DIM_OUT 1
#define OBS_DIM_IN 1

#define HEADER_PRINT BOLDYELLOW "[Observer(1DOF)]: " CRESET 

using namespace std;
using namespace TooN;

VEL_SYSTEM_INFO ss_info;
// sun::Observer_Interface* observer;
sun::SS_Interface* ss_observer;

Vector<OBS_DIM_IN> input_vector; // u = [ generalizedforce ]
Vector<OBS_DIM_OUT> y_kf = Zeros;

//Vars
double MIN_GEN_MAX_FORCE;
bool running = false;

/* USER FUN */
void stopObserver();
void startObserver();
void setInitialConditions();
/**********************/

/*ROS CALLBACK*/
double betaA;
void sub_ls_combined(const slipping_control_msgs::LSCombinedStamped::ConstPtr& msg)
{

    ss_info.cor = msg->cor;

    if( msg->generalized_max_force < MIN_GEN_MAX_FORCE ){
        ss_info.f_max_0 = MIN_GEN_MAX_FORCE;
    } else{
        ss_info.f_max_0 = msg->generalized_max_force;
    }

    y_kf[0] = msg->generalized_force;
    input_vector[0] = msg->generalized_force;

    ss_info.beta_o2 = betaA;
    

    // std::cout << "sigma_1: " << ss_info.beta_o2 << "\n";

}

/*Set Running callback*/
bool setRunning_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

	if(req.data){

        if(!running){
            startObserver();
		    cout << HEADER_PRINT GREEN "RE-STARTED!" CRESET << endl;
        }

	} else{
        stopObserver();
		cout << HEADER_PRINT YELLOW "Stopped!" CRESET << endl;
    }

    res.success = true;
	return true;
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "ft_observer");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    /*PARSAMS*/
    double Hz;
    nh_private.param("hz" , Hz, 500.0 );
    nh_private.param( "Io" , ss_info.Io, 1.4E-3 );
    nh_private.param( "Mo" , ss_info.Mo, 0.35 );
    nh_private.param( "b" , ss_info.b, 0.0 );
    nh_private.param( "betaA" , betaA, 0.07 );
    // nh_private.param( "beta_o" , ss_info.beta_o2, 0.07 );
    nh_private.param( "sigma_0" , ss_info.sigma_02, 1.0E2 );
    nh_private.param( "min_gen_max_force" , MIN_GEN_MAX_FORCE, 0.001 );
    double l;
    nh_private.param( "l" , l, 1000.0 );

    string ls_combined_topic_str("");
    nh_private.param( "ls_combined_topic" , ls_combined_topic_str, string("ls_combined") );

    string estimated_velocity_topic_str("");
    nh_private.param( "estimated_velocity_topic" , estimated_velocity_topic_str, string("estimated_vel") );
    string estimated_state_topic_str("");
    nh_private.param( "estimated_state_topic" , estimated_state_topic_str, string("estimated_state") );
    string estimated_measure_topic_str("");
    nh_private.param( "estimated_measure_topic" , estimated_measure_topic_str, string("estimated_measure") );

    string set_running_service_str("");
    nh_private.param("set_running_service" , set_running_service_str, string("set_running") );
    /******************/

    Matrix<> L = Zeros(OBS_DIM_STATE, OBS_DIM_OUT);
    L(0,0) = l;
    cout << HEADER_PRINT << " L= " << endl << L << endl;

    ss_observer = 
        new sun::RK4(
            sun::Continuous_System( 
                OBS_DIM_STATE, 
                OBS_DIM_OUT, 
                OBS_DIM_IN, 
                boost::bind( obs_vel_sys_1dof_f_fcn_cont, _1, _2, boost::ref(ss_info), l ), 
                boost::bind( obs_vel_sys_1dof_h_fcn, _1, _2, boost::ref(ss_info) ), 
                boost::bind( obs_vel_sys_1dof_FF_fcn_cont, _1, _2, boost::ref(ss_info), l ), 
                boost::bind( obs_vel_sys_HH_fcn, _1, _2, boost::ref(ss_info) ) 
            ), 
            1.0/Hz, 
            false //<- true to use u_n_1_ in RK4
        );

    ros::ServiceServer serviceSetRunning = nh_public.advertiseService(set_running_service_str, setRunning_callbk);

    //Subs
    ros::Subscriber subLSCombined = nh_public.subscribe(ls_combined_topic_str, 1, sub_ls_combined);
    
    //Pubs
    ros::Publisher pubExtVel = nh_public.advertise<sun_ros_msgs::Float64Stamped>(estimated_velocity_topic_str, 1);
    ros::Publisher pubExtState = nh_public.advertise<sun_ros_msgs::MultiVectorStamped>(estimated_state_topic_str, 1);
    ros::Publisher pubExtMeasure = nh_public.advertise<sun_ros_msgs::MultiVectorStamped>(estimated_measure_topic_str, 1);

    //Init Pub Mex
    sun_ros_msgs::Float64Stamped ext_vel_msg;
    sun_ros_msgs::MultiVectorStamped ext_state_msg;
    ext_state_msg.data.data.resize(OBS_DIM_STATE);
    sun_ros_msgs::MultiVectorStamped ext_measure_msg;
    ext_measure_msg.data.data.resize(OBS_DIM_OUT);
    
    Vector<OBS_DIM_OUT> y_hat_k_k1 = Zeros;
    
    ros::Rate loop_rate(Hz);
    while(ros::ok()){

        ros::spinOnce();
        if(!running){
            loop_rate.sleep();
            continue;
        }

        //observer->pushPrecInput(); <-- expose system_ in Observer_SS_Incapsuler/Kalman_Filter to implement this. remember to set true in the constructor RK4
        // y_hat_k_k1 = observer->obs_apply( input_vector, y_kf  );
        y_hat_k_k1 = ss_observer->apply( y_kf  );

        //Fill Msgs
        ext_vel_msg.data = ss_observer->getState()[0];
        ext_state_msg.data.data[0] = ss_observer->getState()[0];
        ext_state_msg.data.data[1] = ss_observer->getState()[1];
        ext_state_msg.data.data[2] = ss_observer->getState()[2];
        ext_measure_msg.data.data[0] = y_hat_k_k1[0];
        ext_measure_msg.data.data[1] = y_hat_k_k1[1];
        ext_measure_msg.header.stamp = ros::Time::now();
        ext_state_msg.header.stamp = ext_measure_msg.header.stamp;
        ext_vel_msg.header.stamp = ext_measure_msg.header.stamp;

        /*Security check*/
        if(     isnan(ss_observer->getState()[0]) || isnan(ss_observer->getState()[1]) || isnan(ss_observer->getState()[2]) )
        {
                //cout << HEADER_PRINT << BOLDRED "NANs... EXIT..." CRESET << endl;
                cout << HEADER_PRINT << BOLDRED "NANs... RESET! ..." CRESET << endl;
                ext_vel_msg.data = 0.0;
                pubExtVel.publish(ext_vel_msg);
                loop_rate.sleep();
                setInitialConditions();
                //exit(-1);
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

void stopObserver(){
    ss_observer->reset();
    running = false;
}
void startObserver(){
    ss_observer->reset();
    setInitialConditions();
    running = true;
}

void setInitialConditions(){
    
    ss_observer->reset();
    ss_observer->setState( makeVector( 0.0, y_kf[0]/ss_info.sigma_02 ) ) ;
    //observer->setPrecInput( input_vector ); <-- expose system_ in Observer_SS_Incapsuler/Kalman_Filter to implement this

}

/*******************/