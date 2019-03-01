/*
    ROS node of high level controller

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
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "slipping_control_common/Grasp.h"
#include "geometry_msgs/WrenchStamped.h"
#include "slipping_control_common/Pivoting.h"
#include "slipping_control_common/FnsParams.h"

#include <slipping_control_common/PivotingAction.h>
#include <slipping_control_common/GraspingAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>

//#include "actionlib/client/simple_client_goal_state.h"



#include "Helper.h"
#include "slipping_control_common/pivoting_error_codes.h"

#define HEADER_PRINT BOLDYELLOW "[Smart Control]: " CRESET

/*STATES*/
#define STATE_UNDEFINED    -1
#define STATE_HOME          0
#define STATE_GRASPING      1
#define STATE_GRASPED       2
#define STATE_FNS           3
#define STATE_FNS_FND       4
#define STATE_REMOVE_FND    5
#define STATE_PIVOTING      6

/*INPUTS*/
bool input_homing();
bool input_remove_bias();
bool input_grasp(double f);
bool input_apply_force(double f);
bool input_apply_fns(bool control_rotation);
bool input_apply_fns_fnd();
bool input_pivoting(double ang, int& error);

int state = STATE_UNDEFINED;

using namespace std;

/*PARAMS*/

//grasp params
//double CONTACT_FN_FORCE = 0.0, CONTACT_FORCE_THR = 0.0, FORCE_TIME_RATIO = 0.0, RATE = 500.0;

//remove fnd params
double sec_remove_fnd, remove_fnd_initial_fnd = 0.0;
double REMOVE_FND_RATE, REMOVE_FND_TIME_CONST;


//Input topics
string force_command_topic_str("");
string fns_topic_str("");
string fnd_topic_str("");
//string pivoting_topic_str("");
//string wrench_topic_str("");

//Client services
string pause_static_service_client_str("");
string fns_params_service_client_str("");
string pause_dyn_controller_service_client_str("");
string pause_force_service_client_str("");
string homing_gripper_service_client_str("");
string remove_bias_0_service_client_str("");
string remove_bias_1_service_client_str("");
string pause_kf_service_client_str("");

//Action client
string grasping_action_client_str("");
string pivoting_action_client_str("");

//Server services
string homing_service_server_str("");
string remove_bias_service_server_str("");
string grasp_service_server_str("");
string apply_force_service_server_str("");
string apply_fns_service_server_str("");
string apply_fnd_service_server_str("");
string pivoting_service_server_str("");

/*Publisher*/
ros::Publisher pub_force;

/*Subscribers*/
ros::Subscriber sub_fns;
ros::Subscriber sub_fnd;
//ros::Subscriber sub_pivoting;

/*Clients*/
ros::ServiceClient client_homing_gripper;
ros::ServiceClient client_pause_static;
ros::ServiceClient client_fns_param;
ros::ServiceClient client_pause_dyn_controller;
ros::ServiceClient client_pause_force;
ros::ServiceClient client_remove_bias_0;
ros::ServiceClient client_remove_bias_1;
ros::ServiceClient client_pause_kf;

//actionclient
actionlib::SimpleActionClient<slipping_control_common::PivotingAction>* action_client_pivoting;
actionlib::SimpleActionClient<slipping_control_common::GraspingAction>* action_client_grasping;

/* USER FUN */
bool pauseStaticControl(bool b, bool control_rotation);
bool pauseDynControl(bool b);
bool pauseKf(bool b);
bool pauseForceControl(bool b);
bool pauseAll();
bool homing();  //(pause all)
/*
bool state_homing();
bool state_remove_bias();
bool state_grasp(double force);
bool state_apply_force(double force);
bool apply_fns();
bool apply_fnd();
bool state_pivoting(double ang, int& error);
bool pivoting_end( int prec_state );
bool abortPivoting();
*/

string getStateStr();
string getStateStr(int s);
void printState();
/**********************/

/*ROS CALLBACK*/

/*==============================SWITCH LOGIC ON CALLBACK*/

std_msgs::Float64 fn;

double fns = 0.0, fnd = 0.0, fns_fnd = 0.0;
void fns_Callback (const std_msgs::Float64::ConstPtr& msg) {

    switch (state){
        case STATE_FNS:{
            fns = msg->data;
            fnd = 0.0;
            fns_fnd = fns;
            fn.data = fns_fnd;
            pub_force.publish(fn);
            return;
        }
        case STATE_FNS_FND:{
            fns = msg->data;
            fns_fnd = fns + fnd;
            fn.data = fns_fnd;
            pub_force.publish(fn);
            return;
        }
        case STATE_REMOVE_FND:{
            //cout << "Removing fnd..." << endl;
            fns = msg->data;
            fns_fnd = fns + remove_fnd_initial_fnd * exp(-sec_remove_fnd/REMOVE_FND_TIME_CONST);
            cout << HEADER_PRINT << sec_remove_fnd << endl;
            cout << HEADER_PRINT << remove_fnd_initial_fnd * exp(-sec_remove_fnd/REMOVE_FND_TIME_CONST) << endl;
            fn.data = fns_fnd;
            pub_force.publish(fn);
            return;
        }
            
        default:{
            fns = 0.0;
            fnd = 0.0;
            fns_fnd = 0.0;
        }
    }

}

void fnd_Callback (const std_msgs::Float64::ConstPtr& msg) {

    if( fns == 0.0 )
        return;

    switch (state){
        case STATE_FNS:{
            fnd = 0.0;
            return;
        }
        case STATE_FNS_FND:{
            fnd = msg->data;
            fns_fnd = fns + fnd;
            fn.data = fns_fnd;
            pub_force.publish(fn);
            return;
        }
        case STATE_REMOVE_FND:{
            fnd = msg->data;
            fns_fnd = fns + remove_fnd_initial_fnd * exp(-sec_remove_fnd/REMOVE_FND_TIME_CONST);
            fn.data = fns_fnd;
            pub_force.publish(fn);
            return;
        }
            
        default:{
            fns = 0.0;
            fnd = 0.0;
            fns_fnd = 0.0;
        }
    }

}
/*
void pivoting_Callback (const std_msgs::Float64::ConstPtr& msg) {

    if( state == STATE_PIVOTING){
        pub_force.publish(msg);
    }

}
*/
/* END ============================ SWITCH LOGIC ON CALLBACK*/

/*Service Homing*/
bool homing_srv_Callbk(std_srvs::Trigger::Request  &req, 
   		 		std_srvs::Trigger::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE HOMING..." << endl;

    res.success = input_homing();

    cout << HEADER_PRINT << "SERVICE HOMING END" << endl;
    printState();

    return true;                    
}

/*Service Remove Bias*/
bool remove_bias_srv_Callbk(std_srvs::Trigger::Request  &req, 
   		 		std_srvs::Trigger::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE REMOVE_BIAS..." << endl;

    res.success = input_remove_bias();

    cout << HEADER_PRINT << "SERVICE REMOVE_BIAS END" << endl;
    printState();

    return true;                    
}

/*Service GRASP*/
bool grasp_srv_Callbk(slipping_control_common::Grasp::Request  &req, 
   		 		slipping_control_common::Grasp::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE GRASP..." << endl;

    res.success = input_grasp(req.force);

    cout << HEADER_PRINT << "SERVICE GRASP END" << endl;
    printState();

    return true;                    
}

/*Service APPLY FORCE*/
bool apply_force_srv_Callbk(slipping_control_common::Grasp::Request  &req, 
   		 		slipping_control_common::Grasp::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE APPLY_FORCE..." << endl;

    res.success = input_apply_force(req.force);

    cout << HEADER_PRINT << "SERVICE APPLY_FORCE END" << endl;
    printState();

    return true;                    
}

/*Service APPLY FNS*/

bool apply_fns_srv_Callbk(slipping_control_common::FnsParams::Request  &req, 
   		 		slipping_control_common::FnsParams::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE APPLY_FNS..." << endl;

    res.success = input_apply_fns(req.control_rotation);

    cout << HEADER_PRINT << "SERVICE APPLY_FNS END" << endl;
    printState();

    return true;                    
}

/*Service APPLY FND*/

bool apply_fnd_srv_Callbk(std_srvs::Trigger::Request  &req, 
   		 		std_srvs::Trigger::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE APPLY_FND..." << endl;

    res.success = input_apply_fns_fnd();

    cout << HEADER_PRINT << "SERVICE APPLY_FND END" << endl;
    printState();

    return true;                    
}


/*Service Pivoting*/

bool pivoting_srv_Callbk(slipping_control_common::Pivoting::Request  &req, 
   		 		slipping_control_common::Pivoting::Response &res){

    printState();
    cout << HEADER_PRINT << "INIT SERVICE PIVOTING..." << endl;

    int tmp_error;
    res.success = input_pivoting(req.ang, tmp_error);
    res.error = tmp_error;

    cout << HEADER_PRINT << "SERVICE PIVOTING END" << endl;
    printState();

    return true;                    
}


int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "smart_control");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    /*PARAMS*/

    //MISC PARAMS
    nh_private.param("remove_fnd_rate" , REMOVE_FND_RATE, 200.0  );
    nh_private.param("remove_fnd_time_const" , REMOVE_FND_TIME_CONST, 1.0  );

    //in topics
    nh_private.param("force_command_topic" , force_command_topic_str, string("force_command_topic") );
    nh_private.param("fns_topic" , fns_topic_str, string("fns_topic") );
    nh_private.param("fnd_topic" , fnd_topic_str, string("fnd_topic") );
    //nh_private.param("pivoting_topic" , pivoting_topic_str, string("pivoting_topic") );

    //service as client
    nh_private.param("pause_static_service" , pause_static_service_client_str, string("pause_static_service") );
    nh_private.param("fns_params_service" , fns_params_service_client_str, string("fns_params_service") );
    nh_private.param("pause_dyn_controller_service" , pause_dyn_controller_service_client_str, string("pause_dyn_controller_service") );
    nh_private.param("pause_force_service" , pause_force_service_client_str, string("pause_force_service") );
    nh_private.param("homing_gripper_service" , homing_gripper_service_client_str, string("homing_gripper_service") );
    nh_private.param("remove_bias_0_service" , remove_bias_0_service_client_str, string("remove_bias_0_service") );
    nh_private.param("remove_bias_1_service" , remove_bias_1_service_client_str, string("remove_bias_1_service") );
    nh_private.param("pause_kf_service" , pause_kf_service_client_str, string("pause_kf_service") );

    //action_clients
    nh_private.param("action_client_grasping" , grasping_action_client_str, string("action_client_grasping") );
    nh_private.param("action_client_pivoting" , pivoting_action_client_str, string("action_client_pivoting") );

    //service as server
    nh_private.param("service_remove_bias" , remove_bias_service_server_str, string("service_remove_bias") );
    nh_private.param("service_grasp" , grasp_service_server_str, string("service_grasp") );
    nh_private.param("service_apply_force" , apply_force_service_server_str, string("service_apply_force") );
    nh_private.param("service_apply_fns" , apply_fns_service_server_str, string("service_apply_fns") );
    nh_private.param("service_apply_fnd" , apply_fnd_service_server_str, string("service_apply_fnd") );
    nh_private.param("service_homing" , homing_service_server_str, string("service_homing") );
    nh_private.param("service_pivoting" , pivoting_service_server_str, string("service_pivoting") );

    /******************/

    pub_force = nh_public.advertise<std_msgs::Float64>(force_command_topic_str, 1);

    sub_fns = nh_public.subscribe(fns_topic_str, 1, fns_Callback);
    sub_fnd  = nh_public.subscribe(fnd_topic_str, 1, fnd_Callback);
    //sub_pivoting = nh_public.subscribe(pivoting_topic, 1, pivoting_Callback);

    //Service clients
    client_pause_static = nh_public.serviceClient<std_srvs::SetBool>(pause_static_service_client_str);
    client_fns_param = nh_public.serviceClient<slipping_control_common::FnsParams>(fns_params_service_client_str);
    client_pause_dyn_controller = nh_public.serviceClient<std_srvs::SetBool>(pause_dyn_controller_service_client_str);
    client_pause_force = nh_public.serviceClient<std_srvs::SetBool>(pause_force_service_client_str);
    client_homing_gripper = nh_public.serviceClient<std_srvs::Empty>(homing_gripper_service_client_str);
    client_remove_bias_0 = nh_public.serviceClient<std_srvs::Empty>(remove_bias_0_service_client_str);
    client_remove_bias_1 = nh_public.serviceClient<std_srvs::Empty>(remove_bias_1_service_client_str);
    client_pause_kf = nh_public.serviceClient<std_srvs::SetBool>(pause_kf_service_client_str);

    // create the action client
    // true causes the client to spin its own thread
    action_client_grasping = new actionlib::SimpleActionClient<slipping_control_common::GraspingAction>(grasping_action_client_str, true);
    action_client_grasping->waitForServer();
    action_client_pivoting = new actionlib::SimpleActionClient<slipping_control_common::PivotingAction>(pivoting_action_client_str, true);
    action_client_pivoting->waitForServer();
    

    ros::ServiceServer the_service_remove_bias = nh_public.advertiseService(remove_bias_service_server_str, remove_bias_srv_Callbk);
    ros::ServiceServer the_service_grasp = nh_public.advertiseService(grasp_service_server_str, grasp_srv_Callbk);
    ros::ServiceServer the_service_apply_force = nh_public.advertiseService(apply_force_service_server_str, apply_force_srv_Callbk);
    ros::ServiceServer the_service_apply_fns = nh_public.advertiseService(apply_fns_service_server_str, apply_fns_srv_Callbk);
    ros::ServiceServer the_service_apply_fnd = nh_public.advertiseService(apply_fnd_service_server_str, apply_fnd_srv_Callbk);
    ros::ServiceServer the_service_homing = nh_public.advertiseService(homing_service_server_str, homing_srv_Callbk);
    ros::ServiceServer the_service_pivoting = nh_public.advertiseService(pivoting_service_server_str, pivoting_srv_Callbk);

    ros::spin();

    return 0;
}

/* USER FUN IMPL */
bool sendSetPauseSrv(ros::ServiceClient client, bool b_pause, string msg){

    std_srvs::SetBool setBoolmsg;
    setBoolmsg.request.data = b_pause;

    string what = b_pause ? "STOP" : "START";
    string what_ing = b_pause ? "STOPPING" : "STARTING";

    cout << HEADER_PRINT << what_ing << " " << msg << endl;

    bool b_out = client.call(setBoolmsg);
    b_out = b_out && setBoolmsg.response.success;

    if(b_out){
        cout << HEADER_PRINT << what << " " << msg << GREEN " OK" CRESET << endl;
    } else {
        cout << HEADER_PRINT RED "Error during " << BOLDRED << what << CRESET RED << " " << msg << CRESET << endl;
    }

    return b_out;

}

bool pauseForceControl(bool b){
    return sendSetPauseSrv(client_pause_force , b , "Force Controller");
}

bool pauseStaticControl(bool b, bool control_rotation){

    slipping_control_common::FnsParams fns_params;
    fns_params.request.control_rotation = control_rotation;
    bool b_set_rotation = client_fns_param.call(fns_params);
    b_set_rotation = b_set_rotation && fns_params.response.success;
    if(b_set_rotation){
        cout << HEADER_PRINT << (control_rotation ? "ACTIVE CONTROL ROTATION" : "DEACTIVATE CONTROL ROTATION") << GREEN " OK" CRESET << endl;
    } else{
        cout << HEADER_PRINT << BOLDRED "ERROR IN " << (control_rotation ? "ACTIVE CONTROL ROTATION" : "DEACTIVATE CONTROL ROTATION") << CRESET << endl;
        state = STATE_UNDEFINED;
        return false;
    }

    return sendSetPauseSrv(client_pause_static , b , "Static Controller");
}

bool pauseDynControl(bool b){
    return sendSetPauseSrv(client_pause_dyn_controller , b , "Dynamic Controller");
}

bool pauseKf(bool b){
    return sendSetPauseSrv(client_pause_kf , b , "Kalman Filter");
}

bool pauseAll(){
    return (pauseDynControl(true) && pauseStaticControl(true, true) && pauseForceControl(true));
}

bool homing(){

    usleep(500000);
    std_srvs::Empty emptySrvMsg;
    cout << HEADER_PRINT "Homing..." << endl;
	if(!client_homing_gripper.call(emptySrvMsg)){
		cout << HEADER_PRINT BOLDRED "Error during homing" CRESET << endl;
        return false;
	} else {
        sleep(1);
        cout << HEADER_PRINT "Homing " GREEN "OK (?)" CRESET << endl;
        return true;
    }

}

bool remove_bias(){
    std_srvs::Empty emptyMsg;
    bool b0 = true;
    if(!client_remove_bias_0.call(emptyMsg) ){
        cout << HEADER_PRINT << RED "Error in remove_bias() of finger 0!" CRESET << endl;
        b0 = false;
    }

    bool b1 = true;
    if(!client_remove_bias_1.call(emptyMsg) ){
        cout << HEADER_PRINT << RED "Error in remove_bias() of finger 1!" CRESET << endl;
        b1 = false;
    }

    return b0 && b1;
}

//STATE FUN
string getStateStr(){
    return getStateStr(state);
}
string getStateStr(int s){
    switch (s)
    {
        case STATE_UNDEFINED:{
            return "STATE_UNDEFINED";
        }
        case STATE_HOME:{
            return "STATE_HOME";
        }
        case STATE_GRASPING:{
            return "STATE_GRASPING";
        }
        case STATE_GRASPED:{
            return "STATE_GRASPED";
        }
        case STATE_FNS:{
            return "STATE_FNS";
        }
        case STATE_FNS_FND:{
            return "STATE_FNS_FND";
        }
        case STATE_PIVOTING:{
            return "STATE_PIVOTING";
        }
        default: {
            return "INVALID_STATE";
        }
    }
}

void printState(){
    cout << HEADER_PRINT "STATE = " << getStateStr() << endl;
}

void pErrorInvalidStartState(string fun_str, string valid_states_str){
    cout << HEADER_PRINT << RED "Error in " << fun_str << " state must be " << valid_states_str << " - Actual State " << getStateStr(state) << CRESET << endl;
}

//*****grasp section*****//
// Called once when the goal completes
void grasping_doneCb(const actionlib::SimpleClientGoalState& state_,
            const slipping_control_common::GraspingResultConstPtr& result)
{
    cout << HEADER_PRINT << GREEN << "GRASPING DONE! finisched in state " << state_.toString()  << CRESET << endl;    
}

// Called once when the goal becomes active
void grasping_activeCb()
{
    cout << HEADER_PRINT << GREEN << "GRASPING just went active " << CRESET << endl;
}

// Called every time feedback is received for the goal
void grasping_feedbackCb(const slipping_control_common::GraspingFeedbackConstPtr& feedback)
{
    if(state == STATE_GRASPING){
        std_msgs::Float64 fn_msg;
        fn_msg.data = feedback->force;
        pub_force.publish(fn_msg);
    }

}
int grasp_call_id = 0;
bool doGraspingAction(double f){

    //make sure that force control is active
    if( !pauseForceControl(false) ){
        cout << HEADER_PRINT << BOLDRED "Error in doGraspingAction() - cannot start force control!" CRESET << endl;
        state = STATE_UNDEFINED;
        return false;
    }

    grasp_call_id++;
    int my_grasp_call_id = grasp_call_id;
    slipping_control_common::GraspingGoal goal;
    goal.force = f;
    action_client_grasping->sendGoal(goal, &grasping_doneCb, &grasping_activeCb, &grasping_feedbackCb);
    //action_client_grasping->sendGoal(goal);

    //wait for the action to return
    while(!action_client_grasping->waitForResult(ros::Duration(0.1))){
        ros::spinOnce();
    }

    actionlib::SimpleClientGoalState ac_state = action_client_grasping->getState();
    if(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED && (my_grasp_call_id == grasp_call_id)){

        slipping_control_common::GraspingResultConstPtr grasp_result = action_client_grasping->getResult();
        
        if( grasp_result->success ){

            state = STATE_GRASPED;
            return true;

        } else {
            
            state = STATE_UNDEFINED;
            return false;

        }
    } else {

        cout << HEADER_PRINT << BOLDRED << "Grasping aborted" << CRESET << endl;
        return false;

    }
    
}

bool abortGrasping(){

    if(state == STATE_GRASPING){
        cout << HEADER_PRINT << BOLDYELLOW "Aborting Grasping..." CRESET << endl;
        action_client_grasping->cancelAllGoals();
    }

    return true;

}
//*****grasp section end*****//

//*****apply force section*****//

bool apply_force(double f){

    //make sure that force control is active
    if( !pauseForceControl(false) ){
        cout << HEADER_PRINT << BOLDRED "Error in apply_force() - cannot start force control!" CRESET << endl;
        state = STATE_UNDEFINED;
        return false;
    }

    fn.data = f;
    pub_force.publish(fn);
    return true;
}

//*****apply force section end*****//

//***** remove fnd section *****//

bool remove_fnd(){
    remove_fnd_initial_fnd = fnd;
    state = STATE_REMOVE_FND;

    double total_time = 1.0*REMOVE_FND_TIME_CONST;
    cout << HEADER_PRINT << "Removing Fnd... Duration " << total_time << endl;
    cout << HEADER_PRINT << "remove_fnd_initial_fnd:  " << remove_fnd_initial_fnd << endl;
    cout << HEADER_PRINT << "fns: " << fns << endl;
 
    sec_remove_fnd = 0.0;
    ros::Rate loop_rate(REMOVE_FND_RATE);   
    double remove_fnd_initial_time = ros::Time::now().toSec();
    while(ros::ok() && sec_remove_fnd < total_time){
        sec_remove_fnd = ros::Time::now().toSec() - remove_fnd_initial_time;
        loop_rate.sleep();
        ros::spinOnce();
    }
    if(!ros::ok())
        return false;

    cout << HEADER_PRINT << "Fnd Removed!" << endl;
    cout << HEADER_PRINT << "fns: " << fns << endl;
    
    return true;

}

//***** remove fnd section end*****//

//*****pivoting section*****//
// Called once when the goal completes
void pivoting_doneCb(const actionlib::SimpleClientGoalState& state_,
            const slipping_control_common::PivotingResultConstPtr& result)
{
    cout << HEADER_PRINT << GREEN << "PIVOTING DONE! finisched in state " << state_.toString()  << CRESET << endl;    
}

// Called once when the goal becomes active
void pivoting_activeCb()
{
    cout << HEADER_PRINT << GREEN << "PIVOTING just went active " << CRESET << endl;
}

// Called every time feedback is received for the goal
void pivoting_feedbackCb(const slipping_control_common::PivotingFeedbackConstPtr& feedback)
{
    if(state == STATE_PIVOTING){
        std_msgs::Float64 fn_msg;
        fn_msg.data = feedback->fn;
        pub_force.publish(fn_msg);
    }

}

bool doPivotingAction(double ang, int& error){

    //make sure that force control is active
    if( !pauseForceControl(false) ){
        cout << HEADER_PRINT << BOLDRED "Error in doPivotingAction() - cannot start force control!" CRESET << endl;
        state = STATE_UNDEFINED;
        error = PIV_ERR_CODE_GENERIC;
        return false;
    }

    slipping_control_common::PivotingGoal goal;
    goal.ang = ang;
    action_client_pivoting->sendGoal(goal, &pivoting_doneCb, &pivoting_activeCb, &pivoting_feedbackCb);
    //action_client_grasping->sendGoal(goal);

    //wait for the action to return
    while(!action_client_pivoting->waitForResult(ros::Duration(0.1))){
        ros::spinOnce();
    }

    actionlib::SimpleClientGoalState ac_state = action_client_pivoting->getState();
    if(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED){
        slipping_control_common::PivotingResultConstPtr piv_result = action_client_pivoting->getResult();
        
        if( piv_result->success ){
            error = PIV_ERR_CODE_OK;
            state = STATE_GRASPED;
            return input_apply_fns_fnd();
            //return input_apply_fns();

        } else {

            error = piv_result->error;
            switch(error){
                case PIV_ERR_CODE_OK: {
                    cout << HEADER_PRINT << BOLDRED << "Invalid error code with success = false | code = ERR_CODE_OK" << CRESET << endl;
                    state = STATE_UNDEFINED;
                    return false;
                }
                case PIV_ERR_CODE_ZERO_FN: {
                    cout << HEADER_PRINT << BOLDRED << "Fn zero after pivoting! homing..." CRESET << endl;
                    return input_homing();
                }
                case PIV_ERR_CODE_MAX_TIME: {
                    cout << HEADER_PRINT << YELLOW << "Max Time expired in pivoting..." << CRESET << endl;
                    state = STATE_GRASPED;
                    return input_apply_fns_fnd();
                    //return input_apply_fns();
                }
                case PIV_ERR_CODE_MIN_FN: {
                    cout << HEADER_PRINT << YELLOW << "Min F_grasp in pivoting... pivoting done?" CRESET << endl;
                    state = STATE_GRASPED;
                    return input_apply_fns_fnd();
                    //return input_apply_fns();
                }
                default : {
                    cout << HEADER_PRINT << BOLDRED << "Error code not known... " << error << CRESET << endl;
                    state = STATE_UNDEFINED;
                    return false;
                }

            }
        }
    } else {

        cout << HEADER_PRINT << BOLDRED << "Pivoting aborted" << CRESET << endl;
        error = PIV_ERR_CODE_ABORTED;
        return false;

    }
    
}

bool abortPivoting(){

    if(state == STATE_PIVOTING){
        cout << HEADER_PRINT << BOLDYELLOW "Aborting Pivoting..." CRESET << endl;
        action_client_pivoting->cancelAllGoals();
    }

    return true;

}
//*****grasp section end*****//

//Abort actions
bool abortAllActions(){
    bool b = abortGrasping();
    b = b && abortPivoting();
    return b;
}

/* END ============= USER FUN ================= */

/* ============= STATE TRANSITIONS ================= */

bool input_homing(){
    /*CHECK VALID STATE*/
    abortAllActions();
    pauseAll();
    if(homing()){
        state = STATE_HOME;
        return true;
    } else{
        state = STATE_UNDEFINED;
        return false;
    }
}

bool input_remove_bias(){

    /*CHECK VALID STATE*/
    switch (state){

        case STATE_HOME:{
            if(remove_bias()){
                return true;
            } else {
                state = STATE_UNDEFINED;
                return false;
            }
        }

        default:{
            pErrorInvalidStartState("input_remove_bias()", getStateStr(STATE_HOME));
            return false;
        }
    }

}

bool input_grasp(double f){

    /*CHECK VALID STATE*/
    switch (state){

        case STATE_HOME:
        case STATE_GRASPED:
        {
            state = STATE_GRASPING;
            return doGraspingAction(f);   
        }
        case STATE_GRASPING:{
            abortGrasping();
            state = STATE_GRASPING;
            return doGraspingAction(f);
        }
        case STATE_FNS:{
            pauseStaticControl(true,true);
            state = STATE_GRASPING;
            return doGraspingAction(f);
        }
        case STATE_FNS_FND:{
            pauseDynControl(true);
            pauseStaticControl(true,true);
            state = STATE_GRASPING;
            return doGraspingAction(f);
        }
        case STATE_PIVOTING: {
            abortPivoting();
            state = STATE_GRASPING;
            return doGraspingAction(f);
        }

        default:{
            pErrorInvalidStartState("input_grasp()",            getStateStr(STATE_HOME) 
                                                        + "|" + getStateStr(STATE_GRASPED) 
                                                        + "|" + getStateStr(STATE_GRASPING)
                                                        + "|" + getStateStr(STATE_FNS)
                                                        + "|" + getStateStr(STATE_FNS_FND)
                                                        + "|" + getStateStr(STATE_PIVOTING));
            return false;
        }
    }
    
}

bool input_apply_force(double f){

    /*CHECK VALID STATE*/
    switch (state){

        case STATE_HOME:
        case STATE_GRASPED:
        {
            bool b = apply_force(f);
            state = STATE_GRASPED;
            return b;   
        }
        case STATE_GRASPING:{
            abortGrasping();
            bool b = apply_force(f);
            state = STATE_GRASPED;
            return b;
        }
        case STATE_FNS:{
            pauseStaticControl(true,true);
            bool b = apply_force(f);
            state = STATE_GRASPED;
            return b;
        }
        case STATE_FNS_FND:{
            pauseDynControl(true);
            pauseStaticControl(true,true);
            bool b = apply_force(f);
            state = STATE_GRASPED;
            return b;
        }
        case STATE_PIVOTING:{
            abortPivoting();
            bool b = apply_force(f);
            state = STATE_GRASPED;
            return b;
        }

        default:{
            pErrorInvalidStartState("input_apply_force()",              getStateStr(STATE_HOME) 
                                                                + "|" + getStateStr(STATE_GRASPED) 
                                                                + "|" + getStateStr(STATE_GRASPING)
                                                                + "|" + getStateStr(STATE_FNS)
                                                                + "|" + getStateStr(STATE_FNS_FND)
                                                                + "|" + getStateStr(STATE_PIVOTING));
            return false;
        }
    }

}

bool input_apply_fns(bool control_rotation){

    /*CHECK VALID STATE*/
    switch (state){

        case STATE_GRASPED:
        {
            bool b = pauseStaticControl(false, control_rotation);
            state = STATE_FNS;
            return b;   
        }
        case STATE_FNS:
        {
            cout << HEADER_PRINT << YELLOW "Warn in input_apply_fns() - Already in " << getStateStr() << " refreshing..." CRESET << endl;
            bool b = pauseStaticControl(false, control_rotation);
            state = STATE_FNS;
            return b;   
        }
        case STATE_FNS_FND:{

            pauseStaticControl(false, control_rotation);
            bool b = remove_fnd();
            pauseDynControl(true);  
            b = b && pauseStaticControl(false, control_rotation); //just in case....       

            state = STATE_FNS;
            return b;
        }
        case STATE_PIVOTING: {
            abortPivoting();
            bool b = pauseStaticControl(false, control_rotation);
            state = STATE_FNS;
            return b;
        }

        default:{
            pErrorInvalidStartState("input_apply_fns()",            getStateStr(STATE_GRASPED)
                                                            + "|" + getStateStr(STATE_FNS) 
                                                            + "|" + getStateStr(STATE_FNS_FND)
                                                            + "|" + getStateStr(STATE_PIVOTING));
            return false;
        }
    }

}

bool input_apply_fns_fnd(){

    /*CHECK VALID STATE*/
    switch (state){

        case STATE_FNS:{
            bool b = pauseDynControl(false);
            state = STATE_FNS_FND;
            return b;
        }

        case STATE_GRASPED:
        {
            bool b = pauseStaticControl(false, true) && pauseDynControl(false);
            state = STATE_FNS_FND;
            return b;   
        }

        case STATE_FNS_FND:
        {
            cout << HEADER_PRINT << YELLOW "Warn in input_apply_fns_fnd() - Already in " << getStateStr() << ", i will not refresh because fnd has an internal state" CRESET << endl;
            return true;   
        }
        
        case STATE_PIVOTING: {
            abortPivoting();
            bool b = pauseStaticControl(false, true) && pauseDynControl(false);
            state = STATE_FNS_FND;
            return b; 
        }

        default:{
           pErrorInvalidStartState("input_apply_fns_fnd()",            getStateStr(STATE_GRASPED)
                                                            + "|" + getStateStr(STATE_FNS) 
                                                            + "|" + getStateStr(STATE_FNS_FND)
                                                            + "|" + getStateStr(STATE_PIVOTING));
            return false;
        }
    }

}

bool input_pivoting(double ang, int& error){

    /*CHECK VALID STATE*/
    switch (state){

        case STATE_GRASPED:
        {
            state = STATE_PIVOTING;
            return doPivotingAction(ang, error);
        }

        case STATE_FNS:{
            pauseStaticControl(true, true);
            state = STATE_PIVOTING;
            return doPivotingAction(ang, error);
        }

        case STATE_FNS_FND:
        {
            pauseDynControl(true);
            pauseStaticControl(true, true);
            state = STATE_PIVOTING;
            return doPivotingAction(ang, error);  
        }

        default:{
           pErrorInvalidStartState("input_pivoting()",              getStateStr(STATE_GRASPED)
                                                            + "|" + getStateStr(STATE_FNS) 
                                                            + "|" + getStateStr(STATE_FNS_FND));
            return false;
        }
    }

}

/* END ============= STATE TRANSITIONS ================= */
