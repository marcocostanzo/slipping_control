/*
    Action server to control slip control nodes

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

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#include <slipping_control_msgs/ContactForcesStamped.h>
#include <sun_ros_msgs/Float64Stamped.h>
#include "slipping_control_msgs/LSCombinedStamped.h"

#include <slipping_control_msgs/HomeGripperAction.h>
#include <sun_tactile_common/ComputeBiasAction.h>
#include <slipping_control_msgs/GraspAction.h>
#include <slipping_control_msgs/SlippingControlAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <slipping_control_msgs/GetState.h>

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

//[N/s]
#define DEFAULT_GRASP_FORCE_SLOPE 2.0

//[m^-1]
#define DEFAULT_OBJECT_PIVOTING_GAIN 2.0
//[Nm] 
#define OBJ_PIV_TAU_EPS 0.005

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

#define HEADER_PRINT_STATE HEADER_PRINT "[" << getStateStr() << "] "

using namespace std;

/*STATES*/
#define STATE_UNDEFINED                    -1
#define STATE_HOME                          0
#define STATE_HOMING                        1
#define STATE_COMPUTING_BIAS                2
#define STATE_GRASPING                      3
#define STATE_GRASPED                       4
#define STATE_TO_GRIPPER_PIVOTING           5
#define STATE_GRIPPER_PIVOTING              6
#define STATE_TO_SLIPPING_AVOIDANCE         7
#define STATE_SLIPPING_AVOIDANCE            8
#define STATE_TO_DYN_SLIPPING_AVOIDANCE     9
#define STATE_DYN_SLIPPING_AVOIDANCE       10
#define STATE_OBJECT_PIVOTING              11

class Slipping_Control_AS {

private:

Slipping_Control_AS(); //No def constructor

protected:

/*
    STATE
*/
int state_ = STATE_UNDEFINED;

/*
    Node Handle
*/
ros::NodeHandle nh_;

/*
    Rate of the actions
*/
double hz_;

/*
    Service client to sent home command to the gripper
*/
ros::ServiceClient service_client_homing_gripper_;

/*
    Service client to Start/Stop low level force controller
*/
ros::ServiceClient service_client_force_controller_set_running_;

/*
    Publisher to the low level force controller
*/
ros::Publisher pub_desired_force_;

/*************************************
    Server Get State
***************************************/
ros::ServiceServer serviceGetState;

/*************************************
    Action Homing
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<slipping_control_msgs::HomeGripperAction> home_gripper_as_;

/*************************************
    Action Remove Bias
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<sun_tactile_common::ComputeBiasAction> compute_bias_as_;
/*
    Action Clients
*/
actionlib::SimpleActionClient<sun_tactile_common::ComputeBiasAction> ac_compute_bias_0, ac_compute_bias_1;

/*************************************
    Action Grasp
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<slipping_control_msgs::GraspAction> grasp_as_;

/*
    Subscribers
*/
ros::Subscriber subGraspForce_, subF0_, subF1_;
std::string topic_grasp_force_str_, topic_force0_str_, topic_force1_str_;
/*
    Params
*/
double CONTACT_FORCE_THR_, BEFORE_CONTACT_FORCE_;

/*************************************
    Action Sipping Control
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<slipping_control_msgs::SlippingControlAction> slipping_control_as_;
/*
    Subscribers
*/
ros::Subscriber subLSCombined_, subDynFn_;
/*
    Service client to Start/Stop observer and dyn_fn_controller
*/
ros::ServiceClient service_client_observer_set_running_, service_client_dyn_controller_set_running_;

/*
    Params
*/
double gain_go_to_zero_deg_ = 1.0; //Use ros-param here...

public:

/*
    Contstructor
*/
Slipping_Control_AS(
    const ros::NodeHandle& nh,
    double hz,
    double CONTACT_FORCE_THR,
    double BEFORE_CONTACT_FORCE,
    const std::string& topic_ls_combined,
    const std::string& topic_dyn_fn,
    const std::string& topic_desired_grasp_force,
    const std::string& topic_grasp_force,
    const std::string& topic_force0,
    const std::string& topic_force1,
    const std::string& service_clinet_home_gripper,
    const std::string& service_clinet_force_controller_set_running,
    const std::string& service_clinet_observer_set_running,
    const std::string& service_clinet_dyn_controller_set_running,
    const std::string& action_client_compute_bias_0,
    const std::string& action_client_compute_bias_1,
    const std::string& action_home_gripper,
    const std::string& action_compute_bias,
    const std::string& action_grasp,
    const std::string& action_slipping_control,
    const std::string& get_state_service_str
):
    nh_(nh),
    hz_(hz),
    CONTACT_FORCE_THR_(CONTACT_FORCE_THR),
    BEFORE_CONTACT_FORCE_(BEFORE_CONTACT_FORCE),
    topic_grasp_force_str_(topic_grasp_force),
    topic_force0_str_(topic_force0),
    topic_force1_str_(topic_force1),
    home_gripper_as_(nh_, action_home_gripper, boost::bind(&Slipping_Control_AS::executeHomeGripperCB, this, _1), false),
    compute_bias_as_(nh_, action_compute_bias, boost::bind(&Slipping_Control_AS::executeComputeBiasCB, this, _1), false),
    grasp_as_(nh_, action_grasp, boost::bind(&Slipping_Control_AS::executeGraspCB, this, _1), false),
    slipping_control_as_(nh_, action_slipping_control, boost::bind(&Slipping_Control_AS::executeSlippingControlCB, this, _1), false),
    // true causes the client to spin its own thread
    ac_compute_bias_0(action_client_compute_bias_0, true),
    ac_compute_bias_1(action_client_compute_bias_1, true)
{
    subLSCombined_ = nh_.subscribe(topic_ls_combined, 1, &Slipping_Control_AS::LSCombined_CB, this);
    subDynFn_ = nh_.subscribe(topic_dyn_fn, 1, &Slipping_Control_AS::dynFn_CB, this);
    pub_desired_force_ = nh_.advertise<sun_ros_msgs::Float64Stamped>(topic_desired_grasp_force, 1);
    service_client_homing_gripper_ = nh_.serviceClient<std_srvs::Empty>(service_clinet_home_gripper);
    service_client_force_controller_set_running_ = nh_.serviceClient<std_srvs::SetBool>(service_clinet_force_controller_set_running);
    service_client_observer_set_running_ = nh_.serviceClient<std_srvs::SetBool>(service_clinet_observer_set_running);
    service_client_dyn_controller_set_running_ = nh_.serviceClient<std_srvs::SetBool>(service_clinet_dyn_controller_set_running);

    serviceGetState = nh_.advertiseService(get_state_service_str, &Slipping_Control_AS::serviceGetStateCB, this);

}

bool serviceGetStateCB(slipping_control_msgs::GetState::Request  &req, 
   		 		slipping_control_msgs::GetState::Response &res){

    res.state = state_;

    return true;

}

/*
    Start the server
*/
void start()
{

    cout << HEADER_PRINT YELLOW "Wait for servers..." CRESET << endl;
    service_client_homing_gripper_.waitForExistence();
    service_client_force_controller_set_running_.waitForExistence();
    service_client_observer_set_running_.waitForExistence();
    service_client_dyn_controller_set_running_.waitForExistence();
    ac_compute_bias_0.waitForServer();
    ac_compute_bias_1.waitForServer();
    cout << HEADER_PRINT GREEN "Servers online!" CRESET << endl;

    cout << HEADER_PRINT YELLOW "Starting Actions..." CRESET << endl;
    home_gripper_as_.start();
    compute_bias_as_.start();
    grasp_as_.start();
    slipping_control_as_.start();
    cout << HEADER_PRINT GREEN "Actions Started!" CRESET << endl;
}

protected:

/*************************************
    Action Homing
***************************************/

void executeHomeGripperCB( const slipping_control_msgs::HomeGripperGoalConstPtr &goal )
{

    cout << HEADER_PRINT_STATE "[Action HomeGripper] Begin." << endl;

    bool b_error = false;

    //state homing
    state_ = STATE_HOMING;  //The initial state does not matter
    //Abort all actions if running
    abortAllActions();
    //stop the low level force controller
    b_error = !force_controller_set_running(false);
    //Stop dyn force controller
    b_error = b_error & !dyn_controller_set_running(false);
    //Stop observer
    b_error = b_error & !observer_set_running(false);

    //send homing command
    cout << HEADER_PRINT_STATE << "[Action HomeGripper] Sending homing command..." << endl;
    if(send_homing_command()){
        //Homing command sent
        cout << HEADER_PRINT_STATE << "[Action HomeGripper] Homing command sent!" << endl;

        //wait for compleate homing
        cout << HEADER_PRINT_STATE BOLDYELLOW "[Action HomeGripper] Homing check is open-loop..." CRESET << endl;
        sleep(1);
        //waitForGripperZeroVel();

        cout << HEADER_PRINT_STATE << "[Action HomeGripper] Homing " GREEN "Done" CRESET << endl;

        slipping_control_msgs::HomeGripperResult result;
        result.success = true;
        if(b_error){
            //Homing ok but some error occurs
            result.msg = "ERROR";
            state_ = STATE_HOME;
        } else {
            //No error
            result.msg = "OK";
            state_ = STATE_HOME;
        }

        cout << HEADER_PRINT_STATE << "[Action HomeGripper] " GREEN "Succeeded" CRESET << endl;
        
        home_gripper_as_.setSucceeded(result);

    } else {
        //Fail to send homing command
        cout << HEADER_PRINT_STATE << "[Action HomeGripper] " BOLDRED "Fail to send homing command" CRESET << endl;
        state_ = STATE_UNDEFINED;
        slipping_control_msgs::HomeGripperResult result;
        result.success = false;
        result.msg = "Fail to send Home command";
        cout << HEADER_PRINT_STATE << "[Action HomeGripper] " RED "Aborted" CRESET << endl;
        home_gripper_as_.setAborted(result);
    }

}

/*************************************
    Action Remove Bias
***************************************/

int compute_bias_id_;
bool call_action_compute_bias(  actionlib::SimpleActionClient<sun_tactile_common::ComputeBiasAction>& ac_compute_bias, 
                                const sun_tactile_common::ComputeBiasGoalConstPtr &goal, 
                                int bias_id)
{

    compute_bias_id_ = bias_id;

    ac_compute_bias.sendGoal(   
                                *goal,
                                boost::bind( &Slipping_Control_AS::compute_bias_doneCb, this, _1, _2 ), 
                                boost::bind( &Slipping_Control_AS::compute_bias_activeCb, this ), 
                                boost::bind( &Slipping_Control_AS::compute_bias_feedbackCb, this, _1 ) 
                            );
                            

    //wait for the action to return
    while(ros::ok() && !ac_compute_bias.waitForResult(ros::Duration(1.0/hz_))){
        ros::spinOnce();
    }

    actionlib::SimpleClientGoalState ac_state = ac_compute_bias.getState();
    if(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED && ac_compute_bias.getResult()->success  ){
        //Ok bias computed...
        return true;
    } else {
        //Error in computing bias
        sun_tactile_common::ComputeBiasResult result = *(ac_compute_bias.getResult());
        result.success = false;
        result.msg += " | Error on bias" + std::to_string(bias_id);
        compute_bias_as_.setAborted(result);
        return false;
    }

}

void executeComputeBiasCB( const sun_tactile_common::ComputeBiasGoalConstPtr &goal )
{

    cout << HEADER_PRINT_STATE "[Action ComputeBias] Called." << endl;

    if(state_ == STATE_HOME){

        state_ = STATE_COMPUTING_BIAS;
        cout << HEADER_PRINT_STATE "[Action ComputeBias] Begin." << endl;

        cout << HEADER_PRINT_STATE "[Action ComputeBias] Computing Bias of Finger 0..." << endl;
        if(call_action_compute_bias(ac_compute_bias_0, goal, 0)){ //<-- Parallelize in future implementation...
            cout << HEADER_PRINT_STATE "[Action ComputeBias] Bias of Finger 0 " GREEN "OK" CRESET << endl;
            cout << HEADER_PRINT_STATE "[Action ComputeBias] Computing Bias of Finger 1..." << endl;
            if(call_action_compute_bias(ac_compute_bias_1, goal, 1)){
                cout << HEADER_PRINT_STATE "[Action ComputeBias] Bias of Finger 1 " GREEN "OK" CRESET << endl;
                //Success
                state_ = STATE_HOME;
                sun_tactile_common::ComputeBiasResult result;
                result.success = true;
                result.msg = "OK";
                result.bias = ac_compute_bias_0.getResult()->bias;
                result.bias.insert( result.bias.end(), ac_compute_bias_1.getResult()->bias.begin(), ac_compute_bias_1.getResult()->bias.end() );
                cout << HEADER_PRINT_STATE << "[Action ComputeBias] " GREEN "Succeeded" CRESET << endl;
                compute_bias_as_.setSucceeded(result);
            } else {
                cout << HEADER_PRINT_STATE "[Action ComputeBias] Bias of Finger 1 " BOLDRED "ERROR" CRESET << endl;
                state_ = STATE_UNDEFINED;
                cout << HEADER_PRINT_STATE << "[Action ComputeBias] " RED "Aborted" CRESET << endl;
            }
        } else {
            cout << HEADER_PRINT_STATE "[Action ComputeBias] Bias of Finger 0 " BOLDRED "ERROR" CRESET << endl;
            state_ = STATE_UNDEFINED;
            cout << HEADER_PRINT_STATE << "[Action ComputeBias] " RED "Aborted" CRESET << endl;
        }

    } else {

        //INVALID START STATE
        pErrorInvalidStartState("ComputeBias", getStateStr(STATE_HOME));

        sun_tactile_common::ComputeBiasResult result;
        result.success = false;
        result.msg = "Invalid Initial State: "+ getStateStr(state_);
        cout << HEADER_PRINT_STATE << "[Action ComputeBias] " RED "Aborted" CRESET << endl;
        compute_bias_as_.setAborted(result);

    }

}

// Called once when the goal completes
void compute_bias_doneCb(const actionlib::SimpleClientGoalState& action_state,
            const sun_tactile_common::ComputeBiasResultConstPtr& result)
{
    cout << HEADER_PRINT_STATE GREEN "Remove Bias " << compute_bias_id_ << " DONE! finisched in state " << action_state.toString()  << CRESET << endl;    
}

// Called once when the goal becomes active
void compute_bias_activeCb()
{
    cout << HEADER_PRINT_STATE GREEN "Remove Bias " << compute_bias_id_ << " just went active " << CRESET << endl;
}

// Called every time feedback is received for the goal
void compute_bias_feedbackCb(const sun_tactile_common::ComputeBiasFeedbackConstPtr& feedback)
{
    compute_bias_as_.publishFeedback( feedback );
}

/*************************************
    Action Grasp
***************************************/

bool b_grasping_preemted_ = false;
void executeGraspCB( const slipping_control_msgs::GraspGoalConstPtr &goal )
{

    cout << HEADER_PRINT_STATE "[Action Grasp] Called." << endl;

    /*CHECK VALID STATE*/
    switch (state_)
    {

        case STATE_HOME:
        //case STATE_HOMING: <-- ???
        case STATE_GRASPED:
        case STATE_GRIPPER_PIVOTING:
        case STATE_SLIPPING_AVOIDANCE:
        case STATE_DYN_SLIPPING_AVOIDANCE:
        {
            //From a stable state
            dyn_controller_set_running(false);// To be sure
            state_ = STATE_GRASPING;
            b_grasping_preemted_ = false;
            return doGraspingAction(goal);   
        }

        case STATE_GRASPING:{
            //Grasping transition state.. this should not happen
            cout << HEADER_PRINT_STATE "[Action Grasp]" BOLDYELLOW "Called executeGraspCB() but state is STATE_GRASPING... this should not happen" CRESET << endl;
            abortGrasping();
            state_ = STATE_GRASPING;
            b_grasping_preemted_ = false;
            return doGraspingAction(goal);
        }

        case STATE_TO_GRIPPER_PIVOTING:
        case STATE_TO_SLIPPING_AVOIDANCE: 
        case STATE_TO_DYN_SLIPPING_AVOIDANCE:
        case STATE_OBJECT_PIVOTING:
        {
            //Slipping control transition state
            cout << HEADER_PRINT_STATE "[Action Grasp]" BOLDYELLOW "Called executeGraspCB() but state is a SlippingControl transition state..." CRESET << endl;
            abortSlippingControl();
            state_ = STATE_GRASPING;
            b_grasping_preemted_ = false;
            return doGraspingAction(goal);
        }

        default:
        {

            pErrorInvalidStartState("Grasp", 
                                                 getStateStr(STATE_HOME)
                                                 + "|" + getStateStr(STATE_GRASPED)
                                                 + "|" + getStateStr(STATE_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_GRASPING)
                                                 + "|" + getStateStr(STATE_TO_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_TO_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_OBJECT_PIVOTING)
                                    );

            graspActionSetAborted( "Invalid Initial State: " + getStateStr(state_) );

        }
    }
}

void graspActionSetAborted(const string& msg  = string("Aborted") )
{
    slipping_control_msgs::GraspResult result;
    result.success = false;
    result.msg = msg;
    cout << HEADER_PRINT_STATE << "[Action Grasp] " RED "Aborted" CRESET << endl;
    grasp_as_.setAborted(result);
}

void graspActionSetPreempted(const string& msg = string("Preempted"))
{
    slipping_control_msgs::GraspResult result;
    result.success = false;
    result.msg = msg;
    cout << HEADER_PRINT_STATE << "[Action Grasp] " BOLDYELLOW "Preempted" CRESET << endl;
    grasp_as_.setPreempted(result);
}

void graspActionPublishForce( double force )
{
    if(state_ == STATE_GRASPING){
        sun_ros_msgs::Float64Stamped force_ref_msg;
        force_ref_msg.data = force;
        force_ref_msg.header.stamp = ros::Time::now();
        pub_desired_force_.publish(force_ref_msg);

        slipping_control_msgs::GraspFeedback feedback_msg;
        feedback_msg.reference_force = force;
        grasp_as_.publishFeedback(feedback_msg);
    }
}

void doGraspingAction(const slipping_control_msgs::GraspGoalConstPtr &goal)
{

    cout << HEADER_PRINT_STATE "[Action Grasp] Begin." << endl;

    subGraspForce_ = nh_.subscribe(topic_grasp_force_str_, 1, &Slipping_Control_AS::read_grasp_force_cb, this);
    subF0_ = nh_.subscribe(topic_force0_str_, 1, &Slipping_Control_AS::read_force0_cb, this);
    subF1_ = nh_.subscribe(topic_force1_str_, 1, &Slipping_Control_AS::read_force1_cb, this);
    

    //make sure that force control is active
    if( !force_controller_set_running(true) ){
        cout << HEADER_PRINT_STATE "[Action Grasp] " BOLDRED "Fail to start Force Controller" CRESET "." << endl;
        state_ = STATE_UNDEFINED;
        graspActionSetAborted("Fail to start force controller");
        subF0_.shutdown();
        subF1_.shutdown();
        subGraspForce_.shutdown();
        return;
    }

    //Wait for first sample of forces
    cout << HEADER_PRINT_STATE "[Action Grasp] Waiting Samples..." << endl;
    b_grasp_force_arrived_ = false;
    b_force0_arrived_ = false;
    b_force1_arrived_ = false;
    while( !b_grasp_force_arrived_ || !b_force0_arrived_ || !b_force1_arrived_ ){
        if (grasp_as_.isPreemptRequested() || b_grasping_preemted_ || !ros::ok()) {
            graspActionSetPreempted();
            subF0_.shutdown();
            subF1_.shutdown();
            subGraspForce_.shutdown();
            state_ = STATE_UNDEFINED; //<-- I really do not know....
            return;
        }
        ros::spinOnce();
    }
    cout << HEADER_PRINT_STATE "[Action Grasp] Waiting Samples. OK." << endl;

    ros::Rate loop_rate(hz_);

    //Check if in contact or not
    cout << HEADER_PRINT_STATE "[Action Grasp] Check Contact..." << endl;
    if( grasp_force_m_ < goal->desired_force && ( grasp_force_m_ <  CONTACT_FORCE_THR_ || f0_m_ < CONTACT_FORCE_THR_/2.0 || f1_m_ < CONTACT_FORCE_THR_/2.0) ){
        //Not In contact.... do Contact..
        cout << HEADER_PRINT_STATE "[Action Grasp] Not In Contact..." << endl;

        double contact_thr_force_local = (goal->desired_force < CONTACT_FORCE_THR_) ? goal->desired_force : CONTACT_FORCE_THR_;

        while ( ros::ok() && (grasp_force_m_ <  contact_thr_force_local || f0_m_ < contact_thr_force_local/2.0 || f1_m_ < contact_thr_force_local/2.0) ){
            if (grasp_as_.isPreemptRequested() || b_grasping_preemted_ || !ros::ok()) {
                graspActionSetPreempted();
                subF0_.shutdown();
                subF1_.shutdown();
                subGraspForce_.shutdown();
                state_ = STATE_UNDEFINED; //<-- Contact not compleate... so?
                return;
            }

            graspActionPublishForce( BEFORE_CONTACT_FORCE_ );    
                
            loop_rate.sleep();
            ros::spinOnce();

        }

        graspActionPublishForce( contact_thr_force_local ); 

    }

    subF0_.shutdown();
    subF1_.shutdown();

    //In contact
    cout << HEADER_PRINT_STATE "[Action Grasp] Contact " GREEN "OK" CRESET << endl;

    //Build Ramp
    double initial_force = grasp_force_m_;
    double force_slope = goal->force_slope;
    if(force_slope <= 0.0){
        //Use default slope
        force_slope = DEFAULT_GRASP_FORCE_SLOPE;
    }
    double ramp_total_time = (goal->desired_force - initial_force)/force_slope;
    if(ramp_total_time < 0.0){
        ramp_total_time = -ramp_total_time;
        force_slope = -force_slope;
    }
    cout << HEADER_PRINT_STATE "[Action Grasp] Starting ramp... total time = " << ramp_total_time << endl;

    //Do Ramp 
    double sec_init = ros::Time::now().toSec();
    double sec = ros::Time::now().toSec() - sec_init; 
    while ( ros::ok() && sec < ramp_total_time ) {
        if (grasp_as_.isPreemptRequested() || b_grasping_preemted_ || !ros::ok()) {
            graspActionSetPreempted();
            subGraspForce_.shutdown();
            state_ = STATE_GRASPED; //<-- Grasped but with a different final force
            return;
        }
        
        graspActionPublishForce( initial_force + sec * force_slope ); 

        loop_rate.sleep();
        ros::spinOnce();

        sec = ros::Time::now().toSec() - sec_init;

    }

    //publish final force (to be sure)
    graspActionPublishForce( goal->desired_force );
    graspActionPublishForce( goal->desired_force );

    cout << HEADER_PRINT_STATE "[Action Grasp] Ramp " GREEN "OK" CRESET << endl;

    state_ = STATE_GRASPED;

    slipping_control_msgs::GraspResult result;
    result.success = true;
    result.msg = "OK";
    cout << HEADER_PRINT_STATE << "[Action Grasp] " GREEN "Succeeded" CRESET << endl;
    grasp_as_.setSucceeded(result);

    subGraspForce_.shutdown();

}

/*************************************
    Action Slopping Control
***************************************/

void slippingControlActionSetAborted(const string& msg  = string("Aborted") )
{
    slipping_control_msgs::SlippingControlResult result;
    result.success = false;
    result.msg = msg;
    cout << HEADER_PRINT_STATE << "[Action SlippingControl] " RED "Aborted" CRESET << endl;
    slipping_control_as_.setAborted(result);
}

void slippingControlActionSetPreempted(const string& msg = string("Preempted"))
{
    slipping_control_msgs::SlippingControlResult result;
    result.success = false;
    result.msg = msg;
    cout << HEADER_PRINT_STATE << "[Action SlippingControl] " BOLDYELLOW "Preempted" CRESET << endl;
    slipping_control_as_.setPreempted(result);
}

void slippingControlActionSetSucceeded(const string& msg = string("Succeeded"))
{
    slipping_control_msgs::SlippingControlResult result;
    result.success = true;
    result.state = state_;
    result.msg = msg;
    cout << HEADER_PRINT_STATE << "[Action SlippingControl] " GREEN "Succeeded" CRESET << endl;
    slipping_control_as_.setSucceeded(result);
}

void executeSlippingControlCB( const slipping_control_msgs::SlippingControlGoalConstPtr &goal )
{

    cout << HEADER_PRINT_STATE << "[Action SlippingControl] Called." << endl;

    switch (goal->mode)
    {

        case slipping_control_msgs::SlippingControlGoal::MODE_GRIPPER_PIVOTING:
        {

            cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_GRIPPER_PIVOTING]" << endl;

            //Check initial state
            switch (state_)
            {

                case STATE_GRASPED:
                case STATE_SLIPPING_AVOIDANCE: 
                case STATE_DYN_SLIPPING_AVOIDANCE: 
                {
                    //From a stable state
                    dyn_controller_set_running(false);// To be sure
                    state_ = STATE_TO_GRIPPER_PIVOTING;
                    if( goToZeroDeg() ){
                        state_ = STATE_GRIPPER_PIVOTING;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToZeroDeg");
                    }                    
                    break;
                }

                case STATE_TO_GRIPPER_PIVOTING:
                case STATE_TO_SLIPPING_AVOIDANCE:
                case STATE_TO_DYN_SLIPPING_AVOIDANCE:
                case STATE_OBJECT_PIVOTING:
                {
                    //From a slipping control transition state | this should not happen
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_GRIPPER_PIVOTING] " BOLDYELLOW "State is a Slipping Control state transition | this should not happen" CRESET << endl;
                    state_ = STATE_TO_GRIPPER_PIVOTING;
                    if( goToZeroDeg() ){
                        state_ = STATE_GRIPPER_PIVOTING;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToZeroDeg");
                    }
                    break;
                }

                case STATE_GRIPPER_PIVOTING:
                {
                    //From the same state
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_GRIPPER_PIVOTING] " YELLOW " Refreshing..." CRESET << endl;
                    state_ = STATE_TO_GRIPPER_PIVOTING;
                    if( goToZeroDeg() ){
                        state_ = STATE_GRIPPER_PIVOTING;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToZeroDeg");
                    }
                    break;
                }
                
                default:
                {
                    //Invalid Initial State
                    pErrorInvalidStartState("SlippingControl/GRIPPER_PIVOTING", 
                                                 getStateStr(STATE_GRASPED)
                                                 + "|" + getStateStr(STATE_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_TO_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_OBJECT_PIVOTING)
                                    );

                    slippingControlActionSetAborted("Invalid Initial State = " + getStateStr(state_) );
                }

            }
            
            break;
        }

        case slipping_control_msgs::SlippingControlGoal::MODE_SLIPPING_AVOIDANCE:
        {

            cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_SLIPPING_AVOIDANCE]" << endl;
            
            //Check initial state
            switch (state_)
            {

                case STATE_GRASPED:
                case STATE_GRIPPER_PIVOTING: 
                case STATE_DYN_SLIPPING_AVOIDANCE: 
                {
                    //From a stable state
                    dyn_controller_set_running(false);// To be sure
                    state_ = STATE_TO_SLIPPING_AVOIDANCE;
                    if(goToSlippingAvoidance()){
                        state_ = STATE_SLIPPING_AVOIDANCE;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToSlippingAvoidance");
                    }
                    break;
                }

                case STATE_TO_GRIPPER_PIVOTING:
                case STATE_TO_SLIPPING_AVOIDANCE:
                case STATE_TO_DYN_SLIPPING_AVOIDANCE:
                case STATE_OBJECT_PIVOTING:
                {
                    //From a slipping control transi tionstate | this should not happen
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_SLIPPING_AVOIDANCE] " BOLDYELLOW "State is a Slipping Control state transition | this should not happen" CRESET << endl;
                    state_ = STATE_TO_SLIPPING_AVOIDANCE;
                    if(goToSlippingAvoidance()){
                        state_ = STATE_SLIPPING_AVOIDANCE;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToSlippingAvoidance");
                    }
                    break;
                }

                case STATE_SLIPPING_AVOIDANCE: 
                {
                    //From the same state
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_SLIPPING_AVOIDANCE] " YELLOW " Refreshing..." CRESET << endl;
                    state_ = STATE_TO_SLIPPING_AVOIDANCE;
                    if(goToSlippingAvoidance()){
                        state_ = STATE_SLIPPING_AVOIDANCE;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToSlippingAvoidance");
                    }
                    break;
                }

                default:
                {
                    //Invalid Initial State
                    pErrorInvalidStartState("SlippingControl/SLIPPING_AVOIDANCE", 
                                                 getStateStr(STATE_GRASPED)
                                                 + "|" + getStateStr(STATE_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_TO_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_OBJECT_PIVOTING)
                                    );

                    slippingControlActionSetAborted("Invalid Initial State = " + getStateStr(state_) );
                }

            }

            break;
        }


        case slipping_control_msgs::SlippingControlGoal::MODE_DYN_SLIPPING_AVOIDANCE:
        {

            cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_DYN_SLIPPING_AVOIDANCE]" << endl;
            
            //Check initial state
            switch (state_)
            {

                case STATE_GRASPED:
                case STATE_GRIPPER_PIVOTING: 
                case STATE_SLIPPING_AVOIDANCE: 
                {
                    //From a stable state
                    state_ = STATE_TO_DYN_SLIPPING_AVOIDANCE;
                    //In this case I will reset the dyn_force_controller
                    if(goToDynSlippingAvoidance(true)){
                        state_ = STATE_DYN_SLIPPING_AVOIDANCE;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToDynSlippingAvoidance");
                    }
                    break;
                }

                case STATE_TO_GRIPPER_PIVOTING:
                case STATE_TO_SLIPPING_AVOIDANCE:
                case STATE_TO_DYN_SLIPPING_AVOIDANCE:
                case STATE_OBJECT_PIVOTING:
                {
                    //From a slipping control transi tionstate | this should not happen
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_DYN_SLIPPING_AVOIDANCE] " BOLDYELLOW "State is a Slipping Control state transition | this should not happen" CRESET << endl;
                    state_ = STATE_TO_DYN_SLIPPING_AVOIDANCE;
                    if(goToDynSlippingAvoidance()){
                        state_ = STATE_DYN_SLIPPING_AVOIDANCE;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error in goToDynSlippingAvoidance");
                    }
                    break;
                }

                case STATE_DYN_SLIPPING_AVOIDANCE: 
                {
                    //From the same state
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][STATE_DYN_SLIPPING_AVOIDANCE] " YELLOW " Refreshing..." << endl
                    << "I will not reset the dyn force controller if active..." CRESET << endl;
                   
                    state_ = STATE_TO_DYN_SLIPPING_AVOIDANCE;
                    if(goToDynSlippingAvoidance()){
                        state_ = STATE_DYN_SLIPPING_AVOIDANCE;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted("Error goToDynSlippingAvoidance");
                    }
                    break;
                }

                default:
                {
                    //Invalid Initial State
                    pErrorInvalidStartState("SlippingControl/DYN_SLIPPING_AVOIDANCE", 
                                                 getStateStr(STATE_GRASPED)
                                                 + "|" + getStateStr(STATE_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_TO_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_OBJECT_PIVOTING)
                                    );

                    slippingControlActionSetAborted("Invalid Initial State = " + getStateStr(state_) );
                }

            }
            
            break;
        }

        case slipping_control_msgs::SlippingControlGoal::MODE_OBJECT_PIVOTING:
        {

            cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_OBJECT_PIVOTING]" << endl;

            int prev_state = state_;
            string resp_str;
            
            //Check initial state
            switch (state_)
            {

                case STATE_GRASPED:
                case STATE_GRIPPER_PIVOTING: 
                case STATE_SLIPPING_AVOIDANCE:
                case STATE_DYN_SLIPPING_AVOIDANCE: 
                {
                    //From a stable state
                    state_ = STATE_OBJECT_PIVOTING;
                    //In this case I will reset the dyn_force_controller
                    if( objectPivoting( goal, resp_str) ){
                        state_ = prev_state;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted( resp_str );
                    }
                    break;
                }

                case STATE_TO_GRIPPER_PIVOTING:
                case STATE_TO_SLIPPING_AVOIDANCE:
                case STATE_TO_DYN_SLIPPING_AVOIDANCE:
                case STATE_OBJECT_PIVOTING:
                {
                    //From a slipping control transi tionstate | this should not happen
                    cout << HEADER_PRINT_STATE "[Action SlippingControl][MODE_OBJECT_PIVOTING] " BOLDYELLOW "State is a Slipping Control state transition | this should not happen" CRESET << endl;
                    state_ = STATE_OBJECT_PIVOTING;
                    if( objectPivoting( goal, resp_str) ){
                        state_ = prev_state;
                        slippingControlActionSetSucceeded();
                    } else {
                        slippingControlActionSetAborted( resp_str );
                    }
                    break;
                }

                default:
                {
                    //Invalid Initial State
                    pErrorInvalidStartState("SlippingControl/OBJECT_PIVOTING", 
                                                 getStateStr(STATE_GRASPED)
                                                 + "|" + getStateStr(STATE_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_GRIPPER_PIVOTING)
                                                 + "|" + getStateStr(STATE_TO_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_TO_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_DYN_SLIPPING_AVOIDANCE)
                                                 + "|" + getStateStr(STATE_OBJECT_PIVOTING)
                                    );

                    slippingControlActionSetAborted("Invalid Initial State = " + getStateStr(state_) );
                }

            }

            break;
        }

        default:
        {
            //Invalid INPUT IN GOAL
            cout << HEADER_PRINT_STATE RED "Invalid mode in Slipping Control" CRESET << endl;
            slippingControlActionSetAborted("Invalid Request");
        }
    }
    
}


/*************************************
    COMMON fcns
***************************************/

/*
    Abort all action except homing
*/
void abortAllActions()
{
    //Add abortAction for each action here
    abortGrasping();
    abortSlippingControl();
}

void abortGrasping()
{ 
    if( grasp_as_.isActive() ){
        cout << HEADER_PRINT_STATE BOLDYELLOW "call abortGrasping() with grasp active..." CRESET << endl;
        b_grasping_preemted_ = true;
    }
}

void abortSlippingControl()
{
    cout << HEADER_PRINT_STATE BOLDYELLOW "abortSlippingControl() is void!" CRESET << endl;
}

double grasp_force_m_;
bool b_grasp_force_arrived_ = false;
void read_grasp_force_cb(const sun_ros_msgs::Float64Stamped::ConstPtr& forceMsg)
{
    grasp_force_m_ = fabs(forceMsg->data);
    b_grasp_force_arrived_ = true;
}

double f0_m_, tau0_m_;
bool b_force0_arrived_ = false;
void read_force0_cb(const slipping_control_msgs::ContactForcesStamped::ConstPtr& forceMsg)
{
    f0_m_ = fabs(forceMsg->forces.fn);
    tau0_m_ = forceMsg->forces.taun;
    b_force0_arrived_ = true;
}

double f1_m_, tau1_m_;
bool b_force1_arrived_ = false;
void read_force1_cb(const slipping_control_msgs::ContactForcesStamped::ConstPtr& forceMsg)
{
    f1_m_ = fabs(forceMsg->forces.fn);
    tau1_m_ = forceMsg->forces.taun;
    b_force1_arrived_ = true;
}

double fn_ls_;
double fn_ls_free_pivot_;
void LSCombined_CB(const slipping_control_msgs::LSCombinedStamped::ConstPtr& msg)
{

    fn_ls_ = msg->fn_ls; //To use it in Dyn Fn CB
    fn_ls_free_pivot_ = msg->fn_ls_free_pivot; //To Use in goToZeroDeg()

    switch (state_)
    {

        case STATE_GRIPPER_PIVOTING:
        {
            publish_force_ref( msg->fn_ls_free_pivot );
            break;
        }

        case STATE_SLIPPING_AVOIDANCE:
        {
            publish_force_ref( msg->fn_ls );
            break;
        }

        case STATE_DYN_SLIPPING_AVOIDANCE:
        {
            publish_force_ref( msg->fn_ls + fn_dyn_ );
            break;
        }

        default:
        {
            //Do nothing
        }

    }
    
}

double fn_dyn_;
void dynFn_CB(const sun_ros_msgs::Float64Stamped::ConstPtr& msg)
{

    //If it is not fn_ls_, return! it is better to wait for the fn_ls_ message to arrive
    if( fn_ls_ == 0.0 )
        return;

    switch (state_)
    {

        case STATE_DYN_SLIPPING_AVOIDANCE:
        {
            fn_dyn_ = msg->data; //To use it in LsCombined CB
            publish_force_ref( fn_ls_ + msg->data );
            break;
        }

        default:
        {
            //Do nothing
            fn_dyn_ = 0.0; //<-- to be sure
        }

    }
    

}

bool goToZeroDeg()
{
    //cout << HEADER_PRINT_STATE BOLDYELLOW "goToZeroDeg() is void" CRESET << endl;
    //remember to change state if something goes wrong
    //I should check the preemption...

    //Wait a sample of grasp force
    subGraspForce_ = nh_.subscribe(topic_grasp_force_str_, 1, &Slipping_Control_AS::read_grasp_force_cb, this);
    b_grasp_force_arrived_ = false;
    while( !b_grasp_force_arrived_ )
    {
        if (!ros::ok()) {
            cout << HEADER_PRINT_STATE BOLDRED " Ros not ok() in goToZeroDeg()" CRESET << endl;
            exit(-1);
            return false;
        }
        ros::spinOnce();
    }

    double fr = grasp_force_m_;

    double gain_local = gain_go_to_zero_deg_/hz_;

    ros::Rate loop_rate(hz_);

    cout << HEADER_PRINT_STATE " goToZeroDeg() - Decreasing grasp force... " << endl;
    while( fr > fn_ls_free_pivot_ )
    {
        if (!ros::ok()) {
            cout << HEADER_PRINT_STATE BOLDRED " Ros not ok() in goToZeroDeg()" CRESET << endl;
            exit(-1);
            return false;
        }
        ros::spinOnce();
        fr = fr - gain_local*(fr - fn_ls_free_pivot_);
        publish_force_ref(fr);
        loop_rate.sleep();
    }

    
    subGraspForce_.shutdown();
    cout << HEADER_PRINT_STATE " goToZeroDeg() - " GREEN "Done" CRESET << endl;
    return true;
}

inline double getTau()
{
    return fabs(tau0_m_ - tau1_m_);
}

bool objectPivoting( const slipping_control_msgs::SlippingControlGoalConstPtr &goal, string& resp_str )
{
    //cout << HEADER_PRINT_STATE BOLDYELLOW "objectPivoting() is void" CRESET << endl;
    //remember to change state if something goes wrong
    //I should check the preemption...

    if(goal->desired_angle == 0.0)
    {
        cout << HEADER_PRINT_STATE "objectPivoting() - desired_angle=0 using goToZeroDeg()" CRESET << endl;
        if(!goToZeroDeg()) 
        {
            cout << HEADER_PRINT_STATE BOLDRED "objectPivoting() - Error in goToZeroDeg()" CRESET << endl;
            resp_str = "error in goToZeroDeg";
            return false;
        } else {
            cout << HEADER_PRINT_STATE "objectPivoting() - goToZeroDeg() " GREEN "Success" CRESET << endl;
            return true;
        }
    }

    //Wait a sample of measures
    subGraspForce_ = nh_.subscribe(topic_grasp_force_str_, 1, &Slipping_Control_AS::read_grasp_force_cb, this);
    subF0_ = nh_.subscribe(topic_force0_str_, 1, &Slipping_Control_AS::read_force0_cb, this);
    subF1_ = nh_.subscribe(topic_force1_str_, 1, &Slipping_Control_AS::read_force1_cb, this);
    b_grasp_force_arrived_ = false;
    b_force0_arrived_ = false;
    b_force1_arrived_ = false;
    while( !b_grasp_force_arrived_ || !b_force0_arrived_ || !b_force1_arrived_ )
    {
        if (!ros::ok()) {
            cout << HEADER_PRINT_STATE BOLDRED " Ros not ok() in objectPivoting()" CRESET << endl;
            exit(-1);
            return false;
        }
        ros::spinOnce();
    }

    double fr = grasp_force_m_;
    double desired_tau  = (sin(goal->desired_angle)/sin(goal->initial_angle)) * getTau(); 

    //Check if task is feasible
    if(desired_tau > getTau())
    { //not feasible
        cout << HEADER_PRINT_STATE " objectPivoting() - " BOLDRED "Can't go up" CRESET << endl;
        resp_str = "impossible_required_rotation";
        subGraspForce_.shutdown();
        return false;
    }

    //sign (-) needed
    double gain_local = -1.0/hz_;
    if(goal->object_pivoting_gain == 0){
        gain_local *= DEFAULT_OBJECT_PIVOTING_GAIN;
    } else {
        gain_local *= goal->object_pivoting_gain;
    }

    ros::Rate loop_rate(hz_);

    cout << HEADER_PRINT_STATE " objectPivoting() - Initial_Grasp_Force= " << fr << " | Initial_tau= " << getTau() << " Desired_tau= " << desired_tau << endl;
    while(
        //Emergency stop
        !(fr <= fn_ls_free_pivot_)
        &&
        //stop condition
        !( ( desired_tau - getTau() ) > OBJ_PIV_TAU_EPS ) 
        )
    {
        if (!ros::ok()) {
            cout << HEADER_PRINT_STATE BOLDRED " Ros not ok() in objectPivoting()" CRESET << endl;
            exit(-1);
            return false;
        }
        ros::spinOnce();
        fr = fr - gain_local*(desired_tau - getTau());
        publish_force_ref(fr);
        loop_rate.sleep();
    }

    //Check result
    //Case emergency stop
    bool b_return = true;
    if(fr <= fn_ls_free_pivot_)
    {
        cout << HEADER_PRINT_STATE " objectPivoting() - " BOLDRED "Minimum Force reached" CRESET << endl;
        resp_str = "minimum_force";
        b_return = false;
    } 
    //Case overshoot
    else if( getTau() < (desired_tau - 3.0*OBJ_PIV_TAU_EPS) )
    {
        cout << HEADER_PRINT_STATE " objectPivoting() - " BOLDYELLOW "Overshoot = " << desired_tau - getTau() << CRESET << endl;
        resp_str = "overshoot";
        b_return = false;
    } else {
        cout << HEADER_PRINT_STATE " objectPivoting() - " GREEN "OK" CRESET << endl;
    }

    subGraspForce_.shutdown();
    
    return b_return;
}

bool goToSlippingAvoidance()
{
    cout << HEADER_PRINT_STATE BOLDYELLOW "goToSlippingAvoidance() is void" CRESET << endl;
    //remember to change state if something goes wrong
    return true;
}

bool goToDynSlippingAvoidance( bool b_reset_dyn_force_controller = false, bool b_reset_observer = false )
{

    //IF flags are true, reset observer and controller by calling first set_running(false)
    if(b_reset_dyn_force_controller)
    {
        if(!dyn_controller_set_running(false))
        {
            state_ = STATE_UNDEFINED;            
            cout << HEADER_PRINT_STATE BOLDRED "FAIL TO RESET DYN CONTROLLER..." CRESET << endl;
            return false;
        }
    }
    if(b_reset_observer)
    {
        if(!observer_set_running(false))
        {
            state_ = STATE_UNDEFINED;            
            cout << HEADER_PRINT_STATE BOLDRED "FAIL TO RESET OBSERVER..." CRESET << endl;
            return false;
        }
    }

    //Make sure that observer is active
    if(!observer_set_running(true))
    {
        state_ = STATE_UNDEFINED;        
        cout << HEADER_PRINT_STATE BOLDRED "FAIL TO START OBSERVER..." CRESET << endl;
        return false;
    }
    //Make sure that dyn controller is active
    if(!dyn_controller_set_running(true))
    {
        state_ = STATE_UNDEFINED;        
        cout << HEADER_PRINT_STATE BOLDRED "FAIL TO START DYN CONTROLLER..." CRESET << endl;
        return false;
    }
    
    return true;

}

void pErrorInvalidStartState( const string& action_name, const string& valid_state_list)
{
    cout << HEADER_PRINT_STATE "[Action " << action_name << "] " RED "Invalid initial state" CRESET "." << endl
    << "Valid states are: " << valid_state_list << endl;
}

string getStateStr()
{
    return getStateStr(state_);
}

string getStateStr(int s)
{
    switch (s)
    {
        case STATE_UNDEFINED:{
            return "STATE_UNDEFINED";
        }
        case STATE_HOME:{
            return "STATE_HOME";
        }
        case STATE_HOMING:{
            return "STATE_HOMING";
        }
        case STATE_COMPUTING_BIAS:{
            return "STATE_COMPUTING_BIAS";
        }
        case STATE_GRASPING:{
            return "STATE_GRASPING";
        }
        case STATE_GRASPED:{
            return "STATE_GRASPED";
        }
        case STATE_TO_GRIPPER_PIVOTING:{
            return "STATE_TO_GRIPPER_PIVOTING";
        }
        case STATE_GRIPPER_PIVOTING:{
            return "STATE_GRIPPER_PIVOTING";
        }
        case STATE_TO_SLIPPING_AVOIDANCE:{
            return "STATE_TO_SLIPPING_AVOIDANCE";
        }
        case STATE_SLIPPING_AVOIDANCE:{
            return "STATE_SLIPPING_AVOIDANCE";
        }
        case STATE_TO_DYN_SLIPPING_AVOIDANCE:{
            return "STATE_TO_DYN_SLIPPING_AVOIDANCE";
        }
        case STATE_DYN_SLIPPING_AVOIDANCE:{
            return "STATE_DYN_SLIPPING_AVOIDANCE";
        }
        case STATE_OBJECT_PIVOTING:{
            return "STATE_OBJECT_PIVOTING";
        }
        default:{
            return (BOLDRED "INVALID_STATE" CRESET);
        }
    }
}

/************************************
    COMMANDs
************************************/

/*
    Send homing command to the gripper
*/
bool send_homing_command()
{
    std_srvs::Empty emptySrvMsg;
    cout << HEADER_PRINT_STATE "Homing..." << endl;
	if(!service_client_homing_gripper_.call(emptySrvMsg)){
		cout << HEADER_PRINT_STATE BOLDRED "Error during homing" CRESET << endl;
        return false;
	} else {
        cout << HEADER_PRINT_STATE "Homing Command Sent" CRESET << endl;
        return true;
    }
}

/*
    Used to start & stop nodes that have setBool interface
*/
bool send_set_running_srv(ros::ServiceClient& client, bool b_running, const string& msg){

    std_srvs::SetBool setBoolmsg;
    setBoolmsg.request.data = b_running;

    string what = b_running ? "START" : "STOP";
    string what_ing = b_running ? "STARTING" : "STOPPING";

    cout << HEADER_PRINT_STATE << what_ing << " " << msg << endl;

    bool b_out = client.call(setBoolmsg);
    b_out = b_out && setBoolmsg.response.success;

    if(b_out){
        cout << HEADER_PRINT_STATE << what << " " << msg << GREEN " OK" CRESET << endl;
    } else {
        cout << HEADER_PRINT_STATE RED "Error during " << BOLDRED << what << CRESET RED << " " << msg << CRESET << endl;
    }

    return b_out;

}

/*
    Start / Stop  low level force controller
*/
bool force_controller_set_running(bool b_running)
{
    return send_set_running_srv(service_client_force_controller_set_running_ , b_running , "Force Controller");
}

/*
    Start / Stop  observer
*/
bool observer_set_running(bool b_running)
{
    return send_set_running_srv(service_client_observer_set_running_ , b_running , "Observer");
}

/*
    Start / Stop  dyn controller
*/
bool dyn_controller_set_running(bool b_running)
{
    return send_set_running_srv(service_client_dyn_controller_set_running_ , b_running , "Dyn Controller");
}

void publish_force_ref( double force )
{
    sun_ros_msgs::Float64Stamped force_ref_msg;
    force_ref_msg.data = force;
    force_ref_msg.header.stamp = ros::Time::now();
    pub_desired_force_.publish(force_ref_msg);
}

};//END CLASS


/************************************
    MAIN
************************************/

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "slipping_control_as");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    /*PARAMS*/
    double hz;
    nh_private.param("rate" , hz, 1000.0 );

    double CONTACT_FORCE_THR;
    nh_private.param("contact_force_thr" , CONTACT_FORCE_THR, 0.8 );
    double BEFORE_CONTACT_FORCE;
    nh_private.param("before_contact_force" , BEFORE_CONTACT_FORCE, 1.7 );

    string topic_ls_combined_str;
    nh_private.param("topic_ls_combined" , topic_ls_combined_str, string("ls_combined") );
    string topic_dyn_fn_str;
    nh_private.param("topic_dyn_force" , topic_dyn_fn_str, string("dyn_force") );
    string topic_desired_grasp_force_str;
    nh_private.param("topic_desired_grasp_force" , topic_desired_grasp_force_str, string("desired_grasp_force") );
    string topic_grasp_force_str;
    nh_private.param("topic_grasp_force" , topic_grasp_force_str, string("grasp_force") );
    string topic_force0_str;
    nh_private.param("topic_force0" , topic_force0_str, string("finger0/force") );
    string topic_force1_str;
    nh_private.param("topic_force1" , topic_force1_str, string("finger1/force") );

    string service_clinet_home_gripper_str;
    nh_private.param("sc_home_gripper" , service_clinet_home_gripper_str, string("homing_gripper") );
    string service_clinet_force_controller_set_running_str;
    nh_private.param("sc_force_control_set_running" , service_clinet_force_controller_set_running_str, string("force_controller/set_running") );
    string service_clinet_observer_set_running_str;
    nh_private.param("sc_observer_set_running" , service_clinet_observer_set_running_str, string("observer/set_running") );
    string service_clinet_dyn_controller_set_running_str;
    nh_private.param("sc_dyn_controller_set_running" , service_clinet_dyn_controller_set_running_str, string("dyn_controller/set_running") );

    string ac_compute_bias_0_str;
    nh_private.param("ac_compute_bias_0" , ac_compute_bias_0_str, string("finger0/compute_bias") );
    string ac_compute_bias_1_str;
    nh_private.param("ac_compute_bias_1" , ac_compute_bias_1_str, string("finger1/compute_bias") );

    string as_home_gripper_str;
    nh_private.param("as_home_gripper" , as_home_gripper_str, string("action_home_gripper") );
    string as_compute_bias_str;
    nh_private.param("as_compute_bias" , as_compute_bias_str, string("action_compute_bias") );
    string as_grasp_str;
    nh_private.param("as_grasp" , as_grasp_str, string("action_grasp") );
    string as_slipping_control_str;
    nh_private.param("as_slipping_control" , as_slipping_control_str, string("action_slipping_control") );

    string get_state_service_str;
    nh_private.param("get_state_service" , get_state_service_str, string("slipping_control/get_state") );

    /*AS*/

    Slipping_Control_AS server(
        nh_public,
        hz,
        CONTACT_FORCE_THR,
        BEFORE_CONTACT_FORCE,
        topic_ls_combined_str,
        topic_dyn_fn_str,
        topic_desired_grasp_force_str,
        topic_grasp_force_str,
        topic_force0_str,
        topic_force1_str,
        service_clinet_home_gripper_str,
        service_clinet_force_controller_set_running_str,
        service_clinet_observer_set_running_str,
        service_clinet_dyn_controller_set_running_str,
        ac_compute_bias_0_str,
        ac_compute_bias_1_str,
        as_home_gripper_str,
        as_compute_bias_str,
        as_grasp_str,
        as_slipping_control_str,
        get_state_service_str
    );
    
    server.start();

    ros::spin();

    return 0;
}