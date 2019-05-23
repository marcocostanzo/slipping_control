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

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

#include <slipping_control_common/HomeGripperAction.h>
#include <sun_tactile_common/ComputeBiasAction.h>
#include <slipping_control_common/GraspAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

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

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

using namespace std;

/*STATES*/
#define STATE_UNDEFINED    -1
#define STATE_HOME          0
#define STATE_HOMING        1
#define STATE_REMOVING_BIAS 2
#define STATE_GRASPING      3
#define STATE_GRASPED       4

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
    Action Homing
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<slipping_control_common::HomeGripperAction> home_gripper_as_;

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
actionlib::SimpleActionServer<slipping_control_common::GraspAction> grasp_as_;

/*
    Subscribers
*/
ros::Subscriber subGraspForce_, subF0_, subF1_;
std::string topic_grasp_force_str_, topic_force0_str_, topic_force1_str_;
/*
    Params
*/
double CONTACT_FORCE_THR_, BEFORE_CONTACT_FORCE_;

public:

/*
    Contstructor
*/
Slipping_Control_AS(
    const ros::NodeHandle& nh,
    double hz,
    double CONTACT_FORCE_THR,
    double BEFORE_CONTACT_FORCE,
    const std::string& topic_desired_grasp_force,
    const std::string& topic_grasp_force,
    const std::string& topic_force0,
    const std::string& topic_force1,
    const std::string& service_clinet_home_gripper,
    const std::string& service_clinet_force_controller_set_running,
    const std::string& action_client_compute_bias_0,
    const std::string& action_client_compute_bias_1,
    const std::string& action_home_gripper,
    const std::string& action_compute_bias,
    const std::string& action_grasp
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
    grasp_as_(nh_, action_compute_bias, boost::bind(&Slipping_Control_AS::executeGraspCB, this, _1), false),
    // true causes the client to spin its own thread
    ac_compute_bias_0(action_client_compute_bias_0, true),
    ac_compute_bias_1(action_client_compute_bias_1, true)
{
    pub_desired_force_ = nh_.advertise<std_msgs::Float64>(topic_desired_grasp_force, 1);
    service_client_homing_gripper_ = nh_.serviceClient<std_srvs::Empty>(service_clinet_home_gripper);
    service_client_force_controller_set_running_ = nh_.serviceClient<std_srvs::SetBool>(service_clinet_force_controller_set_running);
}

/*
    Start the server
*/
void start()
{

    cout << HEADER_PRINT YELLOW "Wait for servers..." CRESET << endl;
    service_client_homing_gripper_.waitForExistence();
    ac_compute_bias_0.waitForServer();
    ac_compute_bias_1.waitForServer();
    cout << HEADER_PRINT GREEN "Servers online!" CRESET << endl;

    cout << HEADER_PRINT YELLOW "Starting Actions..." CRESET << endl;
    home_gripper_as_.start();
    compute_bias_as_.start();
    grasp_as_.start();
    cout << HEADER_PRINT GREEN "Actions Started!" CRESET << endl;
}

/*************************************
    Action Homing
***************************************/
void executeHomeGripperCB( const slipping_control_common::HomeGripperGoalConstPtr &goal )
{

    bool b_error = false;

    //state homing
    state_ = STATE_HOMING;
    //Abort all actions if running
    abortAllActions();
    //stop the low level force controller
    b_error = !force_controller_set_running(false);

    //send homing command
    if(send_homing_command()){
        //Homing command sent

        //wait for compleate homing
        sleep(1);
        //waitForGripperZeroVel();

        slipping_control_common::HomeGripperResult result;
        result.success = true;
        if(b_error){
            //Homing ok but some error occurs
            result.msg = "ERROR";
            state_ = STATE_HOME;
        } else {
            //No error
            result.msg = "OK";
            state_ = STATE_UNDEFINED;
        }
        
        home_gripper_as_.setSucceeded(result);

    } else {
        //Fail to send homing command
        state_ = STATE_UNDEFINED;
        slipping_control_common::HomeGripperResult result;
        result.success = false;
        result.msg = "Fail to send Home command";
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
    while(!ac_compute_bias.waitForResult(ros::Duration(1.0/hz_))){
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

    if(state_ == STATE_HOME){

        state_ = STATE_REMOVING_BIAS;

        if(call_action_compute_bias(ac_compute_bias_0, goal, 0)){ //<-- Parallelize in future implementation...
            if(call_action_compute_bias(ac_compute_bias_1, goal, 1)){
                //Success
                state_ = STATE_HOME;
                sun_tactile_common::ComputeBiasResult result;
                result.success = true;
                result.msg = "OK";
                result.bias = ac_compute_bias_0.getResult()->bias;
                result.bias.insert( result.bias.end(), ac_compute_bias_1.getResult()->bias.begin(), ac_compute_bias_1.getResult()->bias.end() );
                compute_bias_as_.setSucceeded(result);
            } else {
                state_ = STATE_UNDEFINED;
            }
        } else {
            state_ = STATE_UNDEFINED;
        }

    } else {

        //INVALID START STATE
        //pErrorInvalidStartState("executeHomeGripperCB()", getStateStr(STATE_HOME));

        sun_tactile_common::ComputeBiasResult result;
        result.success = false;
        result.msg = "Invalid Initial State: " /*+ getStateStr(state_)*/ ;
        compute_bias_as_.setAborted(result);

    }

}

// Called once when the goal completes
void compute_bias_doneCb(const actionlib::SimpleClientGoalState& action_state,
            const sun_tactile_common::ComputeBiasResultConstPtr& result)
{
    cout << HEADER_PRINT GREEN "Remove Bias " << compute_bias_id_ << " DONE! finisched in state " << action_state.toString()  << CRESET << endl;    
}

// Called once when the goal becomes active
void compute_bias_activeCb()
{
    cout << HEADER_PRINT GREEN "Remove Bias " << compute_bias_id_ << " just went active " << CRESET << endl;
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
void executeGraspCB( const slipping_control_common::GraspGoalConstPtr &goal )
{
    /*CHECK VALID STATE*/
    switch (state_){

        case STATE_HOME:
        case STATE_GRASPED:
        {
            state_ = STATE_GRASPING;
            b_grasping_preemted_ = false;
            return doGraspingAction(goal);   
        }
        case STATE_GRASPING:{
            abortGrasping();
            state_ = STATE_GRASPING;
            b_grasping_preemted_ = false;
            return doGraspingAction(goal);
        }
        default:{
            /*pErrorInvalidStartState("executeGraspCB()",            getStateStr(STATE_HOME) 
                                                        + "|" + getStateStr(STATE_GRASPED) 
                                                        + "|" + getStateStr(STATE_GRASPING)
                                                        + "|" + getStateStr(STATE_FNS)
                                                        + "|" + getStateStr(STATE_FNS_FND)
                                                        + "|" + getStateStr(STATE_PIVOTING));
                                                        */
            cout << HEADER_PRINT BOLDRED "Invalid initial state in executeGraspCB()" CRESET << endl;

            slipping_control_common::GraspResult result;
            result.success = false;
            result.msg = "Invalid Initial State: " /*+ getStateStr(state_)*/ ;

            grasp_as_.setAborted(result);

        }
    }
}

void graspActionSetAborted(const string& msg)
{
    slipping_control_common::GraspResult result;
    result.success = false;
    result.msg = msg;
    grasp_as_.setAborted(result);
}

void graspActionSetPreempted(const string& msg = string("Preempted"))
{
    slipping_control_common::GraspResult result;
    result.success = false;
    result.msg = msg;
    grasp_as_.setPreempted(result);
}

void graspActionPublishForce( double force )
{
    if(state_ == STATE_GRASPING){
        std_msgs::Float64 force_ref_msg;
        force_ref_msg.data = force;
        pub_desired_force_.publish(force_ref_msg);

        slipping_control_common::GraspFeedback feedback_msg;
        feedback_msg.reference_force = force;
        grasp_as_.publishFeedback(feedback_msg);
    }
}

void doGraspingAction(const slipping_control_common::GraspGoalConstPtr &goal)
{

    subGraspForce_ = nh_.subscribe(topic_grasp_force_str_, 1, &Slipping_Control_AS::read_grasp_force_cb, this);
    subF0_ = nh_.subscribe(topic_force0_str_, 1, &Slipping_Control_AS::read_force0_cb, this);
    subF1_ = nh_.subscribe(topic_force1_str_, 1, &Slipping_Control_AS::read_force1_cb, this);
    

    //make sure that force control is active
    if( !force_controller_set_running(true) ){
        cout << HEADER_PRINT << BOLDRED "Error in doGraspingAction() - cannot start force control!" CRESET << endl;
        graspActionSetAborted("Fail to start force controller");
        subF0_.shutdown();
        subF1_.shutdown();
        subGraspForce_.shutdown();
        return;
    }

    //Wait for first sample of forces
    b_grasp_force_arrived_ = false;
    b_force0_arrived_ = false;
    b_force1_arrived_ = false;
    while( !b_grasp_force_arrived_ || !b_force0_arrived_ || !b_force1_arrived_ ){
        if (grasp_as_.isPreemptRequested() || b_grasping_preemted_ || !ros::ok()) {
            graspActionSetPreempted();
            subF0_.shutdown();
            subF1_.shutdown();
            subGraspForce_.shutdown();
            return;
        }
        ros::spinOnce();
    }

    ros::Rate loop_rate(hz_);

    //Check if in contact or not
    if( grasp_force_m_ < goal->desired_force && ( grasp_force_m_ <  CONTACT_FORCE_THR_ || f0_m_ < CONTACT_FORCE_THR_/2.0 || f1_m_ < CONTACT_FORCE_THR_/2.0) ){
        //Not In contact.... do Contact..
        cout << HEADER_PRINT "Contact..." << endl;

        double contact_thr_force_local = (goal->desired_force < CONTACT_FORCE_THR_) ? goal->desired_force : CONTACT_FORCE_THR_;

        while ( ros::ok() && (grasp_force_m_ <  contact_thr_force_local || f0_m_ < contact_thr_force_local/2.0 || f1_m_ < contact_thr_force_local/2.0) ){
            if (grasp_as_.isPreemptRequested() || b_grasping_preemted_ || !ros::ok()) {
                graspActionSetPreempted();
                subF0_.shutdown();
                subF1_.shutdown();
                subGraspForce_.shutdown();
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
    cout << HEADER_PRINT "Contact" GREEN "OK" CRESET << endl;

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
    cout << HEADER_PRINT << "Starting ramp... total time = " << ramp_total_time << endl;

    //Do Ramp
    double sec = 0.0;  
    double sec_init = ros::Time::now().toSec();
    while ( ros::ok() && sec < ramp_total_time ) {
        if (grasp_as_.isPreemptRequested() || b_grasping_preemted_ || !ros::ok()) {
            graspActionSetPreempted();
            subGraspForce_.shutdown();
            return;
        }
        sec = ros::Time::now().toSec() - sec_init;
        graspActionPublishForce( initial_force + sec * force_slope ); 

        loop_rate.sleep();
        ros::spinOnce();

    }

    //publish final force (to be sure)
    graspActionPublishForce( goal->desired_force );

    cout << HEADER_PRINT "Ramp " GREEN "OK" CRESET << endl;

    slipping_control_common::GraspResult result;
    result.success = true;
    result.msg = "OK";
    grasp_as_.setSucceeded(result);

    subGraspForce_.shutdown();

    state_ = STATE_GRASPED;

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
}

void abortGrasping()
{ 
    if( grasp_as_.isActive() ){
        b_grasping_preemted_ = true;
    }
}

double grasp_force_m_;
bool b_grasp_force_arrived_ = false;
void read_grasp_force_cb(const std_msgs::Float64::ConstPtr& forceMsg)
{
    grasp_force_m_ = fabs(forceMsg->data);
    b_grasp_force_arrived_ = true;
}

double f0_m_;
bool b_force0_arrived_ = false;
void read_force0_cb(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg)
{
    f0_m_ = fabs(forceMsg->wrench.force.z);
    b_force0_arrived_ = true;
}

double f1_m_;
bool b_force1_arrived_ = false;
void read_force1_cb(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg)
{
    f1_m_ = fabs(forceMsg->wrench.force.z);
    b_force1_arrived_ = true;
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
    cout << HEADER_PRINT "Homing..." << endl;
	if(!service_client_homing_gripper_.call(emptySrvMsg)){
		cout << HEADER_PRINT BOLDRED "Error during homing" CRESET << endl;
        return false;
	} else {
        cout << HEADER_PRINT "Homing Command Sent" CRESET << endl;
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

/*
    Start / Stop  low level force controller
*/
bool force_controller_set_running(bool b_running)
{
    return send_set_running_srv(service_client_force_controller_set_running_ , b_running , "Force Controller");
}

};

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

    /*AS*/

    Slipping_Control_AS server(
        nh_public,
        hz,
        CONTACT_FORCE_THR,
        BEFORE_CONTACT_FORCE,
        topic_desired_grasp_force_str,
        topic_grasp_force_str,
        topic_force0_str,
        topic_force1_str,
        service_clinet_home_gripper_str,
        service_clinet_force_controller_set_running_str,
        ac_compute_bias_0_str,
        ac_compute_bias_1_str,
        as_home_gripper_str,
        as_compute_bias_str,
        as_grasp_str
    );
    
    server.start();

    ros::spin();

    return 0;
}