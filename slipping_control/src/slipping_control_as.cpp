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

#include <slipping_control_common/HomeGripperAction.h>

#include <actionlib/server/simple_action_server.h>

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

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

using namespace std;

/*STATES*/
#define STATE_UNDEFINED    -1
#define STATE_HOMING        1
#define STATE_HOME          2

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
    Service client to sent home command to the gripper
*/
ros::ServiceClient service_client_homing_gripper_;

/*
    Service client to Start/Stop low level force controller
*/
ros::ServiceClient service_client_force_controller_set_running_;

/*************************************
    Action Homing
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<slipping_control_common::HomeGripperAction> home_gripper_as_;

public:

/*
    Contstructor
*/
Slipping_Control_AS(
    const ros::NodeHandle& nh,
    const std::string& service_clinet_home_gripper,
    const std::string& service_clinet_force_controller_set_running,
    const std::string& action_home_gripper
):
    nh_(nh),
    home_gripper_as_(nh_, action_home_gripper, boost::bind(&Slipping_Control_AS::executeHomeGripperCB, this, _1), false)
{
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
    cout << HEADER_PRINT GREEN "Servers online!" CRESET << endl;

    cout << HEADER_PRINT YELLOW "Starting Actions..." CRESET << endl;
    home_gripper_as_.start();
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
    COMMON fcns
***************************************/

/*
    Abort all action except homing
*/
void abortAllActions()
{
    //Add abortAction for each action here
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
    string service_clinet_home_gripper_str;
    nh_private.param("sc_home_gripper" , service_clinet_home_gripper_str, string("homing_gripper") );
    string service_clinet_force_controller_set_running_str;
    nh_private.param("sc_force_control_set_running" , service_clinet_force_controller_set_running_str, string("force_controller/set_running") );
    string action_home_str;
    nh_private.param("action_home" , action_home_str, string("action_home") );

    /*AS*/

    Slipping_Control_AS server(
        nh_public,
        service_clinet_home_gripper_str,
        service_clinet_force_controller_set_running_str,
        action_home_str
    );
    
    server.start();

    ros::spin();

    return 0;
}