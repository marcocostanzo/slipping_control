/*
    Slipping Control Action Client

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

#ifndef SLIPPING_CONTROL_CLIENT_H
#define SLIPPING_CONTROL_CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include "slipping_control_msgs/HomeGripperAction.h"
#include "slipping_control_msgs/GraspAction.h"
#include "slipping_control_msgs/GraspAction.h"
#include "slipping_control_msgs/SlippingControlAction.h"
#include "slipping_control_msgs/GetState.h"
#include "slipping_control_msgs/ChLSParams.h"

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

class Slipping_Control_Client {

public: 
ros::NodeHandle nh_;

private:

protected:

ros::ServiceClient service_client_get_state_;
ros::ServiceClient service_client_ch_params0_;
ros::ServiceClient service_client_ch_params1_;

bool one_ls_;

public:

actionlib::SimpleActionClient<slipping_control_msgs::HomeGripperAction> ac_home_;
actionlib::SimpleActionClient<slipping_control_msgs::GraspAction> ac_grasp_;
actionlib::SimpleActionClient<slipping_control_msgs::SlippingControlAction> ac_sc_;

bool active;

Slipping_Control_Client( const ros::NodeHandle& nh, bool active = false, bool one_ls = false, bool wait_for_servers = true );

void waitForServers();

void home( bool wait_result = true, bool do_not_move_the_gripper = false);

bool graspIsDone();
void abortGrasp(bool wait_for_result = true);
void grasp( double force, double slope = 0.0, bool wait_result = true,
const actionlib::SimpleActionClient< slipping_control_msgs::GraspAction >::SimpleDoneCallback& done_cb = actionlib::SimpleActionClient< slipping_control_msgs::GraspAction >::SimpleDoneCallback() );

void gripper_pivoting( bool wait_result = true );

void slipping_avoidance( bool dyn_mode = false, bool wait_result = true );

/*
    # define STATE_UNDEFINED                    -1
    # define STATE_HOME                          0
    # define STATE_HOMING                        1
    # define STATE_COMPUTING_BIAS                2
    # define STATE_GRASPING                      3
    # define STATE_GRASPED                       4
    # define STATE_TO_GRIPPER_PIVOTING           5
    # define STATE_GRIPPER_PIVOTING              6
    # define STATE_TO_SLIPPING_AVOIDANCE         7
    # define STATE_SLIPPING_AVOIDANCE            8
    # define STATE_TO_DYN_SLIPPING_AVOIDANCE     9
    # define STATE_DYN_SLIPPING_AVOIDANCE       10
    # define STATE_OBJECT_PIVOTING              11
*/

int get_state();

bool is_grasped();

void change_params(double mu0, double mu1 = -1);

};

#endif