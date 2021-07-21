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

#include "slipping_control_common/Slipping_Control_Client.h"

using namespace std;

Slipping_Control_Client::Slipping_Control_Client( const ros::NodeHandle& nh, bool gripper_active, bool one_ls, bool wait_for_servers)
:
one_ls_(one_ls),
active(gripper_active),
nh_(nh),
ac_home_(nh_, "slipping_control/home_gripper_action", true),
ac_grasp_(nh_, "slipping_control/grasp_action", true),
ac_sc_(nh_, "slipping_control/slipping_control_action", true)
{
    service_client_get_state_ = nh_.serviceClient<slipping_control_msgs::GetState>("slipping_control/get_state");
    if(!one_ls_)
    {
    service_client_ch_params0_ = nh_.serviceClient<slipping_control_msgs::ChLSParams>("slipping_control/finger0/ls/change_params");
    service_client_ch_params1_ = nh_.serviceClient<slipping_control_msgs::ChLSParams>("slipping_control/finger1/ls/change_params");
    }
    else
    {
    service_client_ch_params0_ = nh_.serviceClient<slipping_control_msgs::ChLSParams>("slipping_control/ls/change_params");
    }

    if(wait_for_servers)
        waitForServers();
}

void Slipping_Control_Client::waitForServers()
{
    if(active)
    {
        ac_home_.waitForServer();
        ac_grasp_.waitForServer();
        ac_sc_.waitForServer();
        service_client_get_state_.waitForExistence();
        service_client_ch_params0_.waitForExistence();
        if(!one_ls_)
            service_client_ch_params1_.waitForExistence();
    }
}

void Slipping_Control_Client::home( bool wait_result, bool do_not_move_the_gripper )
{
    if(!active) return;

    slipping_control_msgs::HomeGripperGoal goal;
    goal.do_not_move_the_gripper = do_not_move_the_gripper;
    ac_home_.sendGoal(goal);
    if(wait_result)
    {
        ac_home_.waitForResult();
        if( !ac_home_.getResult()->success )
        {
            cout << BOLDRED "ERROR SlipControl Client home() - " BOLDYELLOW << ac_home_.getResult()->msg << CRESET << endl;
        }
    }
}

bool Slipping_Control_Client::graspIsDone()
{
    return ac_grasp_.getState().isDone();
}

void Slipping_Control_Client::abortGrasp(bool wait_for_result)
{
    ac_grasp_.cancelGoal();
    if(wait_for_result)
    {
        ac_grasp_.waitForResult();
    }
}

void Slipping_Control_Client::grasp( double force, double slope, bool wait_result,
const actionlib::SimpleActionClient< slipping_control_msgs::GraspAction >::SimpleDoneCallback& done_cb)
{
    if(!active) return;

    slipping_control_msgs::GraspGoal goal;
    goal.desired_force = force;
    goal.force_slope = slope;
    ac_grasp_.sendGoal(goal, done_cb);
    if(wait_result)
    {
        ac_grasp_.waitForResult();
        if( !ac_grasp_.getResult()->success )
        {
            cout << BOLDRED "ERROR SlipControl Client grasp() - " BOLDYELLOW << ac_grasp_.getResult()->msg << CRESET << endl;
            throw "Error in grasp";
        }
    }

}

void Slipping_Control_Client::gripper_pivoting( bool wait_result )
{
    if(!active) return;

    slipping_control_msgs::SlippingControlGoal goal;
    goal.mode = slipping_control_msgs::SlippingControlGoal::MODE_GRIPPER_PIVOTING;
    ac_sc_.sendGoal(goal);
    if(wait_result)
    {
        ac_sc_.waitForResult();
        if( !ac_sc_.getResult()->success )
        {
            cout << BOLDRED "ERROR SlipControl Client gripper_pivoting() - " BOLDYELLOW << ac_sc_.getResult()->msg << CRESET << endl;
            throw "Error in gripper pivoting";
        }
    }

}

void Slipping_Control_Client::slipping_avoidance( bool dyn_mode, bool wait_result )
{
    if(!active) return;

    slipping_control_msgs::SlippingControlGoal goal;
    goal.mode = ( dyn_mode ? slipping_control_msgs::SlippingControlGoal::MODE_DYN_SLIPPING_AVOIDANCE : slipping_control_msgs::SlippingControlGoal::MODE_SLIPPING_AVOIDANCE);
    ac_sc_.sendGoal(goal);
    if(wait_result)
    {
        ac_sc_.waitForResult();
        if( !ac_sc_.getResult()->success )
        {
            cout << BOLDRED "ERROR SlipControl Client slipping_avoidance() - " BOLDYELLOW << ac_sc_.getResult()->msg << CRESET << endl;
            throw "Error in slipping avoidance";
        }
    }
}

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

int Slipping_Control_Client::get_state()
{
    if(active)
    {
        slipping_control_msgs::GetState srv;
        if(!service_client_get_state_.call(srv))
        {
            cout << BOLDRED "Error during get state" CRESET << endl;
            throw "Error in get state";
        } 
        else 
        {
            return srv.response.state;
        }
    }
    else
        return 4;

}

bool Slipping_Control_Client::is_grasped()
{
    int state = get_state();
    if( state == 4 || state == 5 || state == 6 || state == 7 || state == 8 || state == 9 || state == 10 )
        return true;
    return false;
}

void Slipping_Control_Client::change_params(double mu0, double mu1)
{

    if(!active)
        return;

    if(mu1 == -1)
        mu1 = mu0;

    slipping_control_msgs::ChLSParams srv;
    srv.request.delta = -1;
    srv.request.gamma = -1;
    srv.request.mu = mu0;
    srv.request.k = -1;

    if(!service_client_ch_params0_.call(srv))
    {
        cout << BOLDRED "Error during set mu0" CRESET << endl;
        throw "Error in set mu0";
    } 

    if(!one_ls_){
        srv.request.mu = mu1;
        if(!service_client_ch_params1_.call(srv))
        {
            cout << BOLDRED "Error during set mu1" CRESET << endl;
            throw "Error in set mu1";
        }
    }

}
