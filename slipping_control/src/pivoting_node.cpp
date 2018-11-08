/*
    ROS node to generate grasp force for pivoting

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

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

#include <actionlib/server/simple_action_server.h>
#include <slipping_control_common/PivotingAction.h>

#include "Helper.h"

#include "slipping_control_common/pivoting_error_codes.h"


#define HEADER_PRINT BOLDYELLOW "[Pivoting Generator]: " CRESET


using namespace std;

string topic_force_command("");
string topic_wrench("");
string topic_grasp_force("");

double  mu_ = 0.8;
double MAX_TIME;
double TAU_THR;
double gain;
double rate;
double sec_wait = -1;



//======    CLASS
class PivotingAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<slipping_control_common::PivotingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    slipping_control_common::PivotingFeedback feedback_;
    slipping_control_common::PivotingResult result_;
    ros::Publisher force_command_pub;
    ros::Subscriber wrench_sub;
    ros::Subscriber grasp_force_sub;

public:

  PivotingAction(std::string name) :
    as_(nh_, name, boost::bind(&PivotingAction::executeCB, this, _1), false),
    action_name_(name),
    force_command_pub( nh_.advertise<std_msgs::Float64>( topic_force_command,1) ),
    wrench_sub(nh_.subscribe(topic_wrench, 1,   &PivotingAction::readForces,this ) ),
    grasp_force_sub(nh_.subscribe(topic_grasp_force, 1, &PivotingAction::readGraspForce,this) )
  {
    as_.start();
  }

  ~PivotingAction(void)
  {
  }

    void executeCB(const slipping_control_common::PivotingGoalConstPtr &goal)
    {
        // helper variables
        double ang_d = goal->ang/180.0*M_PI;

        std_msgs::Float64 fp;
        fp.data = fz;
        double tau_d = fabs(tauz*cos(ang_d));
        minForce = ft/mu_;
        cout << HEADER_PRINT << "Ft = " << ft << " /mu = " << minForce << endl;
        ros::Rate loop_rate(rate);
        bool not_preempted = true;

        // publish info to the console for the user
        cout << HEADER_PRINT << "Action Pivoting Start..." << endl;

        cout << HEADER_PRINT << "tauz = "<< tauz << endl;
        cout << HEADER_PRINT << "taud = "<< tau_d << endl;

        // start executing the action
        //run_pivoting
        double sec_init_ = ros::Time::now().toSec();
        double sec = ros::Time::now().toSec() - sec_init_;
        //if ang_d = 90 then the only stop condition is ft/mu
        while( ros::ok() && sec < MAX_TIME && ( goal->ang == 90.0 ||( (tauz - tau_d) > TAU_THR ) ) && (fp.data > minForce) && (fz > minForce) ){

            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                not_preempted = false;
                break;
            }

            sec = ros::Time::now().toSec() - sec_init_;
	
            
            if(goal->ang == 90.0){
                /* Con il denk mit exp 2a funziona
                if(tauz > 0.010)
                    fp.data =   fp.data - (gain/rate)*(tauz - tau_d);
                else
                    fp.data =   fp.data - (gain/rate)*(fp.data - minForce)/35.0;
                */
                //******NEW
                minForce = ft/mu_;
                fp.data =   fp.data - (gain/rate)*(fp.data - minForce)/35.0;
                //*****END NEW
            } else if(tauz > tau_d) {
			    fp.data =   fp.data - (gain/rate)*(tauz - tau_d);
            }

            //cout << fp.data << endl;
            force_command_pub.publish(fp);

            // publish the feedback
            feedback_.fn = fp.data;
            feedback_.taun = tauz;
            as_.publishFeedback(feedback_);

            ros::spinOnce();
            loop_rate.sleep();
	
	    }

        cout << "while end | tauz = " << tauz << " | taud = " <<  tau_d << endl;

        if(not_preempted){

            if(fp.data <= 0.0){

                result_.success = false;
                result_.error = PIV_ERR_CODE_ZERO_FN;
                cout << HEADER_PRINT << BOLDRED << "F_grasp final = 0 !" << CRESET << endl;

            } else if( (tauz - tau_d) <= TAU_THR ){

                result_.success = true;
                result_.error = PIV_ERR_CODE_OK;
                cout << HEADER_PRINT << "(tauz - tau_d)=" << (tauz - tau_d) << endl;
                cout << HEADER_PRINT << "tauz=" << tauz << endl;
                cout << HEADER_PRINT << "TAU_THR=" << TAU_THR << endl;
                cout << HEADER_PRINT << "Pivoting " << BOLDGREEN << "DONE!" << CRESET << " | Tauz = " << tauz << endl;
            
            } else if(sec >= MAX_TIME){

                result_.success = false;
                result_.error = PIV_ERR_CODE_MAX_TIME;
                cout << HEADER_PRINT <<  BOLDRED << "MAX_TIME [" << MAX_TIME << "] expired!" << CRESET << endl;

            } else if( fp.data <= minForce || (fz <= minForce)){

                //aspetta 4sec or tau = 0;
                cout << HEADER_PRINT << YELLOW << "Wait " << sec_wait << " sec" << endl;
                sec_init_ = ros::Time::now().toSec();
                sec = 0.0;
                while( ros::ok() && sec < sec_wait && (  (tauz - tau_d) > TAU_THR ) ){
                    sec = ros::Time::now().toSec() - sec_init_;
                    ros::spinOnce();
                    loop_rate.sleep();
                }

                if( (tauz - tau_d) <= TAU_THR ){
                    cout << HEADER_PRINT << GREEN << "Pivoting OK" << endl;
                    result_.success = true;
                    result_.error = PIV_ERR_CODE_OK;
                } else {
                    cout << HEADER_PRINT << BOLDRED << "F_grasp final <= minForce !" << CRESET << endl;
                    result_.success = false;
                    result_.error = PIV_ERR_CODE_MIN_FN;
                }

 
            }

            // set the action state to succeeded
            as_.setSucceeded(result_);
        
        }

    }

    double  fz = 0.0, tauz = 0.0, ft = 0.0, minForce = 0.0;
    void readForces(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg){

        ft = sqrt( pow(forceMsg->wrench.force.x,2) + pow(forceMsg->wrench.force.y,2) );
        
        tauz = fabs(forceMsg->wrench.torque.z);

    }

    void readGraspForce(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg){
        fz = fabs(forceMsg->wrench.force.z);
    }


};
// =======  END CLASS


int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "pivoting_generator");

    ros::NodeHandle nh_private("~");

    nh_private.param("mu" , mu_, 0.8 );
    nh_private.param("max_time" , MAX_TIME, 30.0 );
    nh_private.param("tau_thr" , TAU_THR, 3E-3 );
    nh_private.param("gain" , gain, 200.0 );
    nh_private.param("rate" , rate, 500.0 );
	 
    nh_private.param("topic_grasp_force" , topic_grasp_force, string("wrench") );
    nh_private.param("topic_wrench" , topic_wrench, string("grasp_force") );
    
    nh_private.param("topic_force_command" , topic_force_command, string("commandForce") );

    string pivoting_action("");
    nh_private.param("pivoting_action" , pivoting_action, string("action_pivoting") );

    PivotingAction piv_ac(pivoting_action);
    ros::spin();

    return 0;

}
