/*
    ROS node to generate a ramp grasp force

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
#include <slipping_control_common/GraspingAction.h>

#include "Helper.h"


#define HEADER_PRINT BOLDYELLOW "[Grasp Ramp Generator]: " CRESET


using namespace std;

double RATE, CONTACT_FORCE_THR, CONTACT_FN_FORCE, FORCE_TIME_RATIO;

string topic_grasp_force_str("");
string topic_wrench_str("");


//======    CLASS
class GraspingAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<slipping_control_common::GraspingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    slipping_control_common::GraspingFeedback feedback_;
    slipping_control_common::GraspingResult result_;

    ros::Subscriber grasp_force_sub;
    ros::Subscriber wrench_sub;

public:

  GraspingAction(std::string name) :
    as_(nh_, name, boost::bind(&GraspingAction::executeCB, this, _1), false),
    action_name_(name),

    grasp_force_sub(nh_.subscribe(topic_grasp_force_str, 1, &GraspingAction::readGraspForce,this) ),
    wrench_sub(nh_.subscribe(topic_wrench_str, 1, &GraspingAction::readWrench,this) )
  {
    as_.start();
  }

  ~GraspingAction(void)
  {
  }

    void executeCB(const slipping_control_common::GraspingGoalConstPtr &goal)
    {
        // helper variables
        bool not_preempted = true;
        double goal_force = fabs(goal->force);

        // publish info to the console for the user
        cout << HEADER_PRINT << "Action Grasping Start..." << endl;

        //wait for grasp_force fz
        grasp_force = -1.0;
        fz = -1.0;
        while(grasp_force == -1.0 || fz == -1.0){
            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                not_preempted = false;
                return;
            }
            ros::spinOnce();
        }

        ros::Rate loop_rate(RATE);

        //Check if not in contact
        cout << HEADER_PRINT << "1) grasp_force = " << grasp_force << " CONTACT_FORCE_THR = " << CONTACT_FORCE_THR << " f1 = " << f1
                << "f2 = " << f2 << " goal_force = " << goal_force << endl;
        if( (grasp_force <  CONTACT_FORCE_THR || f1 < CONTACT_FORCE_THR/2.0 || f2 < CONTACT_FORCE_THR/2.0) && grasp_force < goal_force){
            cout << HEADER_PRINT << "Starting contact..." << endl;
            //do contact
            double contact_thr_force_local = (goal_force < CONTACT_FORCE_THR) ? goal_force : CONTACT_FORCE_THR;
            double force_to_apply = CONTACT_FN_FORCE;

            while ( ros::ok() && (grasp_force <  contact_thr_force_local || f1 < contact_thr_force_local/2.0 || f2 < contact_thr_force_local/2.0) ){
                if (as_.isPreemptRequested() || !ros::ok()) {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    // set the action state to preempted
                    as_.setPreempted();
                    not_preempted = false;
                    return;
                }

                feedback_.force = force_to_apply;
                as_.publishFeedback(feedback_);
                
                ros::spinOnce();
                loop_rate.sleep();
            }
            cout << HEADER_PRINT << "2) grasp_force = " << grasp_force << " CONTACT_FORCE_THR = " << CONTACT_FORCE_THR << " f1 = " << f1
                << "f2 = " << f2 << " goal_force = " << goal_force << endl;
            feedback_.force = contact_thr_force_local;
            as_.publishFeedback(feedback_);
        }
        cout << HEADER_PRINT << "Contact " GREEN "OK" CRESET << endl;
        double f_init = grasp_force;

        //do ramp    
        double total_time = (goal_force - f_init)/FORCE_TIME_RATIO;
        double direction = 1.0;
        if(total_time < 0.0){
            total_time = -total_time;
            direction = -1.0;
        }
        cout << HEADER_PRINT << "Starting ramp... total time = " << total_time << endl;

        double sec = 0.0;  
        double sec_init = ros::Time::now().toSec();  
        while ( ros::ok() && sec <= total_time ) {
            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                not_preempted = false;
                return;
            }

            sec = ros::Time::now().toSec() - sec_init;
        
            feedback_.force = f_init + sec * FORCE_TIME_RATIO * direction;
            as_.publishFeedback(feedback_);
            
            loop_rate.sleep();
            ros::spinOnce();
	    }

        //publish final force
        feedback_.force = goal_force;
        as_.publishFeedback(feedback_);

        cout << HEADER_PRINT << "Ramp " GREEN "OK" CRESET << endl;

        if(not_preempted){

            result_.success = true;
            result_.error = 0;
            as_.setSucceeded(result_);

        }

    }

/*====== ROS CALLBK ======*/
    double  grasp_force = 0.0;
    void readGraspForce(const std_msgs::Float64::ConstPtr& forceMsg){
        grasp_force = fabs(forceMsg->data);
        calcF1F2();
    }

    double  fz = 0.0;
    void readWrench(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg){
        fz = fabs(forceMsg->wrench.force.z);
        calcF1F2();
    }

    double f1 = 0.0, f2 = 0.0;
    void calcF1F2(){
        f1 = (grasp_force + fz)/2.0;
        f2 = fabs((grasp_force - fz)/2.0);
        //f2 = 1000.0;
    }

};
// =======  END CLASS


int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "grasping_generator");

    ros::NodeHandle nh_private("~");

    nh_private.param("contact_fn_force" , CONTACT_FN_FORCE, 3.0  );
    nh_private.param("contact_force_thr" , CONTACT_FORCE_THR, 1.0  );
    nh_private.param("grasp_force_time_ratio" , FORCE_TIME_RATIO, 3.0  ); //[N/s]
    nh_private.param("grasp_rate" , RATE, 500.0  );
	 
    nh_private.param("topic_grasp_force" , topic_grasp_force_str, string("/wsg_50_driver_sun/finger0/wrench") );
    nh_private.param("topic_wrench" , topic_wrench_str, string("/wsg_50_driver_sun/finger0/wrench") );

    string grasping_action_server_str("");
    nh_private.param("grasping_action_server" , grasping_action_server_str, string("action_grasping") );

    GraspingAction grasping_ac(grasping_action_server_str);
    ros::spin();

    return 0;

}
