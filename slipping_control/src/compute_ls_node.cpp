/*
    Node that compute the LimitSurface informations

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
#include "slipping_control_common/ContactForcesStamped.h"
#include "slipping_control_common/VirtualCORStamped.h"
#include "slipping_control_common/MaxForcesStamped.h"
#include "std_msgs/Float64.h"
#include "slipping_control_common/functions.h"
#include "Helper.h"
#include "learn_algs/learn_algs.h"

using namespace TooN;
using namespace std;

slipping_control_common::VirtualCORStamped cor_msg;
slipping_control_common::MaxForcesStamped max_forces_msg;
std_msgs::Float64 Generalized_force;
std_msgs::Float64 Fn_min;

ros::Publisher pubCOR;
ros::Publisher pubMinForce;
ros::Publisher pubMaxForce;
ros::Publisher pubGenerForce;

LS_INFO ls_info;
bool taun_trigger_state = true;
double TAUN_MIN, TAUN_MAX, TAUN_WHEN_TRIGGER_ACTIVE;
double MAX_SIGMA, MAX_COR_TILDE;
double DEFAULT_MIN_FN, MIN_FN_MIN;

double GD_GAIN, GD_COST_TOL, FIND_ZERO_LAMBDA;
int MAX_GD_ITER;

void contact_force_Callback (const slipping_control_common::ContactForcesStamped::ConstPtr& msg) {

    //Trigger
    double taun_triggered = SchmittTrigger( msg->forces.taun, taun_trigger_state, TAUN_MIN, TAUN_MAX, TAUN_WHEN_TRIGGER_ACTIVE ); 

    cor_msg.cor.sigma = calculateSigma( msg->forces.ft, taun_triggered, ls_info );

    cor_msg.cor.virtual_cor_tilde = computeCOR_R(cor_msg.cor.sigma, MAX_SIGMA);
    if(cor_msg.cor.virtual_cor_tilde > MAX_COR_TILDE){
        cor_msg.cor.virtual_cor_tilde = MAX_COR_TILDE;
    }

    //Calc min Fn
    bool b_max_iter;
    double initial_point;
    if(msg->forces.fn == 0.0){
        initial_point = DEFAULT_MIN_FN;
    } else{
        initial_point = msg->forces.fn;
    }
    double x = findZero(    initial_point,
                            boost::bind(min_force_Jcst, 
                                        _1, 
                                        limitSurface_true,
                                        msg->forces.ft,
                                        msg->forces.taun,
                                        ls_info
                                        ), 
                            boost::bind(min_force_gradJ, 
                                        _1, 
                                        diff_limitSurface_true,
                                        msg->forces.ft,
                                        msg->forces.taun,
                                        ls_info
                                        ), 
                            GD_GAIN, 
                            GD_COST_TOL, 
                            FIND_ZERO_LAMBDA, 
                            MAX_GD_ITER, 
                            b_max_iter );

    if( !isnan(x) && x!=0.0 && !b_max_iter ){
        Fn_min.data = fabs(x);
    }
    if(Fn_min.data < MIN_FN_MIN){
        Fn_min.data = MIN_FN_MIN;
    }

    cor_msg.cor.virtual_radius = ls_info.beta_ * pow(Fn_min.data, ls_info.gamma_ );
    cor_msg.cor.virtual_cor = cor_msg.cor.virtual_cor_tilde * cor_msg.cor.virtual_radius;

    max_forces_msg.forces.force_frac = msg->forces.fn/Fn_min.data;
    max_forces_msg.forces.ft_max = msg->forces.ft * max_forces_msg.forces.force_frac;
    max_forces_msg.forces.taun_max = msg->forces.taun * pow( max_forces_msg.forces.force_frac , ls_info.gamma_+1.0 );

    max_forces_msg.forces.generalized_max_force = max_forces_msg.forces.taun_max + max_forces_msg.forces.ft_max * cor_msg.cor.virtual_cor;

    Generalized_force.data = msg->forces.taun + msg->forces.ft * cor_msg.cor.virtual_cor;

    //Fill headers
    cor_msg.header = msg->header;
    max_forces_msg.header = msg->header;

    //PUB
    pubCOR.publish(cor_msg);
    pubMinForce.publish(Fn_min);
    pubMaxForce.publish(max_forces_msg);
    pubGenerForce.publish(Generalized_force);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "compute_ls");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    //params
    nh_private.param("beta" , ls_info.beta_, 0.002 );
    nh_private.param("gamma" , ls_info.gamma_, 0.3333 );
    nh_private.param("mu" , ls_info.mu_, 0.8 );
    ls_info.alpha_ = computeAlpha( ls_info );

    nh_private.param("trigger_taun_min" , TAUN_MIN, 0.002 );
    nh_private.param("trigger_taun_max" , TAUN_MAX, 0.010 );
    nh_private.param("trigger_taun_when_trigger_active" , TAUN_WHEN_TRIGGER_ACTIVE, TAUN_MIN );

    nh_private.param("max_sigma" , MAX_SIGMA, 1.0 );
    nh_private.param("max_cor_tilde" , MAX_COR_TILDE, 1.0 );

    nh_private.param("default_fn_min" , DEFAULT_MIN_FN, 20.0 );
    nh_private.param("min_fn_min" , MIN_FN_MIN, 2.0 );
    nh_private.param("newton_gain" , GD_GAIN, 1.0 );
    nh_private.param("newton_cost_tol" , GD_COST_TOL, 1.0E-6 );
    nh_private.param("newton_lambda" , FIND_ZERO_LAMBDA, 1.0E-10 );
    nh_private.param("newton_max_iter" , MAX_GD_ITER, 150 );

    string contact_force_topic_str("");
    nh_private.param("contact_force_topic" , contact_force_topic_str, string("contact_force") );

    string cor_topic_str("");
    nh_private.param("cor_topic" , cor_topic_str, string("cor") );
    string min_force_topic_str("");
    nh_private.param("min_force_topic" , min_force_topic_str, string("min_force") );
    string max_force_topic_str("");
    nh_private.param("max_force_topic" , max_force_topic_str, string("max_force") );
    string gen_force_topic_str("");
    nh_private.param("generalized_force_topic" , gen_force_topic_str, string("generalized_force") );

    //Subs
    ros::Subscriber subContact = nh_public.subscribe( contact_force_topic_str, 1, contact_force_Callback);

    //Pubs
    pubCOR = nh_public.advertise<slipping_control_common::VirtualCORStamped>(cor_topic_str, 1);
    pubMinForce = nh_public.advertise<std_msgs::Float64>(min_force_topic_str, 1);
    pubMaxForce = nh_public.advertise<slipping_control_common::MaxForcesStamped>(max_force_topic_str, 1);
    pubGenerForce = nh_public.advertise<std_msgs::Float64>(gen_force_topic_str, 1);

    initANN_COR_R();
    
    ros::spin();

    return 0;

}
