/*
    Node that compute the LimitSurface informations

    Copyright 2018-2019 Università della Campania Luigi Vanvitelli

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
#include "slipping_control_common/LSStamped.h"
#include "slipping_control_common/functions.h"
#include "slipping_control_common/ChLSParams.h"
//#include "Helper.h"
//#include "slipping_control_common/SetMu.h"

//#define DEBUG_FZERO

#define SIGMA_DEFAULT_VALUE 0.0
#define SIGMA_MAX_VALUE 30.0

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

using namespace std;

//Msgs
slipping_control_common::LSStamped ls_msg;

//Publishers
ros::Publisher pubLS;

//Params
LS_INFO ls_info;
std::vector<SIGMA_INFO>* sigma_info; //FT
std::vector<GAUSS_INFO>* gauss_info; //TAUN

//Cor_tilde
double MAX_COR_TILDE;

void contact_force_Callback (const slipping_control_common::ContactForcesStamped::ConstPtr& msg) {

    //Sigma
    ls_msg.sigma = calculateSigma( msg->forces.ft, msg->forces.taun, ls_info );
    if(isnan(ls_msg.sigma)){
        ls_msg.sigma = SIGMA_DEFAULT_VALUE;
    }
    if(ls_msg.sigma>SIGMA_MAX_VALUE){
        ls_msg.sigma = SIGMA_MAX_VALUE;
    } else if(ls_msg.sigma<-SIGMA_MAX_VALUE){
        ls_msg.sigma = -SIGMA_MAX_VALUE;
    }

    //Extimate c_tilde
    bool b_max_iter;
    bool b_first_try = true;
    double c_tilde_initial_point = ls_msg.cor_tilde;
    compute_c_tilde:
    double virtual_cor_tilde_tmp = computeCOR_tilde(    
                        ls_msg.sigma, 
                        ls_info.gamma,
                        b_max_iter, 
                        c_tilde_initial_point
                        /*double MAX_SIGMA = 30.0, 
                        double GD_GAIN = 1.0, 
                        double GD_COST_TOL = 1.0E-6, 
                        double FIND_ZERO_LAMBDA = 1.0E-10,
                        int MAX_GD_ITER = 150*/
                        );
    if( isnan(virtual_cor_tilde_tmp) || b_max_iter ){
        //FindZero not converged
        #ifdef DEBUG_FZERO
        cout << HEADER_PRINT YELLOW "FindZero not converged... c_tilde= " << virtual_cor_tilde_tmp << " | b_max_iter= " << (b_max_iter ? "true" : "false") << CRESET << endl;
        #endif
        if(b_first_try){
            #ifdef DEBUG_FZERO
            cout << HEADER_PRINT YELLOW "Retry..." CRESET << endl;
            #endif
            //Decision -> change initial point
            if(ls_msg.sigma>0.0){
                c_tilde_initial_point = -1.0;
            } else if(ls_msg.sigma<=0.0){
                c_tilde_initial_point = 1.0;
            }
            b_first_try = false;
            goto compute_c_tilde;
        }
        //Decision -> do not update virtual_cor_tilde
        cout << HEADER_PRINT RED "FindZero not converged... c_tilde= " << virtual_cor_tilde_tmp << " | b_max_iter= " << (b_max_iter ? "true" : "false") << CRESET << endl;
    } else{
        //FindZero Converged
        ls_msg.cor_tilde = virtual_cor_tilde_tmp;
    }
    //Saturation to avoid noise when cor_tilde->infinity
    if(ls_msg.cor_tilde > MAX_COR_TILDE){
        ls_msg.cor_tilde = MAX_COR_TILDE;
    } else if(ls_msg.cor_tilde < -MAX_COR_TILDE){
        ls_msg.cor_tilde = -MAX_COR_TILDE;
    }

    //Compute Normalized LS
    ls_msg.ft_tilde_ls = ft_tilde_ls_model( ls_msg.cor_tilde, *sigma_info );
    ls_msg.taun_tilde_ls = taun_tilde_ls_model( ls_msg.cor_tilde, *gauss_info );

    //Compute LS & Radius
    ls_msg.ft_ls = ls_msg.ft_tilde_ls*getMaxFt( msg->forces.fn , ls_info.mu );
    ls_msg.taun_ls = ls_msg.taun_tilde_ls*getMaxTaun( msg->forces.fn , ls_info, ls_msg.radius );

    //Compute COR
    ls_msg.cor = ls_msg.radius*ls_msg.cor_tilde;

    //Compute Generalized Max Force (g of LuGre Model)
    ls_msg.generalized_max_force = fabs(ls_msg.taun_ls) + fabs( ls_msg.ft_ls * ls_msg.cor );

    //Compute Generalized Force
    ls_msg.generalized_force = msg->forces.taun - ls_msg.cor*msg->forces.ft;

    //Compute fn_ls
    ls_msg.fn_ls = getFn_ls( msg->forces.ft, msg->forces.taun, ls_msg.ft_tilde_ls, ls_msg.taun_tilde_ls, ls_info );
    //Compute fn_ls_free_pivot
    ls_msg.fn_ls_free_pivot = getFn_ls( msg->forces.ft, msg->forces.taun, 1.0, 0.0, ls_info );

    //Fill header
    ls_msg.header = msg->header;

    //PUB
    pubLS.publish(ls_msg);

}

bool set_params_service_callbk(slipping_control_common::ChLSParams::Request  &req, 
   		 		                slipping_control_common::ChLSParams::Response &res)
{

    cout << HEADER_PRINT "Change LS parameters..." << endl;
    string what_changed;
    if(req.delta >= 0.0){
        ls_info.delta = req.delta;
        what_changed += "delta = " + to_string(ls_info.delta) + " | ";
    }
    if(req.gamma >= 0.0){
        ls_info.gamma = req.gamma;
        what_changed += "gamma = " + to_string(ls_info.gamma) + " | ";
    }
    if(req.mu > 0.0){
        ls_info.mu = req.mu;
        what_changed += "mu = " + to_string(ls_info.mu);
    }
    if(req.k > 0.0){
        cout << HEADER_PRINT RED "Change k not supported" CRESET << endl;
        //ls_info.k = req.k;
    }
    computeLSInfo(ls_info);

    cout << HEADER_PRINT GREEN "Done: " CRESET << what_changed << endl;

    return true;

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "compute_ls");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    //params
    nh_private.param("delta" , ls_info.delta, 0.002 );
    nh_private.param("gamma" , ls_info.gamma, 0.3333 );
    nh_private.param("mu" , ls_info.mu, 0.8 );
    nh_private.param("k" , ls_info.k, 4.0 );
    computeLSInfo(ls_info);

    //Cor_tilde
    nh_private.param("max_cor_tilde" , MAX_COR_TILDE, 100.0 );
    MAX_COR_TILDE = fabs(MAX_COR_TILDE);

    //Sub
    string contact_force_topic_str("");
    nh_private.param("contact_force_topic" , contact_force_topic_str, string("contact_force") );

    //Pubs
    string ls_topic_str("");
    nh_private.param("ls_topic" , ls_topic_str, string("ls") );

    //Subs
    ros::Subscriber subContact = nh_public.subscribe( contact_force_topic_str, 1, contact_force_Callback);

    //Pubs
    pubLS = nh_public.advertise<slipping_control_common::LSStamped>(ls_topic_str, 1);

    initLS_model(ls_info.k, ros::package::getPath("slipping_control_common") + "/LS_models/"); //<-- error in there is no file with that k
    LS_model_get_ref(sigma_info, gauss_info);

    //Server Change parameters
    string ch_params_server_str("");
    nh_private.param("service_ch_params" , ch_params_server_str, string("ls_change_params") );
    ros::ServiceServer serviceChMu = nh_public.advertiseService(ch_params_server_str, set_params_service_callbk);
    
    
    ros::spin();

    return 0;

}
