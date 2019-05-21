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
//#include "Helper.h"
//#include "slipping_control_common/SetMu.h"

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

using namespace std;

//Msgs
slipping_control_common::LSStamped ls_msg;

//Publishers
ros::Publisher pubLS;

//Params
LS_INFO ls_info;

//Cor_tilde
double MAX_COR_TILDE;

void contact_force_Callback (const slipping_control_common::ContactForcesStamped::ConstPtr& msg) {

    //Sigma
    ls_msg.sigma = calculateSigma( msg->forces.ft, msg->forces.taun, ls_info );

    //Extimate c_tilde
    bool b_max_iter;
    double virtual_cor_tilde_tmp = computeCOR_tilde(    
                        ls_msg.sigma, 
                        ls_info.gamma,
                        b_max_iter, 
                        ls_msg.cor_tilde
                        /*double MAX_SIGMA = 30.0, 
                        double GD_GAIN = 1.0, 
                        double GD_COST_TOL = 1.0E-6, 
                        double FIND_ZERO_LAMBDA = 1.0E-10,
                        int MAX_GD_ITER = 150*/
                        );
    if( isnan(ls_msg.cor_tilde) || b_max_iter ){
        //FindZero not converged
        //Decision -> do not update virtual_cor_tilde
    } else{
        //FindZero Converged
        ls_msg.cor_tilde = virtual_cor_tilde_tmp;
    }
    //Saturation to avoid noise when cor_tilde->infinity
    if(ls_msg.cor_tilde > MAX_COR_TILDE){
        ls_msg.cor_tilde = MAX_COR_TILDE;
    } else if(ls_msg.cor_tilde > -MAX_COR_TILDE){
        ls_msg.cor_tilde = -MAX_COR_TILDE;
    }

    //Compute Normalized LS
    ls_msg.ft_tilde_ls = ft_tilde_ls_model( ls_msg.cor_tilde, __SIGMA_INFO__ );
    ls_msg.taun_tilde_ls = taun_tilde_ls_model( ls_msg.cor_tilde, __GAUSS_INFO__ );

    //Compute LS & Radius
    ls_msg.ft_ls = ls_msg.ft_tilde_ls*getMaxFt( msg->forces.fn , ls_info.mu );
    ls_msg.taun_ls = ls_msg.taun_tilde_ls*getMaxTaun( msg->forces.fn , ls_info, ls_msg.radius );

    //Compute COR
    ls_msg.cor = ls_msg.radius*ls_msg.cor;

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

/*
bool ch_mu_callbk(slipping_control_common::SetMu::Request  &req, 
   		 		slipping_control_common::SetMu::Response &res){

    cout << HEADER_PRINT << "Service Change mu: " << req.mu << endl;

    if(req.mu<0.0){
        cout << HEADER_PRINT << BOLDRED "ERROR!" << endl;
        res.success = false;
        return true;
    }

    ls_info.mu_ = req.mu;
    ls_info.alpha_ = computeAlpha( ls_info );

    cout << HEADER_PRINT << GREEN "Service Change mu OK " << endl;

    res.success = true;
	return true;

}
*/

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
    nh_private.param("max_cor_tilde" , MAX_COR_TILDE, 1.0 );
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

    //initLS_model(ls_info.k, ros::package::getPath("slipping_control_common") + "/LS_models/"); //<-- error in there is no file with that k

    //Server Mu
    /*
    string mu_server_str("");
    nh_private.param("service_ch_mu" , mu_server_str, string("change_mu") );
    ros::ServiceServer serviceChMu = nh_public.advertiseService(mu_server_str, ch_mu_callbk);
    */

    cout << __SIGMA_INFO__.num_sigm << endl;
    cout << __GAUSS_INFO__.num_gauss << endl;
    
    ros::spin();

    return 0;

}
