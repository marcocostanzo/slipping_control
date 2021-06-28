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
#include "slipping_control_common/functions.h"
#include "slipping_control_msgs/ChLSParams.h"
#include "slipping_control_msgs/ContactForcesFixStamped.h"
#include "slipping_control_msgs/LSStamped.h"
//#include "Helper.h"
//#include "slipping_control_msgs/SetMu.h"

//#define DEBUG_FZERO
#define LS_USE_QUADRANT_1_4

// #define FT_EPS 0.1
// #define TAUN_EPS 0.00151

// #define MIN_FN 0.05

#define FT_EPS (0.05/2.0)
#define TAUN_EPS 0.0003

#define MIN_FN 0.03

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET

using namespace std;

// Msgs
slipping_control_msgs::LSStamped ls_msg;

// Publishers
ros::Publisher pubLS;

// Params
LS_INFO ls_info;
std::vector<SIGMA_INFO>* sigma_info;  // FT
std::vector<GAUSS_INFO>* gauss_info;  // TAUN

// Cor_tilde
double MAX_COR_TILDE;

/// NEW: FIXED COR
#include "slipping_control_msgs/FixedCoR.h"
bool b_use_fixed_cor;
double fixed_cor_value;
bool fixed_cor_service_callbk(slipping_control_msgs::FixedCoR::Request& req,
                              slipping_control_msgs::FixedCoR::Response& res)
{
  b_use_fixed_cor = req.use_fixed_cor;
  fixed_cor_value = req.fixed_cor_value;
  if(b_use_fixed_cor)
  {
    cout << HEADER_PRINT GREEN "Using fixed CoR: " CRESET << fixed_cor_value << endl;
  }
  else{
    cout << HEADER_PRINT GREEN "Using CoR estimation alg: " CRESET << endl;
  }
  return true;
}

void contact_force_Callback(const slipping_control_msgs::ContactForcesFixStamped::ConstPtr& msg)
{

  if(msg->forces.fn < MIN_FN)
  {
    return;
  }

  double cor_tilde;
  bool is_inside_ls;

  double max_ft = getMaxFt(msg->forces.fn, ls_info.mu);
  double max_taun = getMaxTaun(msg->forces.fn, ls_info, ls_msg.radius);

  double ft_tilde = msg->forces.ft/max_ft;
  double taun_tilde = msg->forces.taun/max_taun;
  double taun_fixed_tilde = msg->forces.taun_fixed/max_taun;
  double ft_tilde_eps = FT_EPS/max_ft;
  double taun_tilde_eps = TAUN_EPS/max_taun;

  try
  {
    cor_tilde = computeCOR_tilde(ft_tilde, taun_fixed_tilde, ls_info.gamma, ls_info.xik_nuk, ft_tilde_eps,
                                 taun_tilde_eps, is_inside_ls);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(HEADER_PRINT << " " << e.what());
    return;
  }

  if (std::isnan(cor_tilde))
  {
    return;
  }

  // Fix cor_tilde values
  if (cor_tilde > MAX_COR_TILDE)
  {
    cor_tilde = MAX_COR_TILDE;
  }
  else if (cor_tilde < -MAX_COR_TILDE)
  {
    cor_tilde = -MAX_COR_TILDE;
  }

/// NEW:
  if(b_use_fixed_cor){
  if(msg->forces.taun>0.0)
  cor_tilde = - fixed_cor_value/ls_msg.radius;
  else
  {
  cor_tilde = + fixed_cor_value/ls_msg.radius;
  }
  }
  

  ls_msg.cor_tilde = cor_tilde;
  ls_msg.is_inside_ls = is_inside_ls;

  // Compute Normalized LS (the actual sign is -sign(omega))
  ls_msg.ft_tilde_ls = ft_ls_star_tilde(ls_msg.cor_tilde, *sigma_info);
  ls_msg.taun_tilde_ls = taun_ls_star_tilde(ls_msg.cor_tilde, *gauss_info);

// This fix is needed to use the 1^ and 4^ quadrant of the Limit Surface
// This is due to the choice made for the contact frame, i.e., ft>0
#ifdef LS_USE_QUADRANT_1_4
  ls_msg.ft_tilde_ls = fabs(ls_msg.ft_tilde_ls);
  ls_msg.taun_tilde_ls = fabs(ls_msg.taun_tilde_ls);
  if (ls_msg.cor_tilde > 0.0)
    ls_msg.taun_tilde_ls = -ls_msg.taun_tilde_ls;
#endif

  ls_msg.sigma = calculateSigma(ls_msg.ft_tilde_ls, ls_msg.taun_tilde_ls, ls_info.gamma);

  // Compute LS & Radius
  ls_msg.ft_ls = ls_msg.ft_tilde_ls * max_ft;
  ls_msg.taun_ls = ls_msg.taun_tilde_ls * max_taun;

  // Compute COR
  ls_msg.cor = ls_msg.radius * ls_msg.cor_tilde;

  // Compute Generalized Max Force (g of LuGre Model)
  ls_msg.generalized_max_force = fabs(ls_msg.taun_ls) + fabs(ls_msg.ft_ls * ls_msg.cor);

  // Compute Generalized Force
  ls_msg.generalized_force = msg->forces.taun - ls_msg.cor * msg->forces.ft;

  // Compute fn_ls
  ls_msg.fn_ls = getFn_ls(msg->forces.ft, msg->forces.taun, ls_msg.ft_tilde_ls, ls_msg.taun_tilde_ls, ls_info);
  // Compute fn_ls_free_pivot
  ls_msg.fn_ls_free_pivot = getFn_ls(msg->forces.ft, msg->forces.taun, 1.0, 0.0, ls_info);

  // Fill header
  ls_msg.header = msg->header;

  // PUB
  pubLS.publish(ls_msg);
}

bool set_params_service_callbk(slipping_control_msgs::ChLSParams::Request& req,
                               slipping_control_msgs::ChLSParams::Response& res)
{
  cout << HEADER_PRINT "Change LS parameters..." << endl;
  string what_changed;
  if (req.delta >= 0.0)
  {
    ls_info.delta = req.delta;
    what_changed += "delta = " + to_string(ls_info.delta) + " | ";
  }
  if (req.gamma >= 0.0)
  {
    ls_info.gamma = req.gamma;
    what_changed += "gamma = " + to_string(ls_info.gamma) + " | ";
  }
  if (req.mu > 0.0)
  {
    ls_info.mu = req.mu;
    what_changed += "mu = " + to_string(ls_info.mu);
  }
  if (req.k > 0.0)
  {
    cout << HEADER_PRINT RED "Change k not supported" CRESET << endl;
    // ls_info.k = req.k;
  }
  computeLSInfo(ls_info);

  cout << HEADER_PRINT GREEN "Done: " CRESET << what_changed << endl;

  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "compute_ls");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  // params
  nh_private.param("delta", ls_info.delta, 0.002);
  nh_private.param("gamma", ls_info.gamma, 0.3333);
  nh_private.param("mu", ls_info.mu, 0.8);
  nh_private.param("k", ls_info.k, 4.0);
  computeLSInfo(ls_info);
  std::cout << "AAAAAAAAAAAAA -- MU: " << ls_info.mu << std::endl;

  // Cor_tilde
  nh_private.param("max_cor_tilde", MAX_COR_TILDE, 100.0);
  MAX_COR_TILDE = fabs(MAX_COR_TILDE);

  // Sub
  string contact_force_topic_str("");
  nh_private.param("contact_force_topic", contact_force_topic_str, string("contact_force"));

  // Pubs
  string ls_topic_str("");
  nh_private.param("ls_topic", ls_topic_str, string("ls"));

  // Subs
  ros::Subscriber subContact = nh_public.subscribe(contact_force_topic_str, 1, contact_force_Callback);

  // Pubs
  pubLS = nh_public.advertise<slipping_control_msgs::LSStamped>(ls_topic_str, 1);

  initLS_model(ls_info.k, ros::package::getPath("slipping_control_common") + "/LS_models/");  //<-- error in there is no
                                                                                              // file with that k
  LS_model_get_ref(sigma_info, gauss_info);

  // Server Change parameters
  string ch_params_server_str("");
  nh_private.param("service_ch_params", ch_params_server_str, string("ls_change_params"));
  ros::ServiceServer serviceChMu = nh_public.advertiseService(ch_params_server_str, set_params_service_callbk);

  // Server Fixed CoR
  string fixed_cor_server_str(ch_params_server_str + "_ls_fixed_cor");
  //nh_private.param("service_ls_fixed_cor", fixed_cor_server_str, string("ls_change_params"));
  ros::ServiceServer serviceFixedCoR = nh_public.advertiseService(fixed_cor_server_str, fixed_cor_service_callbk);

  ros::spin();

  return 0;
}
