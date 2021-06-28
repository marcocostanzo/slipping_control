/*
    helper functions for slipping_control

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

#ifndef _SLIPPING_CONTROL_COMMON_FUNCTIONS_
#define _SLIPPING_CONTROL_COMMON_FUNCTIONS_

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <iomanip>
#include "boost/function.hpp"
#include "learn_algs/learn_algs.h"
#include "ros/package.h"

#include "TooN/TooN.h"

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLD "\033[1m"                /* Bold */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */
/*===============================*/

#endif

#define INIT_LS_MODEL_FILE_NAME_DIGIT_PRECISION 2

#define MODEL_FILE_EXT ".txt"

struct LS_INFO
{
  double mu;
  double k;
  double gamma;
  double delta;
  double xik_nuk;
  double alpha;  // Deprecated
};

struct SIGMA_INFO
{
  double gain;
  double exponent;
  double mean;
};

struct GAUSS_INFO
{
  double gain;
  double mean;
  double sigma_square;
};

struct VEL_SYSTEM_INFO
{
  double Io;
  double Mo;
  double cor;
  double b;
  double beta_o2;
  double beta_o3;
  double sigma_02;
  double sigma_03;
  double f_max_0;
  double f_max_1;
};

/*
  Update LS struct
*/
void computeLSInfo(LS_INFO& ls_info);

/*
 Compute the product Xik*Nuk using gamma functions
*/
double getXikNuk(double k);

/*
  Deprecated
  Compute alpha 2*mu*xik*nuk*delta
*/
double computeAlpha(const LS_INFO& ls_info);

/*
  Compute sigma from non-normalized forces
  2*xik*nuk*delta/(mu^gamma) * (ft^(gamma+1))/taun
  N.B. the pow is signed
*/
double calculateSigma(double ft, double taun, const LS_INFO& info);

/*
  Compute sigma from normalized forces
  (ft_tilde^(gamma+1))/taun_tilde
  N.B. the pow is signed
*/
double calculateSigma(double ft_tilde, double taun_tilde, double gamma);

/*
  Compute maximum tangential force
  ft = mu*fn
*/
double getMaxFt(double fn, double mu);

/*
  Compute simultaneously the maximum normal torque and the radius of the contact area
*/
double getMaxTaun(double fn, const LS_INFO& info, double& radius);

/*
  Compute the maximum normal torque
*/
double getMaxTaun(double fn, const LS_INFO& info);

/*
  Compute the radius
*/
double getRadius(double fn, const LS_INFO& info);

/*
  Check if (ft_friction_tilde, taun_friction_tilde) is inside the NLS
  Inside it computes c_tilde that is correct only if the point is inside the NLS (i.e. if the function returns true)
  the condition does not depend on gamma, but c_tilde does, the gamma parameter is important only if the output c_tilde
  is used
*/
bool isInsideNLS(double ft_friction_tilde, double taun_friction_tilde, double& c_tilde,
                 double gamma = 0.3333333333333333333333333333);

/*
  Check if (ft_friction_tilde, taun_friction_tilde) is inside the NLS
*/
bool isInsideNLS(double ft_friction_tilde, double taun_friction_tilde);

/*
  Compute cor_tilde (VCOR or COR).
  It can throw an exception if the algorithm does not converged for some reason
*/
double computeCOR_tilde(double ft_friction_tilde, double taun_friction_tilde, double gamma, double xik_nuk,
                        double FT_TILDE_EPS, double TAUN_TILDE_EPS, bool& is_inside_ls);

/*
  Compute cor_tilde (COR) in the hypotesis that the friction point is OUTSIDE the LS.
  this use a Newton method
  It can throw an exception if the algorithm does not converged for some reason
*/
double computeCOR_outsideLS_tilde(double ft_friction_tilde, double taun_friction_tilde, double xik_nuk);

/*
  Compute cor_tilde (VCOR) in the hypotesis that the friction point is INSIDE the LS.
  This function invert the c_tilde->|sigma| function
  this use a Newton method.
  It can throw an exception if the algorithm does not converged for some reason
*/
double computeCOR_insideLS_tilde(double ft_friction_tilde, double taun_friction_tilde, double gamma);

double ft_ls_star_tilde(double c_tilde, const std::vector<SIGMA_INFO>& info);

double taun_ls_star_tilde(double c_tilde, const std::vector<GAUSS_INFO>& info);

/*Support functions*/
double c_tilde_insideLS_J_zero(double c_tilde, double sigma, double gamma,
                               const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                               const boost::function<double(double)>& taun_tilde_star_ls_fcn);

double c_tilde_insideLS_grad_J_zero(double c_tilde, double sigma, double gamma,
                                    const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                                    const boost::function<double(double)>& d_ft_tilde_star_ls_fcn,
                                    const boost::function<double(double)>& taun_tilde_star_ls_fcn,
                                    const boost::function<double(double)>& d_taun_tilde_star_ls_fcn);

double c_tilde_outsideLS_J_zero(double c_tilde, double ft_friction_tilde, double taun_friction_tilde, double xik_nuk,
                                const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                                const boost::function<double(double)>& taun_tilde_star_ls_fcn);

double c_tilde_outsideLS_grad_J_zero(double c_tilde, double ft_friction_tilde, double taun_friction_tilde,
                                     double xik_nuk, const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                                     const boost::function<double(double)>& d_ft_tilde_star_ls_fcn,
                                     const boost::function<double(double)>& taun_tilde_star_ls_fcn,
                                     const boost::function<double(double)>& d_taun_tilde_star_ls_fcn);

double sigm_fun(double x, double gain, double exponent, double mean);

double d_ft_ls_star_tilde(double c_tilde, const std::vector<SIGMA_INFO>& info);

double d_sigm_fun(double x, double gain, double exponent, double mean);

double gauss_fun(double x, double gain, double mean, double sigma_square);

double d_taun_ls_star_tilde(double c_tilde, const std::vector<GAUSS_INFO>& info);

double d_gauss_fun(double x, double gain, double mean, double sigma_square);

/*
  Compute Fn_ls,
  This fcn uses the most (numerically) roboust way
*/
double getFn_ls(double ft, double taun, double ft_tilde_ls, double taun_tilde_ls, const LS_INFO& info);

/*
  Initialize global variables for the Limit Surface model (gaussian and sigmoid)
*/
void initLS_model(double k, const std::string& folder);

/*
  this function changes the input pointers, they will refer the sigma and gauss parameter for ft and tau.
*/
void LS_model_get_ref(std::vector<SIGMA_INFO>*& sigma_info, std::vector<GAUSS_INFO>*& gauss_info);

TooN::Vector<> vel_sys_h_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_HH_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Vector<> vel_sys_f_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_FF_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

//* VEL 1DOF *//

TooN::Vector<> vel_sys_1dof_h_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_1dof_HH_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Vector<> vel_sys_1dof_f_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_1dof_FF_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

//* OBS VEL VEL 1DOF *//

TooN::Vector<> obs_vel_sys_1dof_f_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& y, const VEL_SYSTEM_INFO& info, double l);

TooN::Vector<> obs_vel_sys_1dof_h_fcn(const TooN::Vector<>& x, const TooN::Vector<>& y, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> obs_vel_sys_1dof_FF_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& y, const VEL_SYSTEM_INFO& info, double l);

TooN::Matrix<> obs_vel_sys_HH_fcn(const TooN::Vector<>& x, const TooN::Vector<>& y, const VEL_SYSTEM_INFO& info);

#endif