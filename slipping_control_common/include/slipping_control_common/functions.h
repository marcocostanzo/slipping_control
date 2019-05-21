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

#include "GeometryHelper.h"
#include "learn_algs/learn_algs.h"
#include "ANN/ANN.h"
#include "ros/package.h"
#include "boost/function.hpp"
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <iomanip>

#define INIT_LS_MODEL_FILE_NAME_DIGIT_PRECISION 2

#define MODEL_FILE_EXT ".txt"

struct LS_INFO {
  double mu;
  double k;
  double gamma;
  double delta;
  double xik_nuk;
  double alpha; //Deprecated
};

struct SIGMA_INFO {
  unsigned int num_sigm;
  std::vector<double> gain;
  std::vector<double> exponent;
  std::vector<double> mean;
};

struct GAUSS_INFO {
  unsigned int num_gauss;
  std::vector<double> gain;
  std::vector<double> mean;
  std::vector<double> sigma_square;
};

struct VEL_SYSTEM_INFO {
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

ANN* __ann_COR_R__;

SIGMA_INFO __SIGMA_INFO__; //FT
GAUSS_INFO __GAUSS_INFO__; //TAUN

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
double computeAlpha( const LS_INFO& ls_info );

/*
  Compute sigma from non-normalized forces
  2*xik*nuk*delta/(mu^gamma) * (ft^(gamma+1))/taun
  N.B. the pow is signed
*/
double calculateSigma( double ft, double taun, const LS_INFO& info );

/*
  Compute sigma from normalized forces
  (ft_tilde^(gamma+1))/taun_tilde
  N.B. the pow is signed
*/
double calculateSigma( double ft_tilde, double taun_tilde, double gamma );

/*
  Compute maximum tangential force
  ft = mu*fn
*/
double getMaxFt( double fn, double mu );

/*
  Compute simultaneously the maximum normal torque and the radius of the contact area
*/
double getMaxTaun( double fn , const LS_INFO& info, double& radius );

/*
  Compute the maximum normal torque
*/
double getMaxTaun( double fn , const LS_INFO& info );

/*
  Compute the radius
*/
double getRadius( double fn , const LS_INFO& info );

/*
  Compute cor_tilde, this function invert the c_tilde->sigma function
  this use a Newton method
  b_max_iter is an output. It is true if MAX_GD_ITER is expired.
  If b_max_iter=true or if it returns NaN, the algorithm not converged for some reason.
*/
double computeCOR_tilde(    
                        double sigma, 
                        double gamma,
                        bool& b_max_iter, 
                        double initial_point = 1.0,
                        double MAX_SIGMA = 30.0, 
                        double GD_GAIN = 1.0, 
                        double GD_COST_TOL = 1.0E-6, 
                        double FIND_ZERO_LAMBDA = 1.0E-10,
                        int MAX_GD_ITER = 150
                        );

/*
  Function to use into FindZero for computeCOR_tilde
*/
double c_tilde_J_zero(  
                        double c_tilde, 
                        double sigma,
                        double gamma, 
                        const boost::function<double(double)>& ft_tilde_ls_model_fcn,  
                        const boost::function<double(double)>& taun_tilde_ls_model_fcn 
                      );

/*
  Gradient Function to use into FindZero for computeCOR_tilde
*/
double c_tilde_grad_J_zero(  
                        double c_tilde, 
                        double sigma,
                        double gamma, 
                        const boost::function<double(double)>& ft_tilde_ls_model_fcn,
                        const boost::function<double(double)>& d_ft_tilde_ls_model_fcn,  
                        const boost::function<double(double)>& taun_tilde_ls_model_fcn,
                        const boost::function<double(double)>& d_taun_tilde_ls_model_fcn 
                      );


/*
  Model that aproximate the ft_tilde_ls integral
*/
double ft_tilde_ls_model( double c_tilde, const SIGMA_INFO& info );

/*
  Base function for ft_model
*/
double sigm_fun( double x, double gain, double exponent, double mean );

/*
  Gradient of the Model that aproximate the ft_tilde_ls integral
*/
double d_ft_tilde_ls_model( double c_tilde, const SIGMA_INFO& info );

/*
  Gradient of the Base function for ft_model
*/
double d_sigm_fun( double x, double gain, double exponent, double mean );

/*
  Model that aproximate the taun_tilde_ls integral
*/
double taun_tilde_ls_model( double c_tilde, const GAUSS_INFO& info );

/*
  Base function for taun_model
*/
double gauss_fun( double x, double gain, double mean, double sigma_square );

/*
  Gradient of the Model that aproximate the taun_tilde_ls integral
*/
double d_taun_tilde_ls_model( double c_tilde, const GAUSS_INFO& info );

/*
  Gradient of the Base function for ft_model
*/
double d_gauss_fun( double x, double gain, double mean, double sigma_square );

/*
  Compute Fn_ls,
  This fcn uses the most (numerically) roboust way
*/
double getFn_ls( double ft, double taun, double ft_tilde_ls, double taun_tilde_ls, const LS_INFO& info );

/*
  Initialize global variables for the Limit Surface model (gaussian and sigmoid)
*/
void initLS_model( double k, const std::string& folder );


double min_force_Jcst( 
                      double fn, 
                      const boost::function<double(double)>& limitSurface, 
                      double ft, 
                      double taun,
                      const LS_INFO& info
                      );

double min_force_gradJ( double fn, const boost::function<double(double)>& diff_limitSurface, double ft, double taun ,const LS_INFO& info  );


TooN::Vector<11> __ls_vector__ = TooN::makeVector(
-5.5755199549814404e+02, +2.5600762856660067e+03, -5.0168621788374185e+03, +5.4595553384185614e+03, -3.5993631840014637e+03, 
+1.4748971163000995e+03, -3.7172937778738134e+02, +5.4453338697872042e+01, -4.4753429564631393e+00, -1.7794801753987689e-18, +1.0000000000000000e+00  
);
TooN::Vector<10> __diff_ls_vector__ = polydiff(__ls_vector__);

double limitSurface_true( double fnk );

double diff_limitSurface_true( double fnk );

double limitSurface_line( double ft_n );

double diff_limitSurface_line( double ft_n );

void initANN_COR_R();

TooN::Vector<> vel_sys_h_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_HH_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Vector<> vel_sys_f_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_FF_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

//TRANSL

TooN::Vector<> vel_sys_transl_f_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

TooN::Matrix<> vel_sys_transl_FF_fcn_cont(const TooN::Vector<>& x, const TooN::Vector<>& u, const VEL_SYSTEM_INFO& info);

#endif