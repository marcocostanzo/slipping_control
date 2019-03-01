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
#include "ANN/ANN.h"
#include "ros/package.h"
#include "boost/function.hpp"

struct LS_INFO {
  double mu_;
  double alpha_;
  double gamma_;
  double beta_;
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

double computeAlpha( const LS_INFO& ls_info );

double calculateSigma( double ft, double taun, const LS_INFO& info );

double computeCOR_R(double sigma_, double MAX_SIGMA);

double min_force_Jcst( double fn, const boost::function<double(double)>& limitSurface, double ft, double taun ,const LS_INFO& info );

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