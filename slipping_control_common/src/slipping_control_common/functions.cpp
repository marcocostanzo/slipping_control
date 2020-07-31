/*
    helper functions for slipping_control

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

#include "slipping_control_common/functions.h"

using namespace std;
using namespace TooN;

std::vector<SIGMA_INFO> __SIGMA_INFO__;  // FT
std::vector<GAUSS_INFO> __GAUSS_INFO__;  // TAUN

double sign(double n)
{
  return (double)((n > 0) - (n < 0));
}

double pow_signed(double x, double exponent)
{
  return sign(x) * pow(abs(x), exponent);
}

double d_pow_signed(double x, double exponent)
{
  return exponent * pow(abs(x), exponent - 1.0);
}

void computeLSInfo(LS_INFO& ls_info)
{
  ls_info.xik_nuk = getXikNuk(ls_info.k);
  ls_info.alpha = computeAlpha(ls_info);
}

double getXikNuk(double k)
{
  if (isinf(k))
  {
    return 1.0 / 3.0;
  }
  if (k == 0)
  {
    return 0.0;
  }
  return (3.0 / 8.0) * pow(tgamma(3.0 / k), 2) / (tgamma(2.0 / k) * tgamma(4.0 / k));
}

double computeAlpha(const LS_INFO& ls_info)
{
  return 2.0 * ls_info.mu * ls_info.xik_nuk * ls_info.delta;
}

double calculateSigma(double ft, double taun, const LS_INFO& ls_info)
{
  return ((2.0 * ls_info.xik_nuk * ls_info.delta / pow(ls_info.mu, ls_info.gamma)) *
          pow(fabs(ft), ls_info.gamma + 1.0) / taun);
}

double calculateSigma(double ft_tilde, double taun_tilde, double gamma)
{
  return (pow(fabs(ft_tilde), gamma + 1.0) / taun_tilde);
}

double getMaxFt(double fn, double mu)
{
  return mu * fn;
}

double getMaxTaun(double fn, const LS_INFO& ls_info, double& radius)
{
  radius = getRadius(fn, ls_info);
  return 2.0 * ls_info.mu * ls_info.xik_nuk * fn * radius;
}

double getMaxTaun(double fn, const LS_INFO& ls_info)
{
  double radius;
  return getMaxTaun(fn, ls_info, radius);
}

double getRadius(double fn, const LS_INFO& ls_info)
{
  return ls_info.delta * pow(fn, ls_info.gamma);
}

bool isInsideNLS(double ft_friction_tilde, double taun_friction_tilde, double& c_tilde, double gamma)
{
  c_tilde = computeCOR_insideLS_tilde(ft_friction_tilde, taun_friction_tilde, gamma);
  return ((pow(ft_friction_tilde, 2) + pow(taun_friction_tilde, 2)) <=
          (pow(ft_ls_star_tilde(c_tilde, __SIGMA_INFO__), 2) + pow(taun_ls_star_tilde(c_tilde, __GAUSS_INFO__), 2)));
}

bool isInsideNLS(double ft_friction_tilde, double taun_friction_tilde)
{
  double c_tilde;
  return isInsideNLS(ft_friction_tilde, taun_friction_tilde, c_tilde);
}

double computeCOR_tilde(double ft_friction_tilde, double taun_friction_tilde, double gamma, double xik_nuk,
                        double FT_TILDE_EPS, double TAUN_TILDE_EPS, bool& is_inside_nls)
{
  bool taun_is_zero = fabs(taun_friction_tilde) < TAUN_TILDE_EPS;
  bool ft_is_zero = fabs(ft_friction_tilde) < FT_TILDE_EPS;

  if (taun_is_zero && ft_is_zero)
  {
    return NAN;  // CoR undefined
  }

  if (taun_is_zero && !ft_is_zero)
  {
    is_inside_nls = fabs(ft_friction_tilde) < 1.0;
    // CoR INFINITY
    // let assign a sign to the CoR
    if (taun_friction_tilde > 0.0)
    {
      return -INFINITY * sign(ft_friction_tilde);
    }
    if (taun_friction_tilde < 0.0)
    {
      return INFINITY * sign(ft_friction_tilde);
    }
    return INFINITY;
  }

  if (!taun_is_zero && ft_is_zero)
  {
    return 0;
  }

  double c_tilde;
  is_inside_nls = isInsideNLS(ft_friction_tilde, taun_friction_tilde, c_tilde, gamma);
  if (is_inside_nls)
  {
    return c_tilde;
  }

  if (!taun_is_zero)
  {
    // return c_tilde;
    return computeCOR_outsideLS_tilde(ft_friction_tilde, taun_friction_tilde, xik_nuk);
  }

  throw std::runtime_error("computeCOR_tilde:: no codition met. This should not happen");
}

double computeCOR_outsideLS_tilde(double ft_friction_tilde, double taun_friction_tilde, double xik_nuk)
{
  double GD_GAIN = 0.01;
  double GD_COST_TOL = 1.0E-4;
  double FIND_ZERO_LAMBDA = 1.0E-8;
  int MAX_GD_ITER = 10000;

  double initial_point = -sign(ft_friction_tilde) * sign(taun_friction_tilde);

  const boost::function<double(double)> ft_ls_star_tilde_boost =
      boost::bind(ft_ls_star_tilde, _1, boost::ref(__SIGMA_INFO__));

  const boost::function<double(double)> d_ft_ls_star_tilde_boost =
      boost::bind(d_ft_ls_star_tilde, _1, boost::ref(__SIGMA_INFO__));

  const boost::function<double(double)> taun_ls_star_tilde_boost =
      boost::bind(taun_ls_star_tilde, _1, boost::ref(__GAUSS_INFO__));

  const boost::function<double(double)> d_taun_ls_star_tilde_boost =
      boost::bind(d_taun_ls_star_tilde, _1, boost::ref(__GAUSS_INFO__));

  bool b_max_iter;
  double c_tilde = findZero(initial_point,
                            boost::bind(c_tilde_outsideLS_J_zero, _1, ft_friction_tilde, taun_friction_tilde, xik_nuk,
                                        ft_ls_star_tilde_boost, taun_ls_star_tilde_boost),
                            boost::bind(c_tilde_outsideLS_grad_J_zero, _1, ft_friction_tilde, taun_friction_tilde,
                                        xik_nuk, ft_ls_star_tilde_boost, d_ft_ls_star_tilde_boost,
                                        taun_ls_star_tilde_boost, d_taun_ls_star_tilde_boost),
                            GD_GAIN, GD_COST_TOL, FIND_ZERO_LAMBDA, MAX_GD_ITER, b_max_iter);

  if (b_max_iter)
  {
    throw std::runtime_error("findZero max iter in computeCOR_outsideLS_tilde");
  }

  if (std::isnan(c_tilde) || std::isinf(c_tilde))
  {
    throw std::runtime_error("findZero not converged in computeCOR_outsideLS_tilde");
  }

  // check domain
  if (sign(c_tilde) != -sign(ft_friction_tilde) * sign(taun_friction_tilde))
  {
    std::cout << "computeCOR_outsideLS_tilde c_tilde wrong sign, I will fix this. But it is better to check findZero "
                 "params";
    c_tilde = -c_tilde;
  }

  return c_tilde;
}

double computeCOR_insideLS_tilde(double ft_friction_tilde, double taun_friction_tilde, double gamma)
{
  double GD_GAIN = 0.1;
  double GD_COST_TOL = 1.0E-6;
  double FIND_ZERO_LAMBDA = 1.0E-10;
  int MAX_GD_ITER = 10000;

  double sigma = calculateSigma(ft_friction_tilde, taun_friction_tilde, gamma);

  double initial_point = -sign(ft_friction_tilde) * sign(taun_friction_tilde);

  const boost::function<double(double)> ft_ls_star_tilde_boost =
      boost::bind(ft_ls_star_tilde, _1, boost::ref(__SIGMA_INFO__));

  const boost::function<double(double)> d_ft_ls_star_tilde_boost =
      boost::bind(d_ft_ls_star_tilde, _1, boost::ref(__SIGMA_INFO__));

  const boost::function<double(double)> taun_ls_star_tilde_boost =
      boost::bind(taun_ls_star_tilde, _1, boost::ref(__GAUSS_INFO__));

  const boost::function<double(double)> d_taun_ls_star_tilde_boost =
      boost::bind(d_taun_ls_star_tilde, _1, boost::ref(__GAUSS_INFO__));

  bool b_max_iter;
  double c_tilde =
      findZero(initial_point,
               boost::bind(c_tilde_insideLS_J_zero, _1, sigma, gamma, ft_ls_star_tilde_boost, taun_ls_star_tilde_boost),
               boost::bind(c_tilde_insideLS_grad_J_zero, _1, sigma, gamma, ft_ls_star_tilde_boost,
                           d_ft_ls_star_tilde_boost, taun_ls_star_tilde_boost, d_taun_ls_star_tilde_boost),
               GD_GAIN, GD_COST_TOL, FIND_ZERO_LAMBDA, MAX_GD_ITER, b_max_iter);

  if (b_max_iter)
  {
    throw std::runtime_error("findZero max iter in computeCOR_insideLS_tilde");
  }

  if (std::isnan(c_tilde) || std::isinf(c_tilde))
  {
    throw std::runtime_error("findZero not converged in computeCOR_insideLS_tilde");
  }

  // check domain
  if (sign(c_tilde) != -sign(ft_friction_tilde) * sign(taun_friction_tilde))
  {
    std::cout << "computeCOR_insideLS_tilde c_tilde wrong sign, I will fix this. But it is better to check findZero "
                 "params";
    c_tilde = -c_tilde;
  }

  return c_tilde;
}

double c_tilde_insideLS_J_zero(double c_tilde, double sigma, double gamma,
                               const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                               const boost::function<double(double)>& taun_tilde_star_ls_fcn)
{
  return (fabs(sigma) - (pow(fabs(ft_tilde_star_ls_fcn(c_tilde)), gamma + 1.0) / taun_tilde_star_ls_fcn(c_tilde)));
}

double c_tilde_insideLS_grad_J_zero(double c_tilde, double sigma, double gamma,
                                    const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                                    const boost::function<double(double)>& d_ft_tilde_star_ls_fcn,
                                    const boost::function<double(double)>& taun_tilde_star_ls_fcn,
                                    const boost::function<double(double)>& d_taun_tilde_star_ls_fcn)
{
  double ft_tilde_ls = ft_tilde_star_ls_fcn(c_tilde);
  double d_ft_tilde_ls = d_ft_tilde_star_ls_fcn(c_tilde);
  double taun_tilde_ls = taun_tilde_star_ls_fcn(c_tilde);
  double d_taun_tilde_ls = d_taun_tilde_star_ls_fcn(c_tilde);

  double numerator = (gamma + 1) * pow(fabs(ft_tilde_ls), gamma) * sign(ft_tilde_ls) * d_ft_tilde_ls * taun_tilde_ls -
                     pow(fabs(ft_tilde_ls), gamma + 1) * d_taun_tilde_ls;

  return -(numerator / pow(taun_tilde_ls, 2));
}

double c_tilde_outsideLS_J_zero(double c_tilde, double ft_friction_tilde, double taun_friction_tilde, double xik_nuk,
                                const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                                const boost::function<double(double)>& taun_tilde_star_ls_fcn)
{
  double ft_tilde_ls = ft_tilde_star_ls_fcn(c_tilde);
  double taun_tilde_ls = taun_tilde_star_ls_fcn(c_tilde);
  double s_tau = sign(taun_friction_tilde);

  return (ft_friction_tilde - s_tau * ft_tilde_ls +
          4.0 * xik_nuk * c_tilde * (taun_friction_tilde - s_tau * taun_tilde_ls));
}

double c_tilde_outsideLS_grad_J_zero(double c_tilde, double ft_friction_tilde, double taun_friction_tilde,
                                     double xik_nuk, const boost::function<double(double)>& ft_tilde_star_ls_fcn,
                                     const boost::function<double(double)>& d_ft_tilde_star_ls_fcn,
                                     const boost::function<double(double)>& taun_tilde_star_ls_fcn,
                                     const boost::function<double(double)>& d_taun_tilde_star_ls_fcn)
{
  double ft_tilde_ls = ft_tilde_star_ls_fcn(c_tilde);
  double d_ft_tilde_ls = d_ft_tilde_star_ls_fcn(c_tilde);
  double taun_tilde_ls = taun_tilde_star_ls_fcn(c_tilde);
  double d_taun_tilde_ls = d_taun_tilde_star_ls_fcn(c_tilde);
  double s_tau = sign(taun_friction_tilde);

  return (-s_tau * d_ft_tilde_ls +
          4.0 * xik_nuk * (taun_friction_tilde - s_tau * taun_tilde_ls - c_tilde * s_tau * d_taun_tilde_ls));
}

double ft_ls_star_tilde(double c_tilde, const std::vector<SIGMA_INFO>& info)
{
  double ft_tilde = 0.0;
  for (int i = 0; i < info.size(); i++)
  {
    ft_tilde += sigm_fun(c_tilde, info[i].gain, info[i].exponent, info[i].mean);
  }

  return ft_tilde;
}

double sigm_fun(double x, double gain, double exponent, double mean)
{
  return gain * (2.0 / (1.0 + exp(-exponent * (x - mean))) - 1.0);
}

double d_ft_ls_star_tilde(double c_tilde, const std::vector<SIGMA_INFO>& info)
{
  double d_ft_tilde = 0.0;
  for (int i = 0; i < info.size(); i++)
  {
    d_ft_tilde += d_sigm_fun(c_tilde, info[i].gain, info[i].exponent, info[i].mean);
  }

  return d_ft_tilde;
}

double d_sigm_fun(double x, double gain, double exponent, double mean)
{
  return 2.0 * gain * exponent * (exp(-exponent * (x - mean)) / pow(1.0 + exp(-exponent * (x - mean)), 2));
}

double taun_ls_star_tilde(double c_tilde, const std::vector<GAUSS_INFO>& info)
{
  double taun_tilde = 0.0;
  for (int i = 0; i < info.size(); i++)
  {
    taun_tilde += gauss_fun(c_tilde, info[i].gain, info[i].mean, info[i].sigma_square);
  }

  return taun_tilde;
}

double gauss_fun(double x, double gain, double mean, double sigma_square)
{
  return gain * exp(-pow(x - mean, 2) / (2.0 * sigma_square));
}

double d_taun_ls_star_tilde(double c_tilde, const std::vector<GAUSS_INFO>& info)
{
  double d_taun_tilde = 0.0;
  for (int i = 0; i < info.size(); i++)
  {
    d_taun_tilde += d_gauss_fun(c_tilde, info[i].gain, info[i].mean, info[i].sigma_square);
  }

  return d_taun_tilde;
}

double d_gauss_fun(double x, double gain, double mean, double sigma_square)
{
  return -gain * ((x - mean) / sigma_square) * exp(-pow(x - mean, 2) / (2.0 * sigma_square));
}

double getFn_ls(double ft, double taun, double ft_tilde_ls, double taun_tilde_ls, const LS_INFO& ls_info)
{
  ft_tilde_ls = fabs(ft_tilde_ls);
  taun_tilde_ls = fabs(taun_tilde_ls);
  if (ft_tilde_ls > 0.1)
  {
    return (fabs(ft) / ls_info.mu) / ft_tilde_ls;
  }
  else if (taun_tilde_ls > 0.1)
  {
    return pow((fabs(taun) / (2.0 * ls_info.mu * ls_info.xik_nuk * ls_info.delta)) / taun_tilde_ls,
               1.0 / (ls_info.gamma + 1.0));
  }
  else
  {
    // Unable to compute Fn_ls... something goes wrong...
    // Maybe no object is grasped?
    return 0.0;  // I don't know if it is a good value here...
  }
}

void initLS_model(double k, const string& folder)
{
  // Build Folder Name
  ostringstream streamObj;
  // Set Fixed -Point Notation
  streamObj << std::fixed;
  // Set precision to n digits
  streamObj << std::setprecision(INIT_LS_MODEL_FILE_NAME_DIGIT_PRECISION);
  // Add double to stream
  streamObj << k;
  // Get string from output string stream
  string model_file = streamObj.str();
  boost::replace_all(model_file, ".", "_");
  model_file = folder + model_file + MODEL_FILE_EXT;

  // Open File
  FILE* f;
  f = fopen(model_file.c_str(), "r");

  // Chek exist...
  if (f == NULL)
  {
    printf(BOLDRED "Error opening file..." CRESET);
    printf(BOLDBLUE " %s\n" CRESET, model_file.c_str());
    printf(BOLDBLUE " Does the file exist?\n" CRESET);
    throw std::runtime_error("Could not open file");
  }

  // Read First double -> k
  double k_file;
  fscanf(f, "%lf,", &k_file);
  if (k_file != k)
  {
    // There is a problem... it is not the correct file?
    cout << BOLDRED "k does not match... " CRESET << k << "!=" << k_file << endl;
    fclose(f);
    throw std::runtime_error("k of the file does not match");
  }

  // Read ft_model
  int num_sigma;
  double tmp;
  fscanf(f, "%lf,", &tmp);
  num_sigma = (int)tmp;
  __SIGMA_INFO__.clear();
  __SIGMA_INFO__.resize(num_sigma);
  for (int i = 0; i < __SIGMA_INFO__.size(); i++)
  {
    fscanf(f, "%lf,", &tmp);
    __SIGMA_INFO__[i].gain = tmp;
    fscanf(f, "%lf,", &tmp);
    __SIGMA_INFO__[i].mean = tmp;
    fscanf(f, "%lf,", &tmp);
    __SIGMA_INFO__[i].exponent = tmp;
  }

  // Read taun_model
  int num_gauss;
  fscanf(f, "%lf,", &tmp);
  num_gauss = (int)tmp;
  __GAUSS_INFO__.clear();
  __GAUSS_INFO__.resize(num_gauss);
  for (int i = 0; i < __GAUSS_INFO__.size(); i++)
  {
    fscanf(f, "%lf,", &tmp);
    __GAUSS_INFO__[i].gain = tmp;
    fscanf(f, "%lf,", &tmp);
    __GAUSS_INFO__[i].mean = tmp;
    fscanf(f, "%lf,", &tmp);
    __GAUSS_INFO__[i].sigma_square = pow(tmp, 2);
  }

  fclose(f);
}

void LS_model_get_ref(std::vector<SIGMA_INFO>*& sigma_info, std::vector<GAUSS_INFO>*& gauss_info)
{
  sigma_info = &__SIGMA_INFO__;
  gauss_info = &__GAUSS_INFO__;
}

Vector<> vel_sys_h_fcn(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Vector<2> ret = makeVector(info.sigma_02 * x[1] + info.beta_o2 * x[0], info.sigma_03 * x[2] + info.beta_o3 * x[0]);
  return ret;
}

Matrix<> vel_sys_HH_fcn(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Matrix<2, 3> ret = Data(info.beta_o2, info.sigma_02, 0.0, info.beta_o3, 0.0, info.sigma_03);
  return ret;
}

Vector<> vel_sys_f_fcn_cont(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Vector<3> x_dot = Zeros;

  double den = 1.0 / (info.Io + info.Mo * pow(info.cor + info.b, 2));

  x_dot[0] = den * (-(info.beta_o2 + info.beta_o3) * x[0] - info.sigma_02 * x[1] - info.sigma_03 * x[2] + u[0]);

  x_dot[1] = x[0] - info.sigma_02 / info.f_max_0 * fabs(x[0]) * x[1];

  x_dot[2] = x[0] - info.sigma_03 / info.f_max_1 * fabs(x[0]) * x[2];

  return x_dot;
}

Matrix<> vel_sys_FF_fcn_cont(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Matrix<3, 3> F = Zeros;

  double den = 1.0 / (info.Io + info.Mo * pow(info.cor + info.b, 2));

  F(0, 0) = -den * (info.beta_o2 + info.beta_o3);

  F(0, 1) = -info.sigma_02 * den;

  F(0, 2) = -info.sigma_03 * den;

  F(1, 0) = 1.0 - info.sigma_02 / info.f_max_0 * x[1] * sign(x[0]);

  F(1, 1) = -info.sigma_02 / info.f_max_0 * fabs(x[0]);

  F(1, 2) = 0.0;

  F(2, 0) = 1.0 - info.sigma_03 / info.f_max_1 * x[2] * sign(x[0]);

  F(2, 1) = 0.0;

  F(2, 2) = -info.sigma_03 / info.f_max_1 * fabs(x[0]);

  return F;
}

//* VEL 1DOF *//

Vector<> vel_sys_1dof_h_fcn(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Vector<1> ret = makeVector(info.sigma_02 * x[1] + info.beta_o2 * x[0]);
  return ret;
}

Matrix<> vel_sys_1dof_HH_fcn(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Matrix<1, 2> ret = Data(info.beta_o2, info.sigma_02);
  return ret;
}

Vector<> vel_sys_1dof_f_fcn_cont(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Vector<2> x_dot = Zeros;

  double den = 1.0 / (info.Io + info.Mo * pow(info.cor + info.b, 2));

  x_dot[0] = den * (-info.beta_o2 * x[0] - info.sigma_02 * x[1] + u[0]);

  x_dot[1] = x[0] - info.sigma_02 / info.f_max_0 * fabs(x[0]) * x[1];

  return x_dot;
}

Matrix<> vel_sys_1dof_FF_fcn_cont(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info)
{
  Matrix<2, 2> F = Zeros;

  double den = 1.0 / (info.Io + info.Mo * pow(info.cor + info.b, 2));

  F(0, 0) = -den * (info.beta_o2);

  F(0, 1) = -info.sigma_02 * den;

  F(1, 0) = 1.0 - info.sigma_02 / info.f_max_0 * x[1] * sign(x[0]);

  F(1, 1) = -info.sigma_02 / info.f_max_0 * fabs(x[0]);

  return F;
}
