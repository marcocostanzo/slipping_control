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

std::vector<SIGMA_INFO> __SIGMA_INFO__; //FT
std::vector<GAUSS_INFO> __GAUSS_INFO__; //TAUN

double pow_signed( double x, double exponent )
{
    return sign(x)*pow(abs(x), exponent);
}

double d_pow_signed( double x, double exponent )
{
    return exponent*pow(abs(x), exponent - 1.0);
}

void computeLSInfo(LS_INFO& ls_info)
{
    ls_info.xik_nuk = getXikNuk(ls_info.k);
    ls_info.alpha = computeAlpha( ls_info );
}

double getXikNuk(double k){
    if(isinf(k)){
        return 1.0/3.0;
    }
    if( k == 0 ){
        return 0.0;
    }
    return (3.0/8.0)  * pow(tgamma(3.0/k),2) / (  tgamma(2.0/k) * tgamma(4.0/k) );
}

double computeAlpha( const LS_INFO& ls_info )
{
    return 2.0*ls_info.mu*ls_info.xik_nuk*ls_info.delta;
}

double calculateSigma( double ft, double taun, const LS_INFO& ls_info )
{
    return ( ( 2.0 * ls_info.xik_nuk * ls_info.delta / pow(ls_info.mu, ls_info.gamma ) ) * pow_signed( ft, ls_info.gamma + 1.0 ) / taun ) ;
}

double calculateSigma( double ft_tilde, double taun_tilde, double gamma )
{
    return ( pow_signed( ft_tilde, gamma + 1.0 ) / taun_tilde );
}

double getMaxFt( double fn, double mu )
{
    return mu*fn;
}

double getMaxTaun( double fn , const LS_INFO& ls_info, double& radius )
{
    radius = getRadius(fn, ls_info);
    return 2.0*ls_info.mu*ls_info.xik_nuk*fn*radius;
}

double getMaxTaun( double fn , const LS_INFO& ls_info )
{
    return 2.0*ls_info.mu*ls_info.xik_nuk*fn*getRadius(fn, ls_info);
}

double getRadius( double fn , const LS_INFO& ls_info )
{
    return ls_info.delta*pow(fn, ls_info.gamma );
}

double computeCOR_tilde(    
                        double sigma, 
                        double gamma,
                        bool& b_max_iter, 
                        double initial_point,
                        double MAX_SIGMA, 
                        double GD_GAIN, 
                        double GD_COST_TOL, 
                        double FIND_ZERO_LAMBDA,
                        int MAX_GD_ITER
                        )
{
    //Numeric Fix
    if( (isinf(sigma) && sigma>0.0) || sigma > fabs(MAX_SIGMA) ){
        sigma = fabs(MAX_SIGMA);
    } else if( (isinf(sigma) && sigma<0.0) || sigma < -fabs(MAX_SIGMA) ) {
        sigma = -fabs(MAX_SIGMA);
    }
    if(initial_point == 0.0){
        initial_point = 1.0;
    }

    const boost::function<double(double)> ft_tilde_ls_boost = 
                                                            boost::bind(
                                                                        ft_tilde_ls_model,
                                                                        _1,
                                                                        boost::ref( __SIGMA_INFO__ )
                                                                        );

    const boost::function<double(double)> d_ft_tilde_ls_boost =
                                                            boost::bind(
                                                                        d_ft_tilde_ls_model,
                                                                        _1,
                                                                        boost::ref( __SIGMA_INFO__ )
                                                                        );

    const boost::function<double(double)> taun_tilde_ls_boost =
                                                            boost::bind(
                                                                        taun_tilde_ls_model,
                                                                        _1,
                                                                        boost::ref( __GAUSS_INFO__ )
                                                                        );

    const boost::function<double(double)> d_taun_tilde_ls_boost =
                                                            boost::bind(
                                                                        d_taun_tilde_ls_model,
                                                                        _1,
                                                                        boost::ref( __GAUSS_INFO__ )
                                                                        );

    return findZero(    
                            initial_point,
                            boost::bind(
                                        c_tilde_J_zero, 
                                        _1, 
                                        sigma,
                                        gamma, 
                                        ft_tilde_ls_boost,  
                                        taun_tilde_ls_boost
                            ), 
                            boost::bind(
                                        c_tilde_grad_J_zero, 
                                        _1, 
                                        sigma,
                                        gamma,
                                        ft_tilde_ls_boost,
                                        d_ft_tilde_ls_boost,
                                        taun_tilde_ls_boost,
                                        d_taun_tilde_ls_boost
                            ), 
                            GD_GAIN, 
                            GD_COST_TOL, 
                            FIND_ZERO_LAMBDA, 
                            MAX_GD_ITER, 
                            b_max_iter 
                            );

}

double c_tilde_J_zero(  
                        double c_tilde, 
                        double sigma,
                        double gamma, 
                        const boost::function<double(double)>& ft_tilde_ls_model_fcn,  
                        const boost::function<double(double)>& taun_tilde_ls_model_fcn 
                      )
{

    return ( sigma - (pow_signed( ft_tilde_ls_model_fcn(c_tilde), gamma + 1.0 ) / taun_tilde_ls_model_fcn(c_tilde) ) );

}

double c_tilde_grad_J_zero(  
                        double c_tilde, 
                        double sigma,
                        double gamma, 
                        const boost::function<double(double)>& ft_tilde_ls_model_fcn,
                        const boost::function<double(double)>& d_ft_tilde_ls_model_fcn,  
                        const boost::function<double(double)>& taun_tilde_ls_model_fcn,
                        const boost::function<double(double)>& d_taun_tilde_ls_model_fcn 
                      )
{

    double ft_tilde_ls = ft_tilde_ls_model_fcn(c_tilde);
    double d_ft_tilde_ls = d_ft_tilde_ls_model_fcn(c_tilde);
    double taun_tilde_ls = taun_tilde_ls_model_fcn(c_tilde);
    double d_taun_tilde_ls = d_taun_tilde_ls_model_fcn(c_tilde);

    double numerator = d_pow_signed( ft_tilde_ls, gamma+1.0 )*d_ft_tilde_ls*taun_tilde_ls  - pow_signed( ft_tilde_ls, gamma+1.0 )*d_taun_tilde_ls;

    return -( numerator/pow(taun_tilde_ls,2) );

}

double ft_tilde_ls_model( double c_tilde, const std::vector<SIGMA_INFO>& info )
{

    double ft_tilde = 0.0;
    for( int i=0; i<info.size(); i++ ){
        ft_tilde += sigm_fun( c_tilde, info[i].gain, info[i].exponent, info[i].mean );
    }

    return ft_tilde;

}

double sigm_fun( double x, double gain, double exponent, double mean )
{

    return gain*( 2.0/( 1.0 + exp( -exponent*( x - mean ) ) ) - 1.0 );

}

double d_ft_tilde_ls_model( double c_tilde, const std::vector<SIGMA_INFO>& info )
{

    double d_ft_tilde = 0.0;
    for( int i=0; i<info.size(); i++ ){
        d_ft_tilde += d_sigm_fun( c_tilde, info[i].gain, info[i].exponent, info[i].mean );
    }

    return d_ft_tilde;

}

double d_sigm_fun( double x, double gain, double exponent, double mean )
{

    return 2.0*gain*exponent*( exp( -exponent*( x - mean ) ) /pow( 1.0 + exp( -exponent*( x - mean ) ), 2 ) );

}

double taun_tilde_ls_model( double c_tilde, const std::vector<GAUSS_INFO>& info )
{

    double taun_tilde = 0.0;
    for( int i=0; i<info.size(); i++ ){
        taun_tilde += gauss_fun( c_tilde, info[i].gain, info[i].mean, info[i].sigma_square );
    }

    return taun_tilde;

}

double gauss_fun( double x, double gain, double mean, double sigma_square )
{
    return gain*exp( -pow( x - mean, 2 )/(2.0*sigma_square) );
}

double d_taun_tilde_ls_model( double c_tilde, const std::vector<GAUSS_INFO>& info )
{

    double d_taun_tilde = 0.0;
    for( int i=0; i<info.size(); i++ ){
        d_taun_tilde += d_gauss_fun( c_tilde, info[i].gain, info[i].mean, info[i].sigma_square );
    }

    return d_taun_tilde;

}

double d_gauss_fun( double x, double gain, double mean, double sigma_square )
{
    return -gain*((x - mean)/sigma_square)*exp( -pow( x - mean, 2 )/(2.0*sigma_square) );
}

double getFn_ls( double ft, double taun, double ft_tilde_ls, double taun_tilde_ls, const LS_INFO& ls_info )
{
    ft_tilde_ls = fabs(ft_tilde_ls);
    taun_tilde_ls = fabs(taun_tilde_ls);
    if(ft_tilde_ls > 0.1){
        return (fabs(ft)/ls_info.mu)/ft_tilde_ls;
    } else if(taun_tilde_ls > 0.1){
        return pow( (fabs(taun)/(2.0*ls_info.mu*ls_info.xik_nuk*ls_info.delta))/taun_tilde_ls , 1.0/(ls_info.gamma + 1.0) );
    } else {
        //Unable to compute Fn_ls... something goes wrong...
        //Maybe no object is grasped?
        return 0.0; //I don't know if it is a good value here...
    }
}

void initLS_model( double k, const string& folder )
{

    //Build Folder Name
    ostringstream streamObj;
    // Set Fixed -Point Notation
    streamObj << std::fixed;
    // Set precision to n digits
    streamObj << std::setprecision(INIT_LS_MODEL_FILE_NAME_DIGIT_PRECISION);
    //Add double to stream
    streamObj << k;
    // Get string from output string stream
    string model_file = streamObj.str();
    boost::replace_all(model_file, ".", "_");
    model_file = folder + model_file + MODEL_FILE_EXT;

    //Open File
    FILE *f;
	f = fopen(model_file.c_str(), "r");

    //Chek exist...
	if (f == NULL){
		printf(BOLDRED "Error opening file..." CRESET);
		printf(BOLDBLUE " %s\n" CRESET,model_file.c_str());
        printf(BOLDBLUE " Does the file exist?\n" CRESET);
		throw std::runtime_error("Could not open file");
	}

    //Read First double -> k
    double k_file;
    fscanf(f, "%lf,", &k_file);
    if(k_file!=k){
        //There is a problem... it is not the correct file?
        cout << BOLDRED "k does not match... " CRESET << k << "!=" << k_file << endl;
        fclose(f);
        throw std::runtime_error("k of the file does not match");
    }

    //Read ft_model
    int num_sigma;
    double tmp;
    fscanf(f, "%lf,", &tmp);
    num_sigma = (int)tmp;
    __SIGMA_INFO__.clear();
    __SIGMA_INFO__.resize(num_sigma);
    for( int i=0; i<__SIGMA_INFO__.size(); i++ ){
        fscanf(f, "%lf,", &tmp);
        __SIGMA_INFO__[i].gain = tmp;
        fscanf(f, "%lf,", &tmp);
        __SIGMA_INFO__[i].mean = tmp;
        fscanf(f, "%lf,", &tmp);
        __SIGMA_INFO__[i].exponent = tmp;
    }

    //Read taun_model
    int num_gauss;
    fscanf(f, "%lf,", &tmp); 
    num_gauss = (int)tmp;
    __GAUSS_INFO__.clear();
    __GAUSS_INFO__.resize(num_gauss);
    for( int i=0; i<__GAUSS_INFO__.size(); i++ ){
        fscanf(f, "%lf,", &tmp);
        __GAUSS_INFO__[i].gain = tmp;
        fscanf(f, "%lf,", &tmp);
        __GAUSS_INFO__[i].mean = tmp;
        fscanf(f, "%lf,", &tmp);
        __GAUSS_INFO__[i].sigma_square = pow(tmp,2);
    }

    fclose(f);

}

void LS_model_get_ref(std::vector<SIGMA_INFO>* &sigma_info, std::vector<GAUSS_INFO>* &gauss_info )
{
    sigma_info = &__SIGMA_INFO__;
    gauss_info = &__GAUSS_INFO__;
}

Vector<> vel_sys_h_fcn(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info){
    Vector<2> ret = makeVector( info.sigma_02*x[1] + info.beta_o2*x[0], info.sigma_03*x[2] + info.beta_o3*x[0]);
    return ret;
}

Matrix<> vel_sys_HH_fcn(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info){
    Matrix<2,3> ret = Data( info.beta_o2 , info.sigma_02, 0.0, info.beta_o3, 0.0, info.sigma_03 );
    return ret;
}

Vector<> vel_sys_f_fcn_cont(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info){
    
    Vector<3> x_dot = Zeros;

    double den = 1.0/( info.Io + info.Mo*pow(info.cor+info.b,2) );

    x_dot[0] = den * ( -(info.beta_o2 + info.beta_o3)*x[0] - info.sigma_02*x[1] - info.sigma_03*x[2] + u[0] );

    x_dot[1] = x[0] - info.sigma_02/info.f_max_0 * fabs(x[0]) * x[1];

    x_dot[2] = x[0] - info.sigma_03/info.f_max_1 * fabs(x[0]) * x[2];

    return x_dot;

}

Matrix<> vel_sys_FF_fcn_cont(const Vector<>& x, const Vector<>& u, const VEL_SYSTEM_INFO& info){
    
    Matrix<3,3> F = Zeros;

    double den = 1.0/( info.Io + info.Mo*pow(info.cor+info.b,2) );

    F(0,0) = -den * ( info.beta_o2 + info.beta_o3 );

    F(0,1) = -info.sigma_02*den;

    F(0,2) = -info.sigma_03*den;

    F(1,0) = 1.0 - info.sigma_02/info.f_max_0 * x[1] * sign( x[0] );

    F(1,1) = -info.sigma_02/info.f_max_0 * fabs( x[0] );

    F(1,2) = 0.0;

    F(2,0) = 1.0 - info.sigma_03/info.f_max_1 * x[2] * sign( x[0] );

    F(2,1) = 0.0;

    F(2,2) = -info.sigma_03/info.f_max_1 * fabs( x[0] );

    return F;

}
