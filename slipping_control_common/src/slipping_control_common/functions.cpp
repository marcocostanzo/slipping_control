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

#include "slipping_control_common/functions.h"


using namespace std;
using namespace TooN;

double computeAlpha( const LS_INFO& ls_info ){
    return (3.0*M_PI/16.0) * ls_info.mu_ * ls_info.beta_;
}

double calculateSigma( double ft, double taun, const LS_INFO& info ){
    ( info.alpha_/pow(info.mu_,info.gamma_+1.0) ) * ( pow(ft,info.gamma_+1.0) / taun );
}

double computeCOR_R(double sigma_, double MAX_SIGMA){
    if( sigma_ <= MAX_SIGMA)
        return __ann_COR_R__->compute(makeVector(sigma_))[0];
    else
        return __ann_COR_R__->compute(makeVector(MAX_SIGMA))[0];
}

double min_force_Jcst( double fn, const boost::function<double(double)>& limitSurface, double ft, double taun ,const LS_INFO& info ){

	fn = fabs(fn);
    if(fn == 0.0)
        return 10.0;

    double ft_norm = ft/(info.mu_*fn);
    if(ft_norm > 1.0)
        ft_norm = 1.0;
    

    return ( limitSurface( ft_norm ) - (1.0/info.alpha_) * taun / pow(fn,1.0+info.gamma_) );

}

double min_force_gradJ( double fn, const boost::function<double(double)>& diff_limitSurface, double ft, double taun ,const LS_INFO& info  ){


    double segn;
    if(fn < 0.0)
        segn = -1.0;
    else if(fn > 0.0)
        segn = 1.0;
    else
        return -1;
        
    fn = fabs(fn);
    
    double ft_norm = ft/(info.mu_*fn);
    if(ft_norm < 1.0)
        return segn*( -diff_limitSurface( ft_norm )*ft/(info.mu_*pow(fn,2))  + (1.0/info.alpha_)*taun*(1.0+info.gamma_)/pow(fn,info.gamma_+2.0) );
    else
        return segn*( (1.0/info.alpha_)*taun*(1.0+info.gamma_)/pow(fn,info.gamma_+2.0) );

}

double limitSurface_true( double fnk ){
    return polyval( __ls_vector__, fnk  );    
}

double diff_limitSurface_true( double fnk ){
    return polyval( __diff_ls_vector__, fnk  );
}

void initANN_COR_R(){
    string path("");
    path = ros::package::getPath("slipping_control_common");
    path += "/ANN_COR";

    __ann_COR_R__ = new ANN ( path );
}