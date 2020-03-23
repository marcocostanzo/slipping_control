/*
    ROS node to calculate the velocity

    Copyright 2019 Università della Campania Luigi Vanvitelli

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

#include <sun_ros_msgs/Float64Stamped.h>
#include "std_srvs/SetBool.h"
#include "sun_systems_lib/TF/TF_SISO.h"

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[Velocity Controller]: " CRESET

using namespace std;

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

bool b_dynamic_system;

ros::Publisher outPub;
ros::Subscriber w_sub;
ros::Subscriber wd_sub;

string w_topic("");
string wd_topic("");
string out_topic("");

double k_w;
bool running = false;

double w = 0.0, w_d = 0.0;
void read_w(const sun_ros_msgs::Float64Stamped::ConstPtr& msg)
{
    w = msg->data;
}

void read_w_and_pub(const sun_ros_msgs::Float64Stamped::ConstPtr& msg){

    if(!running){
        return;
    }

    w = msg->data;

    sun_ros_msgs::Float64Stamped fnd;

    fnd.data =  -k_w*(w_d - w)*sgn(w_d);
    fnd.header = msg->header;

	outPub.publish(fnd);

}

void read_wd(const sun_ros_msgs::Float64Stamped::ConstPtr& msg)
{
    w_d = msg->data;
}

void read_wd_and_pub(const sun_ros_msgs::Float64Stamped::ConstPtr& msg){

    if(!running){
        return;
    }

    w_d = msg->data;

    sun_ros_msgs::Float64Stamped fnd;

    fnd.data =  -k_w*(w_d - w)*sgn(w_d);
    fnd.header = msg->header;

	outPub.publish(fnd);
    
}

sun::TF_SISO* C_tf_ptr;
void reset_tf()
{
    if(b_dynamic_system)
    {
        C_tf_ptr->reset();
        cout << HEADER_PRINT "RESET!" CRESET << endl;
    }
}

/*Pause callback*/
bool setRunning_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

    if(req.data){

        if(!running){
            w = 0.0;
            w_d = 0.0;
            reset_tf();
            cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
        }
        running = true;

    } else{

        w = 0.0;
        w_d = 0.0;
        sun_ros_msgs::Float64Stamped fnd;
        fnd.data = 0.0;
        reset_tf();
        running = false;
        fnd.header.stamp = ros::Time::now();
        outPub.publish(fnd);

        cout << HEADER_PRINT YELLOW "Stopped!" CRESET << endl;

    }

    res.success = true;
	return true;
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "velocity_controller");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;
     
    nh_private.param("w_topic" , w_topic, string("w_topic") );
    nh_private.param("wd_topic" , wd_topic, string("wd_topic") );
    
    nh_private.param("out_topic" , out_topic, string("dyn_force") );

    string set_running_service_str("");
    nh_private.param("set_running_service" , set_running_service_str, string("set_running") );

    nh_private.param("k_w" , k_w, 0.01 );

    b_dynamic_system = true;

	// Publisher Subscriber
    if(b_dynamic_system)
    {
        w_sub = nh_public.subscribe(w_topic, 1, read_w);
        wd_sub = nh_public.subscribe(wd_topic, 1, read_wd);
    } else {
        w_sub = nh_public.subscribe(w_topic, 1, read_w_and_pub);
        wd_sub = nh_public.subscribe(wd_topic, 1, read_wd_and_pub);
    }
    outPub = nh_public.advertise<sun_ros_msgs::Float64Stamped>( out_topic,1);

    ros::ServiceServer serviceSetRunning = nh_public.advertiseService(set_running_service_str, setRunning_callbk);

    if(b_dynamic_system)
    {
        double Ts = 2.0000000000E-02;
        // double Ts = 1.0000000000E-02 ;



        // Ts = 1.0000000000E-02;
        // TF_SISO C_tf( 
        //     TooN::makeVector(9.6002830427E+00, -1.8633876190E+01, 9.0419558552E+00   ) , //const TooN::Vector<>& num_coeff, 
        //     TooN::makeVector(1.0000000000E+00, -1.9894405129E+00, 9.8946838863E-01  ), //const TooN::Vector<>& den_coeff, 
        //     Ts //double Ts
        // );

        //FNS
        // Ts = 1.0000000000E-02;
        // TF_SISO C_tf( 
        //     TooN::makeVector(4.5795721798E+00, -8.5935611836E+00, 4.0314515700E+00    ) , //const TooN::Vector<>& num_coeff, 
        //     TooN::makeVector(1.0000000000E+00, -1.9658800487E+00, 9.6617109145E-01   ), //const TooN::Vector<>& den_coeff, 
        //     Ts //double Ts
        // );

        //INT
        // Ts = 1.0000000000E-02;
        // TF_SISO C_tf( 
        //     TooN::makeVector(2.9645798029E-01, 6.7771966218E-04, -2.9578026063E-01     ) , //const TooN::Vector<>& num_coeff, 
        //     TooN::makeVector(1.0000000000E+00, -1.9864456068E+00, 9.8644560676E-01     ), //const TooN::Vector<>& den_coeff, 
        //     Ts //double Ts
        // );

        //INT FNS
        // Ts = 1.0000000000E-02;
        // TF_SISO C_tf( 
        //     TooN::makeVector(2.9360220664E-01, 4.4234924533E-04, -2.9315985739E-01     ) , //const TooN::Vector<>& num_coeff, 
        //     TooN::makeVector(1.0000000000E+00, -1.9873614501E+00, 9.8736145013E-01     ), //const TooN::Vector<>& den_coeff, 
        //     Ts //double Ts
        // );

        //INT Resina S
        // Ts = 1.0000000000E-02;
        // TF_SISO C_tf( 
        //     TooN::makeVector(2.9116511710E-01, 4.1748868289E-04, -2.9074762842E-01   ) , //const TooN::Vector<>& num_coeff, 
        //     TooN::makeVector(1.0000000000E+00, -1.9880717519E+00, 9.8807175192E-01   ), //const TooN::Vector<>& den_coeff, 
        //     Ts //double Ts
        // );

        //SELLA
        Ts = 1.0000000000E-02;
        sun::TF_SISO C_tf( 
            TooN::makeVector(9.7217712425E-01, -9.4955427898E-01, -9.7210546680E-01, 9.4962593643E-01     ) , //const TooN::Vector<>& num_coeff, 
            TooN::makeVector(1.0000000000E+00, -2.8743715626E+00, 2.7488864401E+00, -8.7451487752E-01    ), //const TooN::Vector<>& den_coeff, 
            Ts //double Ts
        );

        // Ts = 1.0000000000E-02;
        // TF_SISO C_tf( 
        //     TooN::makeVector(6.1143217410E+00, 3.9851878146E-02, -6.0744698629E+00  ) , //const TooN::Vector<>& num_coeff, 
        //     TooN::makeVector(1.0000000000E+00, -1.6014812185E+00, 6.0148121854E-01   ), //const TooN::Vector<>& den_coeff, 
        //     Ts //double Ts
        // );

        C_tf_ptr = &C_tf;

        //Main Loop
        ros::Rate loop_rate(1/Ts);
        while(ros::ok()){

            ros::spinOnce();
            if(!running){
                loop_rate.sleep();
                continue;
            }
            
            //Apply control
            sun_ros_msgs::Float64Stamped fnd;
            // if(sgn(w_d) == 0.0)
            // {
            //     fnd.data = k_w*C_tf.apply( fabs(w) );
            // } else {
            //     fnd.data = -k_w*C_tf.apply( (w_d - w)*sgn(w_d) );
            // }

            if( w_d == 0 ) {
                fnd.data = k_w*C_tf.apply( 0.0 );
            } else {
                fnd.data = k_w*C_tf.apply( (w_d - w) );
            }

            cout << HEADER_PRINT << "w = " << (w) << endl;
            cout << HEADER_PRINT << "w_d = " << (w_d) << endl;
            cout << HEADER_PRINT << "Error = " << (w_d - w) << endl;
            fnd.header.stamp = ros::Time::now();

            /*Security check*/
            if( isnan(fnd.data) || isinf(fnd.data) )
            {
                    cout << HEADER_PRINT << BOLDRED "INF/NAN... EXIT! ..." CRESET << endl;
                    fnd.data = 0.0;
                    outPub.publish(fnd);
                    loop_rate.sleep();
                    exit(-1);
            }
            
            outPub.publish(fnd);

            loop_rate.sleep();

        } //end main loop

    } else {
        ros::spin();
    }

    return 0;
}
