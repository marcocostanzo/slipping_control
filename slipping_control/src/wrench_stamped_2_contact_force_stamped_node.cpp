/*
    ROS node to convert wrenchmsg in contact msg

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

#include "ros/ros.h"

#include "geometry_msgs/WrenchStamped.h"
#include "slipping_control_msgs/ContactForcesStamped.h"

using namespace std;

ros::Publisher pubContact;

slipping_control_msgs::ContactForcesStamped contact_force_msg;
void wrenchCB( const geometry_msgs::WrenchStamped::ConstPtr& msg ){

    contact_force_msg.header = msg->header;

    contact_force_msg.forces.fn = fabs(msg->wrench.force.z);
    contact_force_msg.forces.ft = sqrt( pow(msg->wrench.force.x, 2) + pow(msg->wrench.force.y, 2) );
    // contact_force_msg.forces.ft = 0.1*9.8;
    contact_force_msg.forces.taun = msg->wrench.torque.z;

    pubContact.publish( contact_force_msg );

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "wrench_2_contact");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    string input_topic_str;
    nh_private.param("input_topic" , input_topic_str, string("wrench") );
    string output_topic_str;
    nh_private.param("output_topic" , output_topic_str, string("contact") );

    ros::Subscriber subWrench = nh_public.subscribe( input_topic_str, 1, wrenchCB);

    pubContact = nh_public.advertise<slipping_control_msgs::ContactForcesStamped>(output_topic_str, 1);
    
    ros::spin();

    return 0;
}