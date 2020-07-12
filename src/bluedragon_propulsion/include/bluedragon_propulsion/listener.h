#ifndef LISTENER_H
#define LISTENER_H

#include <geometry_msgs/Twist.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// command velocity listener 
class cmd_vel_listener
{
	private:
	    geometry_msgs::Twist temp_cmd_vel_msg;
		
	public:
	    void cmd_vel_callback(const geometry_msgs::Twist& msg)
	    {
	        temp_cmd_vel_msg=msg;
	    }
	    geometry_msgs::Twist get_cmd_vel_msg(){return temp_cmd_vel_msg;}	
};

#endif // LISTENER_H
