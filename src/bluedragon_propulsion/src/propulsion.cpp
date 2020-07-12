#define ROBOT_WIDTH 0.6

#define MAX_PWM 1850
#define MIN_PWM 1300

#define MAX_VEL 1.6
#define MIN_VEL 0.5

#include "ros/ros.h"
#include <bluedragon_propulsion/propulsion.h>
#include <bluedragon_propulsion/listener.h>
#include <thread>


int translate_to_PWM(double speed)
{
    int temp_PWM=1500;
    if(speed == 0)
    {
        return temp_PWM;
    }
    else if(speed > 0)
    {
        temp_PWM = (MAX_PWM/MAX_VEL) * (speed);
        return temp_PWM;
    }
    else
    {
        temp_PWM = (MIN_PWM/MIN_VEL) * (speed) * (-1);
        return temp_PWM;
    }
}

void prop_commander(bluedragon_propulsion::propulsion* propulsion_msg, int64_t * throttle)
{
    propulsion_msg->header.stamp=ros::Time::now();
    propulsion_msg->throttle=*throttle;
}


int main(int argc, char **argv)
{
// initialize ros
     ros::init(argc, argv, "propulsion");
     
     // create propulsion node to produce propulsion message
     // the message is received by an arduino mega which controls
     // two ESC motor controllers connected to two rear thrusters
     ros::NodeHandle left_propulsion_node;
     ros::Publisher left_propulsion_pub = left_propulsion_node.advertise<bluedragon_propulsion::propulsion>("left_propulsion", 100);

     ros::NodeHandle right_propulsion_node;
     ros::Publisher right_propulsion_pub = right_propulsion_node.advertise<bluedragon_propulsion::propulsion>("right_propulsion", 100);
     
     // create cmd_vel subscriber node to listen for the navigation stack velocity suggestion
     cmd_vel_listener cv_listener;
     ros::NodeHandle cmd_vel_node;
     ros::Subscriber cmd_vel_sub = cmd_vel_node.subscribe("cmd_vel", 100, &cmd_vel_listener::cmd_vel_callback, &cv_listener);

     // main variables
     int64_t left_throttle=1503;
     int64_t right_throttle=1503;
     double left_throttle_multiplier=0.0;
     double right_throttle_multiplier=0.0;
     bool automatik = false;
     geometry_msgs::Twist cmd_vel;
     bluedragon_propulsion::propulsion left_propulsion_msg;
     bluedragon_propulsion::propulsion right_propulsion_msg;
     left_propulsion_msg.header.frame_id="left_thruster_link";
     right_propulsion_msg.header.frame_id="right_thruster_link";

     // user input motor overrides both motors to the same value
     if(argc == 2)
     {
	       std::string temp_command = argv[1];
	       if(temp_command == "automatik")
	       {
	           automatik=true;
	       }   

	       int temp_propulsion = atoi(argv[1]);
	    
           if((temp_propulsion > 1100) || (temp_propulsion < 1900))
	       {
	           left_throttle=temp_propulsion;
	           right_throttle=temp_propulsion;
	       }
      }

     // user input motor overrides for each motor seperately
     if(argc > 2)
     {
	    int temp_left_propulsion = atoi(argv[1]);
	    int temp_right_propulsion = atoi(argv[2]);
	    if((temp_left_propulsion > 1100) || (temp_left_propulsion < 1900))
	    {
	        if((temp_right_propulsion > 1100) || (temp_right_propulsion < 1900))
	        {
	      	    left_throttle=temp_left_propulsion;
	            right_throttle=temp_right_propulsion;
	        }
	    }
     }
     
     // ret ros loop rate
     ros::Rate loop_rate(100);

     // main while loop
     while(ros::ok())
     {	
	     //CONVERT geometry_msgs::Twist into PWM values 
         //
	     if(automatik == true)
	     {   
             cmd_vel = cv_listener.get_cmd_vel_msg();

             // Send cmd_vel to PID controller ?
             /*
             */
        
             left_throttle_multiplier  = cmd_vel.linear.x - cmd_vel.angular.z*ROBOT_WIDTH/2;
             right_throttle_multiplier = cmd_vel.linear.x + cmd_vel.angular.z*ROBOT_WIDTH/2;
	      
	         left_throttle=translate_to_PWM(left_throttle_multiplier);
	         right_throttle=translate_to_PWM(right_throttle_multiplier);
         }

  	     // fill propulsion message
         std::thread thread_1(&prop_commander, &left_propulsion_msg, &left_throttle);
         std::thread thread_2(&prop_commander, &right_propulsion_msg, &right_throttle);
         thread_1.join();
         thread_2.join();
  	    
         left_propulsion_pub.publish(left_propulsion_msg);
         right_propulsion_pub.publish(right_propulsion_msg);

	     // loop ros
	     ros::spinOnce();
         loop_rate.sleep();
      }
      // exit normally
      return 0;
}
