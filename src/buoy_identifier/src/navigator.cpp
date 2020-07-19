#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include "buoy_identifier/Buoy.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <stdbool.h>
int count;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 std::vector<move_base_msgs::MoveBaseGoal> goals;
void spinThread()
{
    ros::spin();
}
std::string greens, reds, color;
float greenang, redang, greendist, reddist;
float red_x, red_y, green_x, green_y;

void buoyMsg_callback(const buoy_identifier::Buoy &msg)
{
    color = msg.color;
    if (strcmp(msg.color.c_str(), "red") == 0)
    {
        reds = msg.color;
        redang = msg.angle;
        reddist = msg.distance;
        red_x = msg.rel_vect_len_x;
        red_y = msg.rel_vect_len_y;
    }
    if (strcmp(msg.color.c_str(), "green") == 0)
    {
        greens = msg.color;
        greenang = msg.angle;
        greendist = msg.distance;
        green_x = msg.rel_vect_len_x;
        green_y = msg.rel_vect_len_y;
    }
}

float distance(int x1, int y1, int x2, int y2) 
{ 
    // Calculating distance 
    return sqrt(pow(x2 - x1, 2) +  
                pow(y2 - y1, 2) * 1.0); 
} 

void modifyGoal(move_base_msgs::MoveBaseGoal* goal) {
    bool have_red = !std::isnan(redang) && !std::isnan(reddist) && !std::isnan(red_x) && !std::isnan(red_y);
    bool have_green = !std::isnan(greenang) && !std::isnan(greendist) && !std::isnan(green_x) && !std::isnan(green_y);
float target_x;
float target_y;
float target_x1;
float target_y1;
float target_x2;
float target_y2;
float xvect;
float yvect;

    if (have_red && have_green) {
        if (redang < greenang) {//if the red buoy is to the left of the green buoy
            ROS_INFO("We can move straight to the middle of the two buoys");
            // We can move straight to the middle of the two buoys

            float target_x = (green_x + red_x) / 2.0; 
            float target_y = (red_y + green_y) / 2.0;

		ROS_INFO("target_x= " , target_x);
		ROS_INFO("target_y= " , target_y);
            goal->target_pose.pose.position.x = target_y; 
            goal->target_pose.pose.position.y = -target_x;
            goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI); // TODO confirm orientation == current orientation

        }
        else {
            ROS_INFO("We must circle around the two buoys");
            // We must circle around the two buoys. First move to the right of red and rotate 180 deg,

			if(reddist>greendist) {//if the red buoy is further than the green buoy, then we want to go to the left of the green buoy. Otherwise, the right side.
//find midpoint between red and green buoy using vectors
//half that vector and subtract it from the closer point to obtain first goal. Add half the orthogonal vector to closer point to get second point. Add 1/4 orthogonal vector to midpoint to get third point, and subtract 1/4 orthogonal vector from midpoint to get final point.
xvect=(red_x-green_x)/2; //we're always going to be using the distance to the midpoint as a constant distance to hold to goals.
yvect=(red_y-green_y)/2;


if(distance(green_x-xvect, greeny+yvect, 0,0)>distance(green_x+xvect, greeny-yvect, 0,0))
{target_x=(green_x-xvect);
target_y=(green_y+yvect);
float target_x1=green_x;
float target_y1=green_y+(2*y_vect);
float target_x2=green_x+(2*x_vect);
float target_y2=green_y;
}
else{target_x=(green_x+xvect);
target_y=(green_y-yvect);
float target_x2=green_x;
float target_y2=green_y+(2*y_vect);
float target_x1=green_x+(2*x_vect);
float target_y1=green_y;
}
goal->target_pose.pose.position.x = target_y2;
goal->target_pose.pose.position.y = -target_x2;
goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
goals.push_back(goal); //Goal D put into stack
goal->target_pose.pose.position.x = target_y1;
goal->target_pose.pose.position.y = -target_x1;
goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
goals.push_back(goal); //Goal C put into stack
goal->target_pose.pose.position.x = target_y;
goal->target_pose.pose.position.y = -target_x;
goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
goals.push_back(goal); //Goal B put into stack
target_x=(green_x-xvect); 
target_y=(green_y-yvect);
goal->target_pose.pose.position.x = target_y;
goal->target_pose.pose.position.y = -target_x;
goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
goals.push_back(goal); //Goal A put into stack
}

          
		ROS_INFO("target_x= " , target_x);
		ROS_INFO("target_y= " , target_y);
            goal->target_pose.pose.position.x = target_y;
            goal->target_pose.pose.position.y = -target_x;
            goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 2.0);
            // ^ TODO confirm orientation points towards the buoys

        }
    }
    else if (have_red) {
        ROS_INFO("Rotate until we see both");
        goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.5);
    }
    else if (have_green) {
        ROS_INFO("Rotate until we see both");
        goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.5);
    }
    else {
        ROS_INFO("Rotate until we see both");
        goal->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.5);
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_goal");

    ros::NodeHandle n;

    boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

    MoveBaseClient ac("move_base", true);

    ros::Subscriber redBuoySub = n.subscribe("/redbuoy_publisher", 1000, buoyMsg_callback);
    std::cout << "created subscriber for redbuoy" << std::endl;
    ros::Subscriber greenBuoySub = n.subscribe("/greenbuoy_publisher", 1000, buoyMsg_callback);
    std::cout << "created publisher for greenbuoy" << std::endl;
    ros::Rate loop_rate(5); //10 messages per second



    //give some time for connections to register
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    while(ros::ok())
    {   
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
if(goals.empty()){
        modifyGoal(&goal);}
else{goal=goals.back();
goals.pop_back();
}
        // if(reddist > 1.0 && greendist > 1.0)
        // {
        //     //we'll send a goal to the robot to move 2 meters forward
        //     goal.target_pose.header.frame_id = "/map";
        //     goal.target_pose.header.stamp = ros::Time::now();
        //     if(reddist > greendist)
        //     {
        //         ROS_INFO("Sending goal to move forward and towards the left");
        //         goal.target_pose.pose.position.x = (greendist / 2) + 0.5;
        //         goal.target_pose.pose.position.y = -0.2;
        //         goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
        //     }
        //     if(greendist > reddist)
        //     {
        //         ROS_INFO("Sending goal to move forward and towards the right");
        //         goal.target_pose.pose.position.x = (reddist / 2) + 0.5;
        //         goal.target_pose.pose.position.y = 0.2;
        //         goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
        //     }
       ROS_INFO("Sending goal");
       ac.sendGoal(goal);

       ac.waitForResult();

       if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
           ROS_INFO("Hooray, the base moved somewhere");
       }
       else {
           ROS_INFO("The base failed to move forward for some reason");
       }

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;

}
