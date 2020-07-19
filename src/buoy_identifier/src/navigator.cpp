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
#include <visualization_msgs/Marker.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES
bool test=true;//used to set test condition
bool left=true; //when left is true, test condition is green buoy on the left, red on the right, which flip flops which buoy gets assigned left_test_x and left_test_y etc.
float left_test_x=2.5;
float right_test_x=4.5;
float left_test_y=2;
float right_test_y=-0.5;
float leftdist=2.25;
float rightdist=4.25;
//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES



// Global memory
std::vector<move_base_msgs::MoveBaseGoal*> goals;
int count;
std::string greens, reds, color;
float greenang, redang, greendist, reddist;
float red_x, red_y, green_x, green_y;

void spinThread()
{
    ros::spin();
}

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
if(test) //FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES//FOR TESTING PURPOSES
{
	reds="red";
	greens="green";
	
	if(left)
		{
		redang=20;
		greenang=-20;
		green_x=left_test_x;
		red_x=right_test_x;
		green_y=left_test_y;
		red_y=right_test_y;
		reddist=leftdist;
		greendist=rightdist;
		}
	else
		{
		redang=-20;
		greenang=20;
		green_x=right_test_x;
		red_x=left_test_x;
		green_y=right_test_y;
		red_y=left_test_y;
		reddist=rightdist;
		greendist=leftdist;
		}
}
}
float distance(int x1, int y1, int x2, int y2)
{
    // Calculating distance between two points
    return sqrt(pow(x2 - x1, 2) +
                pow(y2 - y1, 2) * 1.0);
}

void createGoals(
    move_base_msgs::MoveBaseGoal *goalA,
    move_base_msgs::MoveBaseGoal *goalB,
    move_base_msgs::MoveBaseGoal *goalC,
    move_base_msgs::MoveBaseGoal *goalD
)
{
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

    if (have_red && have_green)
    {
        if (redang < greenang)  //if the red buoy is to the left of the green buoy
        {
            ROS_INFO("We can move straight to the middle of the two buoys");
            // We can move straight to the middle of the two buoys

            float target_x = (green_x + red_x) / 2.0;
            float target_y = (red_y + green_y) / 2.0;

            ROS_INFO("target_x= ", target_x);
            ROS_INFO("target_y= ", target_y);
            goalA->target_pose.pose.position.x = target_y;
            goalA->target_pose.pose.position.y = -target_x;
            goalA->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI); // TODO confirm orientation == current orientation

            // Push a pointer to goalA to the queue,
            // which will be executed next time the loop runs
            goals.push_back(goalA);

        }
        else
        {
            ROS_INFO("We must circle around the two buoys");
            // We must circle around the two buoys. First move to the right of red and rotate 180 deg,

            if(reddist > greendist) //we set 4 goals going around the left side of the green buoy and through the buoys. A is to the left, B is above, C is in line but behind the midpoint, D is through the midpoint.
            {ROS_INFO("Calculating path around green buoy");
                //if the red buoy is further than the green buoy, then we want to go to the left of the green buoy. Otherwise, the right side.
                //find midpoint between red and green buoy using vectors
                //half that vector and subtract it from the closer point to obtain first goal. Add half the orthogonal vector to closer point to get second point. Add 1/4 orthogonal vector to midpoint to get third point, and subtract 1/4 orthogonal vector from midpoint to get final point.
                xvect = (red_x - green_x) / 2; //we're always going to be using the distance to the midpoint as a constant distance to hold to goals.
                yvect = (red_y - green_y) / 2;


                if(distance(green_x - xvect, green_y + yvect, 0, 0) > distance(green_x + xvect, green_y - yvect, 0, 0))
                {
                    target_x = (green_x - xvect);
                    target_y = (green_y + yvect);
                    target_x1 = green_x;
                    target_y1 = green_y + (2 * yvect);
                    target_x2 = green_x + (2 * xvect);
                    target_y2 = green_y;
                }
                else
                {
                    target_x = (green_x + xvect);
                    target_y = (green_y - yvect);
                    target_x2 = green_x;
                    target_y2 = green_y + (2 * yvect);
                    target_x1 = green_x + (2 * xvect);
                    target_y1 = green_y;
                }

                goalD->target_pose.pose.position.x = target_y2;
                goalD->target_pose.pose.position.y = -target_x2;
                goalD->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalD); //Goal D put into stack
                ROS_INFO("Initialized goal D");
			ROS_INFO("X distance : %g m", target_y2);
			ROS_INFO("Y distance : %g m", -target_x2);
                goalC->target_pose.pose.position.x = target_y1;
                goalC->target_pose.pose.position.y = -target_x1;
                goalC->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalC); //Goal C put into stack
                ROS_INFO("Initialized goal C");
			ROS_INFO("X distance : %g m", target_y1);
			ROS_INFO("Y distance : %g m", -target_x1);
                goalB->target_pose.pose.position.x = target_y;
                goalB->target_pose.pose.position.y = -target_x;
                goalB->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalB); //Goal B put into stack
                ROS_INFO("Initialized goal B");
			ROS_INFO("X distance : %g m", target_y);
			ROS_INFO("Y distance : %g m", -target_x);
                target_x = (green_x - xvect);
                target_y = (green_y - yvect);
                goalA->target_pose.pose.position.x = target_y;
                goalA->target_pose.pose.position.y = -target_x;
                goalA->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalA); //Goal A put into stack
                ROS_INFO("Initialized goal A");
			ROS_INFO("X distance : %g m", target_y);
			ROS_INFO("Y distance : %g m", -target_x);
            }
            else //if(reddist<greendist) we set 4 goals going around the right side of the red buoy and through the buoys. E is to the right, F is above, C is in line but behind the midpoint, D is through the midpoint.
            {ROS_INFO("Calculating path around red buoy");
                xvect = (-red_x + green_x) / 2; //we're always going to be using the distance to the midpoint as a constant distance to hold to goals.
                yvect = (-red_y + green_y) / 2;
                if(distance(red_x - xvect, red_y + yvect, 0, 0) > distance(red_x + xvect, red_y - yvect, 0, 0))
                {
                    target_x = (red_x - xvect);
                    target_y = (red_y + yvect);
                    target_x1 = red_x;
                    target_y1 = red_y + (2 * yvect);
                    target_x2 = red_x + (2 * xvect);
                    target_y2 = red_y;
                }
                else
                {
                    target_x = (red_x + xvect);
                    target_y = (red_y - yvect);
                    target_x2 = red_x;
                    target_y2 = red_y + (2 * yvect);
                    target_x1 = red_x + (2 * xvect);
                    target_y1 = red_y;
                }

                goalD->target_pose.pose.position.x = target_y2;
                goalD->target_pose.pose.position.y = -target_x2;
                goalD->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalD); //Goal D put into stack
                ROS_INFO("Initialized goal D");
			ROS_INFO("X distance : %g m", target_y2);
			ROS_INFO("Y distance : %g m", -target_x2);
                goalC->target_pose.pose.position.x = target_y1;
                goalC->target_pose.pose.position.y = -target_x1;
                goalC->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalC); //Goal C put into stack
                ROS_INFO("Initialized goal C");
			ROS_INFO("X distance : %g m", target_y1);
			ROS_INFO("Y distance : %g m", -target_x1);
                goalB->target_pose.pose.position.x = target_y;
                goalB->target_pose.pose.position.y = -target_x;
                goalB->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalB); //Goal F put into stack
                ROS_INFO("Initialized goal F");
			ROS_INFO("X distance : %g m", target_y);
			ROS_INFO("Y distance : %g m", -target_x);
                target_x = (red_x - xvect);
                target_y = (red_y - yvect);
                goalA->target_pose.pose.position.x = target_y;
                goalA->target_pose.pose.position.y = -target_x;
                goalA->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
                goals.push_back(goalA); //Goal E put into stack
                ROS_INFO("Initialized goal E");
			ROS_INFO("X distance : %g m", target_y);
			ROS_INFO("Y distance : %g m", -target_x);
            }

        }
    }
    else if (have_red)
    {
        ROS_INFO("Rotate until we see both, we see only red");
        goalA->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.5);
        goals.push_back(goalA);
    }
    else if (have_green)
    {
        ROS_INFO("Rotate until we see both, we see only green");
        goalA->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.5);
        goals.push_back(goalA);
    }
    else
    {
        ROS_INFO("Rotate until we see both, we see no buoys");
        goalA->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.5);
        goals.push_back(goalA);
    }

}

int main(int argc, char **argv)
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

	ros::Publisher green_pub = n.advertise<visualization_msgs::Marker>( "green_buoy", 0 ); //we want to mark the positions of the buoys
	ros::Publisher red_pub = n.advertise<visualization_msgs::Marker>( "red_buoy", 0 );









    //give some time for connections to register
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // This memory lives on the stack, but we can use it b/c it lives for the entire time while (ros::ok()) runs.
    move_base_msgs::MoveBaseGoal goalA;
    move_base_msgs::MoveBaseGoal goalB;
    move_base_msgs::MoveBaseGoal goalC;
    move_base_msgs::MoveBaseGoal goalD;

    move_base_msgs::MoveBaseGoal* current_goal = NULL;



visualization_msgs::Marker greenmarker;
visualization_msgs::Marker redmarker;
    while(ros::ok())
    {


greenmarker.header.frame_id = "base_link";
greenmarker.header.stamp = ros::Time();
greenmarker.ns = "my_namespace";
greenmarker.id = 0;
greenmarker.type = visualization_msgs::Marker::SPHERE;
greenmarker.action = visualization_msgs::Marker::ADD;
greenmarker.pose.position.x = green_x;
greenmarker.pose.position.y = green_y;
greenmarker.pose.position.z = 0;
greenmarker.pose.orientation.x = 0.0;
greenmarker.pose.orientation.y = 0.0;
greenmarker.pose.orientation.z = 0.0;
greenmarker.pose.orientation.w = 1.0;
greenmarker.scale.x = 0.3;
greenmarker.scale.y = 0.3;
greenmarker.scale.z = 0.3;
greenmarker.color.a = 1.0; // Don't forget to set the alpha!
greenmarker.color.r = 0.0;
greenmarker.color.g = 1.0;
greenmarker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
green_pub.publish( greenmarker );

redmarker.header.frame_id = "base_link";
redmarker.header.stamp = ros::Time();
redmarker.ns = "my_namespace";
redmarker.id = 1;
redmarker.type = visualization_msgs::Marker::SPHERE;
redmarker.action = visualization_msgs::Marker::ADD;
redmarker.pose.position.x = red_x;
redmarker.pose.position.y = red_y;
redmarker.pose.position.z = 0;
redmarker.pose.orientation.x = 0.0;
redmarker.pose.orientation.y = 0.0;
redmarker.pose.orientation.z = 0.0;
redmarker.pose.orientation.w = 1.0;
redmarker.scale.x = 0.3;
redmarker.scale.y = 0.3;
redmarker.scale.z = 0.3;
redmarker.color.a = 1.0; // Don't forget to set the alpha!
redmarker.color.r = 1.0;
redmarker.color.g = 0.0;
redmarker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
red_pub.publish( redmarker );









        goalA.target_pose.header.frame_id = "/map";
        goalA.target_pose.header.stamp = ros::Time::now();
        goalB.target_pose.header.frame_id = "/map";
        goalB.target_pose.header.stamp = ros::Time::now();
        goalC.target_pose.header.frame_id = "/map";
        goalC.target_pose.header.stamp = ros::Time::now();
        goalD.target_pose.header.frame_id = "/map";
        goalD.target_pose.header.stamp = ros::Time::now();

        if(goals.empty())
        {
            // createGoals is responsible for putting as many of these as it feels
            // is necessary on the queue goals.
            createGoals(&goalA, &goalB, &goalC, &goalD);
        }
        else
        {
            current_goal = goals.back();
            goals.pop_back();
        }

        if (current_goal != NULL) {
            ROS_INFO("Sending goal");
            ac.sendGoal(*current_goal);
            ac.waitForResult();

            // It only ever makes sense to check state after we assign current_goal to something
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Hooray, the base moved somewhere");
            }
            else
            {
                ROS_INFO("The base failed to move for some reason");
            }

        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;

}
