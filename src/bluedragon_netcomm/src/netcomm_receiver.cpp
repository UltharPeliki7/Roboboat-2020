#define PORT_GROUP_1 5999
#define PORT_GROUP_2 6999
#define PORT_GROUP_3 7999
#define PORT_GROUP_4 8999

#include <bluedragon_netcomm/priapus.h>
#include <bluedragon_netcomm/listener.h>

/* main 
 *
*/
int main(int argc, char **argv)
{
	/* initialize ros node 
	 *
	*/
    ros::init(argc, argv, "netcomm_receiver");
    printf("...starting listening station... \n");

	/* left propulsion publisher node
	 *
	*/
    ros::NodeHandle left_propulsion_node;
    ros::Publisher  left_propulsion_pub = left_propulsion_node.advertise
                                            <bluedragon_propulsion::propulsion>(
                                                "left_propulsion", 
                                                100);

	/* right propulsion publisher node
	 *
	*/
    ros::NodeHandle right_propulsion_node;
    ros::Publisher  right_propulsion_pub = right_propulsion_node.advertise
                                            <bluedragon_propulsion::propulsion>(
                                                "right_propulsion", 
                                                100);
     
	/* laser scan publisher node
	 *
	*/
    ros::NodeHandle laser_scan_node;
    ros::Publisher  laser_scan_pub = laser_scan_node.advertise
                                            <sensor_msgs::LaserScan>(
                                                "scan", 
                                                100);

	/* odom combined publisher node
	 *
	*/
    ros::NodeHandle odom_combined_node;
    ros::Publisher  odom_combined_pub = odom_combined_node.advertise
                                            <geometry_msgs::PoseWithCovarianceStamped>(
                                                "robot_pose_ekf/odom_combined", 
                                                100);

	/* command velocity publisher node
	 *
	*/
    ros::NodeHandle cmd_vel_node;
    ros::Publisher  cmd_vel_pub = cmd_vel_node.advertise
                                            <geometry_msgs::Twist>(
                                                "cmd_vel", 
                                                100);

	/* point cloud publisher node
	 *
	*/
    ros::NodeHandle point_cloud_node;
    ros::Publisher  point_cloud_pub = point_cloud_node.advertise
                                            <sensor_msgs::PointCloud2>(
                                                "zed/point_cloud/cloud_registered",         
                                                100);

	/* left image publisher node
	 *
	*/
    ros::NodeHandle left_image_node;
    ros::Publisher  left_image_pub = left_image_node.advertise
                                            <sensor_msgs::Image>(
                                                "zed/left/image_rect_color", 
                                                100);

	/* right image publisher node
	 *
	*/
    ros::NodeHandle right_image_node;
    ros::Publisher  right_image_pub = right_image_node.advertise
                                            <sensor_msgs::Image>(
                                                "zed/right/image_rect_color", 
                                                100);
     
	/* odometry publisher node
	 *
	*/
    ros::NodeHandle odometry_node;
    ros::Publisher  odometry_pub = odometry_node.advertise 
                                            <nav_msgs::Odometry>(
                                                "zed/odom", 
                                                100);
     
	/* ultrasound_1 publisher node
	 *
	*/
    ros::NodeHandle range_1_node;
    ros::Publisher  range_1_pub = range_1_node.advertise
                                            <sensor_msgs::Range>(
                                                "Ultrasound/PortSide", 
                                                100);

	/* ultrasound_2 publisher node
	 *
	*/
    ros::NodeHandle range_2_node;
    ros::Publisher  range_2_pub = range_2_node.advertise
                                            <sensor_msgs::Range>(
                                                "Ultrasound/Starboard", 
                                                100);
    	
	/* map publisher node
	 *
	*/
    ros::NodeHandle map_node;
    ros::Publisher  map_pub = map_node.advertise
                                            <nav_msgs::OccupancyGrid>(
                                                "map", 
                                                100);

	/* imu publisher node
	 *
	*/
    ros::NodeHandle imu_node;
    ros::Publisher  imu_pub = imu_node.advertise
                                            <sensor_msgs::Imu>(
                                                "imu_data", 
                                                100);
    	
	/* gps publisher node
	 *
	*/
    ros::NodeHandle gps_node;
    ros::Publisher  gps_pub = gps_node.advertise
                                            <sensor_msgs::NavSatFix>(
                                                "gps/fix", 
                                                100);

	/* temperature publisher node
	 *
	*/
    ros::NodeHandle temperature_node;
    ros::Publisher  temperature_pub = temperature_node.advertise
                                            <bluedragon_description::temperature>(
                                                "/Temperature/Motherboard", 
                                                100);


	/* set ros loop rate to 30 Hz
	 *
	*/
    ros::Rate loop_rate(30);

	/* which group of ports will be used
	 *
	*/
    int start_port = PORT_GROUP_1;

	/* create socket variable
	 *
	*/
    int sock;

	/* create broadcast variable
	 * set to one so that the socket can broadcast
	*/
    int broadcast = 1;

	/* default broadcast ip address
	 *
	*/
    char *ip = "10.10.10.255";

	/* message containers to be filled by the ros callback functions
	 *
	*/
    bluedragon_propulsion::propulsion           left_propulsion_msg;
    bluedragon_propulsion::propulsion           right_propulsion_msg;
    geometry_msgs::Twist                        cmd_vel_msg;
    geometry_msgs::PoseWithCovarianceStamped    odom_combined_msg;
    sensor_msgs::PointCloud2                     point_cloud_msg;
    sensor_msgs::Image                          left_image_msg;
    sensor_msgs::Image                          right_image_msg;
    sensor_msgs::LaserScan                      laser_scan_msg; 
    sensor_msgs::Range                          range_1_msg;
    sensor_msgs::Range                          range_2_msg;
    sensor_msgs::Imu                            imu_msg;
    sensor_msgs::NavSatFix                      gps_msg;
    nav_msgs::Odometry                          odometry_msg;   
    nav_msgs::OccupancyGrid                     map_msg;  
    bluedragon_description::temperature         temperature_msg;
    bluedragon_netcomm::serial_size             serial_size_msg;

	/* if user provided ip, overwrite default
	 *
	*/
    if(argc > 1)
		{
	 		ip = argv[1];
         	printf("...overwriting default ip...\n");
     	}

	/* if user provided port group, overwrite default
	 *
	*/
    if(argc > 2)
    	{
			char*	temp_arg    =  argv[2];
			int 	temp_group  =  atoi(temp_arg);

			if((temp_group > 1000) && (temp_group < 10000))
				{
	     			if((temp_group == PORT_GROUP_1) || 
					   (temp_group == PORT_GROUP_2) || 
					   (temp_group == PORT_GROUP_3) || 
					   (temp_group == PORT_GROUP_4))
	     			   {
		 			    	start_port = temp_group;
	     			   }
      
 					   printf("...overwriting default port group...\n");
     			}
        }

	/* create priapus pointer, our data receiver
	 *
	*/
    priapus* priapus_1 = new priapus(start_port, ip);


	/* main while loop
	 *
	*/
    while(ros::ok())
     	{
			/* receive message from chariot::hermes be deserialized and published
			 *
			*/
	  	 	priapus_1 -> chariot(
							&left_propulsion_msg, 
							&right_propulsion_msg, 
							&cmd_vel_msg, 
							&laser_scan_msg, 
							&range_1_msg, 
							&range_2_msg, 
							&odometry_msg, 
							&map_msg, 
							&point_cloud_msg, 
							&left_image_msg, 
							&right_image_msg, 
							&odom_combined_msg, 
							&imu_msg, 
							&gps_msg, 
                            &temperature_msg,
							&serial_size_msg);

			/* publish the network messages locally
			 *
			*/
    	    left_propulsion_pub.publish(     left_propulsion_msg);
    	    right_propulsion_pub.publish(    right_propulsion_msg);
    	    cmd_vel_pub.publish(             cmd_vel_msg);
    	    laser_scan_pub.publish(          laser_scan_msg);   
    	    range_1_pub.publish(             range_1_msg);   
    	    range_2_pub.publish(             range_2_msg);
    	    odometry_pub.publish(            odometry_msg);
	        map_pub.publish(                 map_msg);
	        point_cloud_pub.publish(         point_cloud_msg);
	        left_image_pub.publish(          left_image_msg);
	        right_image_pub.publish(         right_image_msg);
    	    odom_combined_pub.publish(       odom_combined_msg);
	        imu_pub.publish(                 imu_msg);
	        gps_pub.publish(                 gps_msg);
            temperature_pub.publish(         temperature_msg);

			/* spin ros and loop at 30 Hz 
			 *
			*/
	  	    ros::spinOnce();
  	        loop_rate.sleep();     	
     		printf("...receiving...\n");
     	}

	/* exit normally
	 *
	*/
    return 0;
} 
// END OF MAIN

