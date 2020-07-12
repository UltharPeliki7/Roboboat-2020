#define PORT_GROUP_1 5999
#define PORT_GROUP_2 6999
#define PORT_GROUP_3 7999
#define PORT_GROUP_4 8999

#include <bluedragon_netcomm/hermes.h>
#include <bluedragon_netcomm/listener.h>

/* main 
 *
*/
int main(int argc, char **argv)
{
	/* initialize ros node 
	 *
	*/
    ros::init(argc, argv, "netcomm");
    printf("\nstarting netcomm broadcast station\n");

	/* create left propulsion subscriber node
	 *
	*/
    propulsion_listener pr_listener;
    ros::NodeHandle left_propulsion_node;
    ros::Subscriber left_propulsion_sub = left_propulsion_node.subscribe(
										"left_propulsion", 
										100, 
										&propulsion_listener::propulsion_callback, 
										&pr_listener);

	/* create right propulsion subscriber node
	 *
	*/
    propulsion_listener pr2_listener;
    ros::NodeHandle right_propulsion_node;
    ros::Subscriber right_propulsion_sub = right_propulsion_node.subscribe(
										"right_propulsion", 
										100, 
										&propulsion_listener::propulsion_callback, 
										&pr2_listener);
     
	/* create command velocity subscriber node
	 *
	*/
    cmd_vel_listener cv_listener;
    ros::NodeHandle cmd_vel_node;
    ros::Subscriber cmd_vel_sub = cmd_vel_node.subscribe(
										"cmd_vel", 
										100, 
										&cmd_vel_listener::cmd_vel_callback, 
										&cv_listener);
     
	/* create laser scan subscriber node
	 *
	*/
    laser_scan_listener ls_listener;
    ros::NodeHandle laser_scan_node;
    ros::Subscriber laser_scan_sub = laser_scan_node.subscribe(
										"scan", 
										100, 
										&laser_scan_listener::laser_scan_callback, 
										&ls_listener);
     
	/* create odometry subscriber node
	 *
	*/
    odometry_listener od_listener;
    ros::NodeHandle odometry_node;
    ros::Subscriber odometry_sub = odometry_node.subscribe(
										"zed/odom", 
										100, 
										&odometry_listener::odometry_callback, 
										&od_listener);

	/* create odom combined subscriber node
	 *
	*/
    odom_combined_listener oc_listener;
    ros::NodeHandle odom_combined_node;
    ros::Subscriber odom_combined_sub = odom_combined_node.subscribe(
										"robot_pose_ekf/odom_combined", 
										100, 
										&odom_combined_listener::odom_combined_callback, 
										&oc_listener);
     
	/* create ultrasound_1 subscriber node
	 *
	*/
    range_listener ra_listener_1;
    ros::NodeHandle range_node_1;
    ros::Subscriber range_sub_1 = range_node_1.subscribe(
										"Ultrasound/PortSide", 
										100, 
										&range_listener::range_callback, 
										&ra_listener_1);

	/* create ultrasound_2 subscriber node
	 *
	*/
    range_listener ra_listener_2;
    ros::NodeHandle range_node_2;
    ros::Subscriber range_sub_2 = range_node_2.subscribe(
										"Ultrasound/Starboard", 
										100, 
										&range_listener::range_callback, 
										&ra_listener_2);

	/* create map subscriber node
	 *
	*/
    map_listener ma_listener;
    ros::NodeHandle map_node;
    ros::Subscriber map_sub = map_node.subscribe(
										"map", 
										100, 
										&map_listener::map_callback, 
										&ma_listener);

	/* create imu subscriber node
	 *
	*/
    imu_listener im_listener;
    ros::NodeHandle imu_node;
    ros::Subscriber imu_sub = imu_node.subscribe(
										"imu_data", 
										100, 
										&imu_listener::imu_callback, 
									    &im_listener);

	/* create gps subscriber node
	 *
	*/
    gps_listener gp_listener;
    ros::NodeHandle gps_node;
    ros::Subscriber gps_sub = gps_node.subscribe(
										"gps/fix",
										100, 
										&gps_listener::gps_callback, 
										&gp_listener);

	/* create point cloud subscriber node
	 *
	*/
    point_cloud_listener pc_listener;
    ros::NodeHandle point_cloud_node;
    ros::Subscriber point_cloud_sub = point_cloud_node.subscribe(
										"zed/point_cloud/cloud_registered", 
										100, 
										&point_cloud_listener::point_cloud_callback, 
										&pc_listener);

	/* create left image subscriber node
	 *
	*/
    image_listener li_listener;
    ros::NodeHandle left_image_node;
    ros::Subscriber left_image_sub = left_image_node.subscribe(
										"zed/left/image_rect_color", 
										100, 
										&image_listener::image_callback, 
										&li_listener);

	/* create right image subscriber node
	 *
	*/
    image_listener ri_listener;
    ros::NodeHandle right_image_node;
    ros::Subscriber right_image_sub = right_image_node.subscribe(
										"zed/right/image_rect_color", 
										100, 
										&image_listener::image_callback, 
										&ri_listener);

	/* create temperature subscriber node
	 *
	*/
    temperature_listener tp_listener;
    ros::NodeHandle temperature_node;
    ros::Subscriber temperature_sub = temperature_node.subscribe(
										"Temperature/Motherboard", 
										100, 
										&temperature_listener::temperature_callback, 
										&tp_listener);


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
    bluedragon_description::temperature         temperature_msg;
    bluedragon_propulsion::propulsion           left_propulsion_msg;
    bluedragon_propulsion::propulsion           right_propulsion_msg;
    geometry_msgs::Twist                        cmd_vel_msg;
    geometry_msgs::PoseWithCovarianceStamped    odom_combined_msg;
    sensor_msgs::PointCloud2                    point_cloud_msg;
    sensor_msgs::Image                          left_image_msg;
    sensor_msgs::Image                          right_image_msg;
    sensor_msgs::LaserScan                      laser_scan_msg; 
    sensor_msgs::Range                          range_1_msg;
    sensor_msgs::Range                          range_2_msg;
    sensor_msgs::Imu                            imu_msg;
    sensor_msgs::NavSatFix                      gps_msg;
    nav_msgs::Odometry                          odometry_msg;   
    nav_msgs::OccupancyGrid                     map_msg;  
    bluedragon_netcomm::serial_size             serial_size_msg;



	/* if user provided ip, overwrite default
	 *
	*/
    if(argc > 1)
		{
	 		ip = argv[1];
         	printf("overwriting default ip\n");
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
      
 					   printf("overwriting default port group\n");
     			}
        }
     
	/* error check the socket while initializing it
	 *
	*/
    if( (sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    	{
         	perror("socket : ");
         	return -1;
     	}
    if( setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) != 0 )
    	{
        	perror("setsockopt : ");
       		close(sock);
       		return -1;
     	}

	/* create chariot pointer, our data messenger
	 *
	*/
    hermes* hermes_1 = new hermes(start_port, ip);


	/* main while loop
	 *
	*/
    while(ros::ok())
     	{
     		printf("...broadcasting...\n");

			/* fill messages with callback to subscribers
			 *
			*/
	  		left_propulsion_msg		=	pr_listener.get_propulsion_msg();
	  		right_propulsion_msg	=	pr2_listener.get_propulsion_msg();
	  		cmd_vel_msg           	=	cv_listener.get_cmd_vel_msg();
	 		laser_scan_msg        	=	ls_listener.get_laser_scan_msg();
	  		range_1_msg			  	=	ra_listener_1.get_range_msg();
	 		range_2_msg				=	ra_listener_2.get_range_msg();
	 		odometry_msg			=	od_listener.get_odometry_msg();
	  		map_msg					=	ma_listener.get_map_msg();
	  		point_cloud_msg			=	pc_listener.get_point_cloud_msg();
	  		left_image_msg			=	li_listener.get_image_msg();
	  		right_image_msg			=	ri_listener.get_image_msg();
	  		odom_combined_msg		=	oc_listener.get_odom_combined_msg();
	  		imu_msg					=	im_listener.get_imu_msg();
	  		gps_msg					=	gp_listener.get_gps_msg();
            temperature_msg         =   tp_listener.get_temperature_msg();

			/* send messages to chariot::hermes to be serialized and sent off
			 *
			*/
	  	 	hermes_1 -> chariot(
							&sock, 
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

			/* spin ros (call callbacks) and reduce loop rate to 30 Hz ]\
			 *
			*/
	  	    ros::spinOnce();
  	        loop_rate.sleep();
     	}

	/* exit normally
	 *
	*/
    return 0;
}

