#ifndef PRIAPUS_H
#define PRIAPUS_H

#include "ros/ros.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/unistd.h>
#include <sys/fcntl.h>
#include <cstdlib>
#include <bluedragon_propulsion/propulsion.h>
#include <bluedragon_description/temperature.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <bluedragon_netcomm/serial_size.h>

/* priapus class declaration
 *
*/
class priapus
{
    /* Priapus private member variables
     *
    */
	private:
        /* serial size socket 
         *
        */
        int ss_socket;
		struct sockaddr_in    ss_si;

        /* propulsion socket 
         *
        */
        int pr_socket;
	    struct sockaddr_in    pr_si;

        /* propulsion socket 
         *
        */
        int pr2_socket;
	    struct sockaddr_in    pr2_si;

        /* command velocity socket 
         *
        */
        int cv_socket;
	    struct sockaddr_in    cv_si;

        /* laser scan socket
         *
        */
        int ls_socket;
	    struct sockaddr_in    ls_si;

        /* ultrasound_1 socket 
         *
        */
        int ra1_socket;
	    struct sockaddr_in    ra1_si;

        /* ultrasound_2  socket 
         *
        */
        int ra2_socket;
	    struct sockaddr_in    ra2_si;

        /* odometry socket 
         *
        */
        int od_socket;
	    struct sockaddr_in    od_si;

        /* odom combined socket 
         *
        */
        int oc_socket;
	    struct sockaddr_in    oc_si;

        /* map socket
         *
        */
        int ma_socket;
	    struct sockaddr_in    ma_si;

        /* point cloud socket 
         *
        */
        int pc_socket;
	    struct sockaddr_in    pc_si;

        /* left image socket 
         *
        */
        int li_socket;
	    struct sockaddr_in    li_si;

        /* right image socket 
         *
        */
        int ri_socket;
	    struct sockaddr_in    ri_si;

        /* imu socket 
         *
        */
        int im_socket;
	    struct sockaddr_in    im_si;

        /* gps  socket 
         *
        */
        int gp_socket;
	    struct sockaddr_in 	  gp_si;

        /* temperature  socket 
         *
        */
        int tp_socket;
	    struct sockaddr_in 	  tp_si;

 
    /* chariot public functions
     *
    */
	public:
	    /* Default Constructor declaration
	     *
		*/
	    priapus();

	    /* Initializing Constructor declaration
	     *
		*/
	    priapus(
			int      port_group, 
			char*    ip);

	    /* Deconstructor 
	     *
		*/
	    ~priapus(){}

	    /* set port data function declaration
	     *
		*/
	    int set_port_data(
			int      port_group, 
			char*    ip);

	    /* hermes function declaration
	     *
		*/
	    void chariot(
		    bluedragon_propulsion::propulsion*          left_propulsion_msg,
		    bluedragon_propulsion::propulsion*          right_propulsion_msg,  
		    geometry_msgs::Twist*                       cmd_vel_msg, 
		    sensor_msgs::LaserScan*                     laser_scan_msg, 
		    sensor_msgs::Range*                         range_1_msg, 
		    sensor_msgs::Range*                         range_2_msg, 
		    nav_msgs::Odometry*                         odometry_msg, 
		    nav_msgs::OccupancyGrid*                    map_msg, 
		    sensor_msgs::PointCloud2*                   point_cloud_msg, 
		    sensor_msgs::Image*                         left_image_msg, 
		    sensor_msgs::Image*                         right_image_msg, 
		    geometry_msgs::PoseWithCovarianceStamped*   odom_combined_msg, 
		    sensor_msgs::Imu*                           imu_msg, 
		    sensor_msgs::NavSatFix*                     gps_msg, 
            bluedragon_description::temperature*        temperature_msg,
		    bluedragon_netcomm::serial_size*            serial_size_msg);
};

/* default constructor definition
 *
*/
priapus::priapus()
	{
		set_port_data(0, 0);
	}

/* initializing constructor definition
 *
*/
priapus::priapus(
	int      port_group, 
	char*    ip)
	{
	    /* call set port data functino
	     *
		*/
		set_port_data(port_group, ip);
	}

/* set port data definition
 *
*/
int priapus::set_port_data(
	int      start_port, 
	char*    ip)
    {	
        bool error_flag = false;

    ss_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    ss_si.sin_addr.s_addr =  INADDR_ANY;
    ss_si.sin_port = htons( start_port );
    ss_si.sin_family = AF_INET;
    
    if (bind(ss_socket, (struct sockaddr *)&ss_si, sizeof(ss_si)) < 0) 
    {
       perror("Connection error");

    }
      
  ///////////////////////////////////////////////////////
  // left propulsion socket

    //create socket
    pr_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    pr_si.sin_addr.s_addr =  INADDR_ANY;
    pr_si.sin_port = htons( start_port + 1 );
    pr_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(pr_socket, (struct sockaddr *)&pr_si, sizeof(pr_si)) < 0) 
    {
       perror("Connection error");
    }

  ///////////////////////////////////////////////////////
  // right propulsion socket

	
    //create socket
    pr2_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    pr2_si.sin_addr.s_addr =  INADDR_ANY;
    pr2_si.sin_port = htons( start_port + 4);
    pr2_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(pr2_socket, (struct sockaddr *)&pr2_si, sizeof(pr2_si)) < 0) 
    {
       perror("Connection error");
    }
    
  ///////////////////////////////////////////////////////
  // command velocity socket

    //create socket
    cv_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    cv_si.sin_addr.s_addr =  INADDR_ANY;
    cv_si.sin_port = htons( start_port + 2 );
    cv_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(cv_socket, (struct sockaddr *)&cv_si, sizeof(cv_si)) < 0) 
    {
       perror("Connection error");
    }

  ///////////////////////////////////////////////////////
  // laser scan socket
    //create socket
    ls_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    ls_si.sin_addr.s_addr =  INADDR_ANY;
    ls_si.sin_port = htons( start_port + 3 );
    ls_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(ls_socket, (struct sockaddr *)&ls_si, sizeof(ls_si)) < 0) 
    {
       perror("Connection error");
    }
    
  ///////////////////////////////////////////////////////
  // range socket
  
    //create socket
    ra1_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    ra1_si.sin_addr.s_addr =  INADDR_ANY;
    ra1_si.sin_port = htons( start_port + 5 );
    ra1_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(ra1_socket, (struct sockaddr *)&ra1_si, sizeof(ra1_si)) < 0) 
    {
       perror("Connection error");
    }

  ///////////////////////////////////////////////////////
  // odometry socket
	
    //create socket
    od_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    od_si.sin_addr.s_addr =  INADDR_ANY;
    od_si.sin_port = htons( start_port + 6 );
    od_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(od_socket, (struct sockaddr *)&od_si, sizeof(od_si)) < 0) 
    {
       perror("Connection error");

    }
    
  ///////////////////////////////////////////////////////
  // map socket
  
    //create socket
    ma_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    ma_si.sin_addr.s_addr =  INADDR_ANY;
    ma_si.sin_port = htons( start_port + 7 );
    ma_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(ma_socket, (struct sockaddr *)&ma_si, sizeof(ma_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // range 2 socket
  
	
    //create socket
    ra2_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    ra2_si.sin_addr.s_addr =  INADDR_ANY;
    ra2_si.sin_port = htons( start_port + 8 );
    ra2_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(ra2_socket, (struct sockaddr *)&ra2_si, sizeof(ra2_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // point cloud socket

	
    //create socket
    pc_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    pc_si.sin_addr.s_addr =  INADDR_ANY;
    pc_si.sin_port = htons( start_port + 9 );
    pc_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(pc_socket, (struct sockaddr *)&pc_si, sizeof(pc_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // left image socket

    //create socket
    li_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    li_si.sin_addr.s_addr =  INADDR_ANY;
    li_si.sin_port = htons( start_port + 10 );
    li_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(li_socket, (struct sockaddr *)&li_si, sizeof(li_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // rigth image socket
  
    //create socket
    ri_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    ri_si.sin_addr.s_addr =  INADDR_ANY;
    ri_si.sin_port = htons( start_port + 11 );
    ri_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(ri_socket, (struct sockaddr *)&ri_si, sizeof(ri_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // odom combined socket
  
    //create socket
    oc_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    oc_si.sin_addr.s_addr =  INADDR_ANY;
    oc_si.sin_port = htons( start_port + 12 );
    oc_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(oc_socket, (struct sockaddr *)&oc_si, sizeof(oc_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // imu data socket

    //create socket
    im_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    im_si.sin_addr.s_addr =  INADDR_ANY;
    im_si.sin_port = htons( start_port + 13 );
    im_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(im_socket, (struct sockaddr *)&im_si, sizeof(im_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // gps data socket
	
    //create socket
    gp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    gp_si.sin_addr.s_addr =  INADDR_ANY;
    gp_si.sin_port = htons( start_port + 14 );
    gp_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(gp_socket, (struct sockaddr *)&gp_si, sizeof(gp_si)) < 0) 
    {
       perror("Connection error");

    }

  ///////////////////////////////////////////////////////
  // temperature socket
	
    //create socket
    tp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    //assign socket values
    tp_si.sin_addr.s_addr =  INADDR_ANY;
    tp_si.sin_port = htons( start_port + 15 );
    tp_si.sin_family = AF_INET;
    
    //checks connection
    if (bind(tp_socket, (struct sockaddr *)&tp_si, sizeof(tp_si)) < 0) 
    {
       perror("Connection error");

    }

    }


/* hermes definition
 *
*/
void priapus::chariot(
	bluedragon_propulsion::propulsion*          left_propulsion_msg, 
	bluedragon_propulsion::propulsion*          right_propulsion_msg, 
	geometry_msgs::Twist* 				        cmd_vel_msg, 
	sensor_msgs::LaserScan* 			        laser_scan_msg, 
	sensor_msgs::Range* 			  	        range_1_msg, 
	sensor_msgs::Range* 				        range_2_msg, 
	nav_msgs::Odometry* 				        odometry_msg, 
	nav_msgs::OccupancyGrid* 			        map_msg, 
	sensor_msgs::PointCloud2* 			        point_cloud_msg, 
	sensor_msgs::Image* 			 	        left_image_msg, 
	sensor_msgs::Image* 				        right_image_msg, 
	geometry_msgs::PoseWithCovarianceStamped*   odom_combined_msg, 
	sensor_msgs::Imu* 					        imu_msg, 
	sensor_msgs::NavSatFix* 			        gps_msg,  
    bluedragon_description::temperature*        temperature_msg,
	bluedragon_netcomm::serial_size* 	        serial_size_msg)
	{
        namespace ser = ros::serialization;

    // serialize message buffer
        uint32_t ss_serial_size = ser::serializationLength(*serial_size_msg);
        boost::shared_array<uint8_t> ss_buffer(new uint8_t[ss_serial_size]);
 	   
        //Receive an incoming message
        if( recv(ss_socket, ss_buffer.get(), ss_serial_size, 0) < 0) 
        {      
	         perror("ss_Received failed");
        }
        // deserialize stream
        ser::IStream ss_stream(ss_buffer.get(), ss_serial_size);
        ser::deserialize(ss_stream, *serial_size_msg);


	 ///////////////////////////////////////////////////////
	 // left propulsion message

	      // serialize message buffer
	      uint32_t pr_serial_size = serial_size_msg->pr_serial_size;
 	      boost::shared_array<uint8_t> pr_buffer(new uint8_t[pr_serial_size]);
           
    	     //Receive an incoming message
	     if( recv(pr_socket, pr_buffer.get(), pr_serial_size, 0) < 0) 
	     {      
		 perror("pr_Received failed");
   	     }
   	      
   	     ser::IStream pr_stream(pr_buffer.get(), pr_serial_size);
         ser::deserialize(pr_stream, *left_propulsion_msg);
    		 

	 ///////////////////////////////////////////////////////
	 // right propulsion message

	      // serialize message buffer
	      uint32_t pr2_serial_size = serial_size_msg->pr2_serial_size;
 	      boost::shared_array<uint8_t> pr2_buffer(new uint8_t[pr2_serial_size]);
           
    	     //Receive an incoming message
	     if( recv(pr2_socket, pr2_buffer.get(), pr2_serial_size, 0) < 0) 
	     {      
		 perror("pr2_Received failed");
   	     }
   	      
   	     ser::IStream pr2_stream(pr2_buffer.get(), pr2_serial_size);
             ser::deserialize(pr2_stream, *right_propulsion_msg);
    		 

	 ///////////////////////////////////////////////////////
	 // command velocity message

	      // serialize message buffer
	      uint32_t cv_serial_size = serial_size_msg->cv_serial_size;
 	      boost::shared_array<uint8_t> cv_buffer(new uint8_t[cv_serial_size]);
           
    	     //Receive an incoming message
	     if( recv(cv_socket, cv_buffer.get(), cv_serial_size, 0) < 0) 
	     {      
		 perror("cv_Received failed");
   	     }
   	      
   	     ser::IStream cv_stream(cv_buffer.get(), cv_serial_size);
             ser::deserialize(cv_stream, *cmd_vel_msg);
    		
         ///////////////////////////////////////////////////////
	 // laser scan message

	      // serialize message buffer
 	      uint32_t ls_serial_size = serial_size_msg->ls_serial_size;
 	      boost::shared_array<uint8_t> ls_buffer(new uint8_t[ls_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(ls_socket, ls_buffer.get(), ls_serial_size, 0) < 0) 
	      {      
	          perror("ls_Received failed");
    	      }   
    		 
    	      ser::IStream ls_stream(ls_buffer.get(), ls_serial_size);
    	      ser::deserialize(ls_stream, *laser_scan_msg);	
    		 
         ///////////////////////////////////////////////////////
	 // range 1 message
	   
	      // serialize message buffer
 	      uint32_t ra1_serial_size = serial_size_msg->ra1_serial_size;
 	      boost::shared_array<uint8_t> ra1_buffer(new uint8_t[ra1_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(ra1_socket, ra1_buffer.get(), ra1_serial_size, 0) < 0) 
	      {      
		   perror("ra_Received failed");
   	      }
    		 
    	      ser::IStream ra1_stream(ra1_buffer.get(), ra1_serial_size);
    	      ser::deserialize(ra1_stream, *range_1_msg);
    		 
         ///////////////////////////////////////////////////////
	 // range 2 message
	   
	      // serialize message buffer
 	      uint32_t ra2_serial_size = serial_size_msg->ra2_serial_size;
 	      boost::shared_array<uint8_t> ra2_buffer(new uint8_t[ra2_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(ra2_socket, ra2_buffer.get(), ra2_serial_size, 0) < 0) 
	      {      
		   perror("ra_Received failed");
   	      }
    		 
    	      ser::IStream ra2_stream(ra2_buffer.get(), ra2_serial_size);
    	      ser::deserialize(ra2_stream, *range_2_msg);
    		 
         ///////////////////////////////////////////////////////
	 // odometry message
	 
	      // serialize message buffer
 	      uint32_t od_serial_size = serial_size_msg->od_serial_size;
 	      boost::shared_array<uint8_t> od_buffer(new uint8_t[od_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(od_socket, od_buffer.get(), od_serial_size, 0) < 0) 
	      {      
		   perror("od_Received failed");
   	      }
   	      
   	      ser::IStream od_stream(od_buffer.get(), od_serial_size);
    	      ser::deserialize(od_stream, *odometry_msg);
    		 

         ///////////////////////////////////////////////////////
	 // robot pose ekf odom combined message
	 
	      // serialize message buffer
 	      uint32_t oc_serial_size = serial_size_msg->oc_serial_size;
 	      boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(oc_socket, oc_buffer.get(), oc_serial_size, 0) < 0) 
	      {      
		   perror("oc_Received failed");
   	      }
   	      
   	      ser::IStream oc_stream(oc_buffer.get(), oc_serial_size);
    	      ser::deserialize(oc_stream, *odom_combined_msg);
    		     		 
         ///////////////////////////////////////////////////////
	 // map message
	      
	      // serialize message buffer
              uint32_t ma_serial_size = serial_size_msg->ma_serial_size;
 	      boost::shared_array<uint8_t> ma_buffer(new uint8_t[ma_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(ma_socket, ma_buffer.get(), ma_serial_size, 0) < 0) 
	      {      
		   perror("ma_Received failed");
   	      }
	
	      ser::IStream ma_stream(ma_buffer.get(), ma_serial_size);
    	      ser::deserialize(ma_stream, *map_msg);
	 
	   ///////////////////////////////////////////////////////
	 // imu message
	      // serialize message buffer
              uint32_t im_serial_size = serial_size_msg->im_serial_size;
 	      boost::shared_array<uint8_t> im_buffer(new uint8_t[im_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(im_socket, im_buffer.get(), im_serial_size, 0) < 0) 
	      {      
		   perror("im_Received failed");
   	      }
	
	      ser::IStream im_stream(im_buffer.get(), im_serial_size);
    	      ser::deserialize(im_stream, *imu_msg);
	 
         ///////////////////////////////////////////////////////
	 // gps message
	      
	      // serialize message buffer
              uint32_t gp_serial_size = serial_size_msg->gp_serial_size;
 	      boost::shared_array<uint8_t> gp_buffer(new uint8_t[gp_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(gp_socket, gp_buffer.get(), gp_serial_size, 0) < 0) 
	      {      
		   perror("gp_Received failed");
   	      }
	
	      ser::IStream gp_stream(gp_buffer.get(), gp_serial_size);
    	      ser::deserialize(gp_stream, *gps_msg);
	 
         ///////////////////////////////////////////////////////
	 // zed point cloud message
	      
	      // serialize message buffer
              uint32_t pc_serial_size = serial_size_msg->pc_serial_size;
 	      boost::shared_array<uint8_t> pc_buffer(new uint8_t[pc_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(pc_socket, pc_buffer.get(), pc_serial_size, 0) < 0) 
	      {      
		   perror("pc_Received failed");
   	      }
	
	      ser::IStream pc_stream(pc_buffer.get(), pc_serial_size);
    	      ser::deserialize(pc_stream, *point_cloud_msg);
	 
        ///////////////////////////////////////////////////////
	 // zed left_image message
	      
	      // serialize message buffer
              uint32_t li_serial_size = serial_size_msg->li_serial_size;
 	      boost::shared_array<uint8_t> li_buffer(new uint8_t[li_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(li_socket, li_buffer.get(), li_serial_size, 0) < 0) 
	      {      
		   perror("li_Received failed");
   	      }
	
	      ser::IStream li_stream(li_buffer.get(), li_serial_size);
    	      ser::deserialize(li_stream, *left_image_msg);
	 
        ///////////////////////////////////////////////////////
	 // zed right_image message
	      
	      // serialize message buffer
              uint32_t ri_serial_size = serial_size_msg->ri_serial_size;
 	      boost::shared_array<uint8_t> ri_buffer(new uint8_t[ri_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(ri_socket, ri_buffer.get(), ri_serial_size, 0) < 0) 
	      {      
		   perror("ri_Received failed");
   	      }
	
	      ser::IStream ri_stream(ri_buffer.get(), ri_serial_size);
    	      ser::deserialize(ri_stream, *right_image_msg);

        ///////////////////////////////////////////////////////
	 // temperature message
	      
	      // serialize message buffer
              uint32_t tp_serial_size = serial_size_msg->tp_serial_size;
 	      boost::shared_array<uint8_t> tp_buffer(new uint8_t[tp_serial_size]);
 		 
    	      //Receive an incoming message
	      if( recv(tp_socket, tp_buffer.get(), tp_serial_size, 0) < 0) 
	      {      
		   perror("tp_Received failed");
   	      }
	
	      ser::IStream tp_stream(tp_buffer.get(), tp_serial_size);
    	  ser::deserialize(tp_stream, *temperature_msg);

	 
    }          
#endif // PRIAPUS_H
