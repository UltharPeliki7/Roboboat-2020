#ifndef HERMES_H
#define HERMES_H

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

/* hermes class declaration
 *
*/
class hermes
{
    /* chariot private member variables
     *
    */
	private:
        /* serial size socket 
         *
        */
		struct sockaddr_in    ss_si;

        /* propulsion socket 
         *
        */
	    struct sockaddr_in    pr_si;

        /* propulsion socket 
         *
        */
	    struct sockaddr_in    pr2_si;

        /* command velocity socket 
         *
        */
	    struct sockaddr_in    cv_si;

        /* laser scan socket
         *
        */
	    struct sockaddr_in    ls_si;

        /* ultrasound_1 socket 
         *
        */
	    struct sockaddr_in    ra1_si;

        /* ultrasound_2  socket 
         *
        */
	    struct sockaddr_in    ra2_si;

        /* odometry socket 
         *
        */
	    struct sockaddr_in    od_si;

        /* odom combined socket 
         *
        */
	    struct sockaddr_in    oc_si;

        /* map socket
         *
        */
	    struct sockaddr_in    ma_si;

        /* point cloud socket 
         *
        */
	    struct sockaddr_in    pc_si;

        /* left image socket 
         *
        */
	    struct sockaddr_in    li_si;

        /* right image socket 
         *
        */
	    struct sockaddr_in    ri_si;

        /* imu socket 
         *
        */
	    struct sockaddr_in    im_si;

        /* gps  socket 
         *
        */
	    struct sockaddr_in 	  gp_si;

        /* temperature  socket 
         *
        */
	    struct sockaddr_in 	  tp_si;

 
    /* chariot public functions
     *
    */
	public:
	    /* Default Constructor declaration
	     *
		*/
	    hermes();

	    /* Initializing Constructor declaration
	     *
		*/
	    hermes(
			int      port_group, 
			char*    ip);

	    /* Deconstructor 
	     *
		*/
	    ~hermes(){}

	    /* set port data function declaration
	     *
		*/
	    void set_port_data(
			int      port_group, 
			char*    ip);

	    /* hermes function declaration
	     *
		*/
	    void chariot(
            int*                                        sock,
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
hermes::hermes()
	{
		set_port_data(0, 0);
	}

/* initializing constructor definition
 *
*/
hermes::hermes(
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
void hermes::set_port_data(
	int      PG, 
	char*    ip)
    {	
	    /* assign serial size message to start_port
	     *
		*/
        ss_si.sin_family = AF_INET;
        ss_si.sin_port   = htons(PG);
        inet_aton( ip, &ss_si.sin_addr);

	    /* assign left propulsion message to start_port + 1
	     *
		*/
        pr_si.sin_family = AF_INET;
        pr_si.sin_port   = htons(PG + 1);
        inet_aton( ip, &pr_si.sin_addr);

	    /* assign right propulsion message to start_port + 4
	     *
		*/
        pr2_si.sin_family = AF_INET;
        pr2_si.sin_port   = htons(PG + 4);
        inet_aton( ip, &pr2_si.sin_addr);

	    /* assign command velocity message to start_port + 2
	     *
		*/
        cv_si.sin_family = AF_INET;
        cv_si.sin_port   = htons(PG + 2);
        inet_aton( ip, &cv_si.sin_addr);

	    /* assign laser scan message to start_port + 3
	     *
		*/
        ls_si.sin_family = AF_INET;
        ls_si.sin_port   = htons(PG + 3);
        inet_aton( ip, &ls_si.sin_addr);

	    /* assign ultrasound_1 message to start_port + 5
	     *
		*/
        ra1_si.sin_family = AF_INET;
        ra1_si.sin_port   = htons(PG + 5);
        inet_aton( ip, &ra1_si.sin_addr);

	    /* assign odometry message to start_port + 6
	     *
		*/
        od_si.sin_family = AF_INET;
        od_si.sin_port   = htons(PG + 6);
        inet_aton( ip, &od_si.sin_addr);

	    /* assign map message to start_port + 7
	     *
		*/
        ma_si.sin_family = AF_INET;
        ma_si.sin_port   = htons(PG + 7);
        inet_aton( ip, &ma_si.sin_addr);

	    /* assign ultrasound_2 message to start_port + 8
	     *
		*/
        ra2_si.sin_family = AF_INET;
        ra2_si.sin_port   = htons(PG + 8);
        inet_aton( ip, &ra2_si.sin_addr);

	    /* assign point cloud message to start_port + 9
	     *
		*/
        pc_si.sin_family = AF_INET;
        pc_si.sin_port   = htons(PG + 9);
        inet_aton( ip, &pc_si.sin_addr);
 
	    /* assign left image message to start_port + 10
	     *
		*/
        li_si.sin_family = AF_INET;
        li_si.sin_port   = htons(PG + 10);
        inet_aton( ip, &li_si.sin_addr);

	    /* assign right image message to start_port + 11
	     *
		*/
        ri_si.sin_family = AF_INET;
        ri_si.sin_port   = htons(PG + 11);
        inet_aton( ip, &ri_si.sin_addr);

	    /* assign odom combined message to start_port + 12
	     *
		*/
        oc_si.sin_family = AF_INET;
        oc_si.sin_port   = htons(PG + 12);
        inet_aton( ip, &oc_si.sin_addr);

	    /* assign imu message to start_port + 13
	     *
		*/
        im_si.sin_family = AF_INET;
        im_si.sin_port   = htons(PG + 13);
        inet_aton( ip, &im_si.sin_addr);

	    /* assign gps message to start_port + 14
	     *
		*/
        gp_si.sin_family = AF_INET;
        gp_si.sin_port   = htons(PG + 14);
        inet_aton( ip, &gp_si.sin_addr);

	    /* assign temperature message to start_port + 15
	     *
		*/
        tp_si.sin_family = AF_INET;
        tp_si.sin_port   = htons(PG + 15);
        inet_aton( ip, &tp_si.sin_addr);

    }

/* hermes definition
 *
*/
void hermes::chariot(
    int*                                        sock, 
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
	     /* create variable ser for ros::serialization to save space
	      *
		 */
		 namespace ser = ros::serialization;

	     /* left propulsion message serialization
	      *
		 */	
 	     uint32_t pr_serial_size = ser::serializationLength(*left_propulsion_msg);
         boost::shared_array<uint8_t> pr_buffer(new uint8_t[pr_serial_size]);		            
         ser::OStream pr_stream(pr_buffer.get(), pr_serial_size);
         ser::serialize(pr_stream, *left_propulsion_msg);   
         size_t pr_nBytes = sendto(*sock, pr_buffer.get(), pr_serial_size, 0, (struct sockaddr*) &pr_si, sizeof(pr_si));

	     /* right propulsion message serialization
	      *
		 */	
 	     uint32_t pr2_serial_size = ser::serializationLength(*right_propulsion_msg);
         boost::shared_array<uint8_t> pr2_buffer(new uint8_t[pr2_serial_size]);		            
         ser::OStream pr2_stream(pr2_buffer.get(), pr2_serial_size);
         ser::serialize(pr2_stream, *right_propulsion_msg);   
         size_t pr2_nBytes = sendto(*sock, pr2_buffer.get(), pr2_serial_size, 0, (struct sockaddr*) &pr2_si, sizeof(pr2_si));

	     /* command velocity message serialization
	      *
		 */
 	     uint32_t cv_serial_size = ser::serializationLength(*cmd_vel_msg);
 	     boost::shared_array<uint8_t> cv_buffer(new uint8_t[cv_serial_size]);		            
         ser::OStream cv_stream(cv_buffer.get(),cv_serial_size);
         ser::serialize(cv_stream, *cmd_vel_msg);          
         size_t cv_nBytes = sendto(*sock, cv_buffer.get(), cv_serial_size, 0, (struct sockaddr*) &cv_si, sizeof(cv_si));

	     /* laser scan message serialization
	      *
		 */
 	     uint32_t ls_serial_size = ser::serializationLength(*laser_scan_msg);
 	     boost::shared_array<uint8_t> ls_buffer(new uint8_t[ls_serial_size]);		            
         ser::OStream ls_stream(ls_buffer.get(),ls_serial_size);
         ser::serialize(ls_stream, *laser_scan_msg);
	     size_t ls_nBytes = sendto(*sock, ls_buffer.get(), ls_serial_size, 0, (struct sockaddr*) &ls_si, sizeof(ls_si));
	  
	     /* ultrasound_1 message serialization
	      *
		 */
 	     uint32_t ra1_serial_size = ser::serializationLength(*range_1_msg);
 	     boost::shared_array<uint8_t> ra1_buffer(new uint8_t[ra1_serial_size]);		            
         ser::OStream ra1_stream(ra1_buffer.get(),ra1_serial_size);
         ser::serialize(ra1_stream, *range_1_msg);         
         size_t ra1_nBytes = sendto(*sock, ra1_buffer.get(), ra1_serial_size, 0, (struct sockaddr*) &ra1_si, sizeof(ra1_si));

	     /* ultrasound_2 message serialization
	      *
		 */
 	     uint32_t ra2_serial_size = ser::serializationLength(*range_2_msg);
 	     boost::shared_array<uint8_t> ra2_buffer(new uint8_t[ra2_serial_size]);		            
         ser::OStream ra2_stream(ra2_buffer.get(),ra2_serial_size);
         ser::serialize(ra2_stream, *range_2_msg);          
         size_t ra2_nBytes = sendto(*sock, ra2_buffer.get(), ra2_serial_size, 0, (struct sockaddr*) &ra2_si, sizeof(ra2_si));

	     /* odometry message serialization
	      *
		 */	 
 	     uint32_t od_serial_size = ser::serializationLength(*odometry_msg);
 	     boost::shared_array<uint8_t> od_buffer(new uint8_t[od_serial_size]);		            
         ser::OStream od_stream(od_buffer.get(),od_serial_size);
         ser::serialize(od_stream, *odometry_msg);           
         size_t od_nBytes = sendto(*sock, od_buffer.get(), od_serial_size, 0, (struct sockaddr*) &od_si, sizeof(od_si));

	     /* map message serialization
	      *
		 */
 	     uint32_t ma_serial_size = ser::serializationLength(*map_msg);
 	     boost::shared_array<uint8_t> ma_buffer(new uint8_t[ma_serial_size]);            
         ser::OStream ma_stream(ma_buffer.get(),ma_serial_size);
         ser::serialize(ma_stream, *map_msg);
         size_t ma_nBytes = sendto(*sock, ma_buffer.get(), ma_serial_size, 0, (struct sockaddr*) &ma_si, sizeof(ma_si));

	     /* point cloud message serialization
	      *
		 */
 	     uint32_t pc_serial_size = ser::serializationLength(*point_cloud_msg);
 	     boost::shared_array<uint8_t> pc_buffer(new uint8_t[pc_serial_size]);       
         ser::OStream pc_stream(pc_buffer.get(),pc_serial_size);
         ser::serialize(pc_stream, *point_cloud_msg);
         size_t pc_nBytes = sendto(*sock, pc_buffer.get(), pc_serial_size, 0, (struct sockaddr*) &pc_si, sizeof(pc_si));

	     /* left image message serialization
	      *
		 */
 	     uint32_t li_serial_size = ser::serializationLength(*left_image_msg);
 	     boost::shared_array<uint8_t> li_buffer(new uint8_t[li_serial_size]);       
         ser::OStream li_stream(li_buffer.get(),li_serial_size);
         ser::serialize(li_stream, *left_image_msg);
         size_t li_nBytes = sendto(*sock, li_buffer.get(), li_serial_size, 0, (struct sockaddr*) &li_si, sizeof(li_si));

	     /* right image message serialization
	      *
		 */
 	     uint32_t ri_serial_size = ser::serializationLength(*right_image_msg);
 	     boost::shared_array<uint8_t> ri_buffer(new uint8_t[ri_serial_size]);       
         ser::OStream ri_stream(ri_buffer.get(),ri_serial_size);
         ser::serialize(ri_stream, *right_image_msg);
         size_t ri_nBytes = sendto(*sock, ri_buffer.get(), ri_serial_size, 0, (struct sockaddr*) &ri_si, sizeof(ri_si));

	     /* odometry combined message serialization
	      *
		 */
 	     uint32_t oc_serial_size = ser::serializationLength(*odom_combined_msg);
 	     boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_serial_size]);       
         ser::OStream oc_stream(oc_buffer.get(),oc_serial_size);
         ser::serialize(oc_stream, *odom_combined_msg);
         size_t oc_nBytes = sendto(*sock, oc_buffer.get(), oc_serial_size, 0, (struct sockaddr*) &oc_si, sizeof(oc_si));

	     /* imu message serialization
	      *
		 */
 	     uint32_t im_serial_size = ser::serializationLength(*imu_msg);
 	     boost::shared_array<uint8_t> im_buffer(new uint8_t[im_serial_size]);       
         ser::OStream im_stream(im_buffer.get(),im_serial_size);
         ser::serialize(im_stream, *imu_msg);
         size_t im_nBytes = sendto(*sock, im_buffer.get(), im_serial_size, 0, (struct sockaddr*) &im_si, sizeof(im_si));

	     /* gps message serialization
	      *
		 */
 	     uint32_t gp_serial_size = ser::serializationLength(*gps_msg);
 	     boost::shared_array<uint8_t> gp_buffer(new uint8_t[gp_serial_size]);       
         ser::OStream gp_stream(gp_buffer.get(),gp_serial_size);
         ser::serialize(gp_stream, *gps_msg);
         size_t gp_nBytes = sendto(*sock, gp_buffer.get(), gp_serial_size, 0, (struct sockaddr*) &gp_si, sizeof(gp_si));

	     /* temperature message serialization
	      *
		 */
 	     uint32_t tp_serial_size = ser::serializationLength(*temperature_msg);
 	     boost::shared_array<uint8_t> tp_buffer(new uint8_t[tp_serial_size]);       
         ser::OStream tp_stream(tp_buffer.get(),tp_serial_size);
         ser::serialize(tp_stream, *temperature_msg);
         size_t tp_nBytes = sendto(*sock, tp_buffer.get(), tp_serial_size, 0, (struct sockaddr*) &tp_si, sizeof(tp_si));


	     /* fill serial size message with the size of all of the other messages
	      *
		 */
	     serial_size_msg  ->  pr_serial_size    =   pr_serial_size;
	     serial_size_msg  ->  pr2_serial_size   =   pr2_serial_size;
	     serial_size_msg  ->  cv_serial_size    =   cv_serial_size;
	     serial_size_msg  ->  ls_serial_size    =   ls_serial_size;
	     serial_size_msg  ->  ra1_serial_size   =   ra1_serial_size;
	     serial_size_msg  ->  ra2_serial_size   =   ra2_serial_size;
	     serial_size_msg  ->  od_serial_size    =   od_serial_size;
	     serial_size_msg  ->  ma_serial_size    =   ma_serial_size;
	     serial_size_msg  ->  pc_serial_size    =   pc_serial_size;
	     serial_size_msg  ->  li_serial_size    =   li_serial_size;
	     serial_size_msg  ->  ri_serial_size    =   ri_serial_size;
	     serial_size_msg  ->  oc_serial_size    =   oc_serial_size;
	     serial_size_msg  ->  im_serial_size    =   im_serial_size;
	     serial_size_msg  ->  gp_serial_size    =   gp_serial_size;
	     serial_size_msg  ->  tp_serial_size    =   tp_serial_size;

	     /* serial size message serialization
	      *
		 */
 	     uint32_t ss_serial_size = ser::serializationLength(*serial_size_msg);
 	     boost::shared_array<uint8_t> ss_buffer(new uint8_t[ss_serial_size]);		       
         ser::OStream ss_stream(ss_buffer.get(), ss_serial_size);
         ser::serialize(ss_stream, *serial_size_msg);           
         size_t ss_nBytes = sendto(*sock, ss_buffer.get(), ss_serial_size, 0, (struct sockaddr*) &ss_si, sizeof(ss_si));
    }          
#endif // HERMES_H
