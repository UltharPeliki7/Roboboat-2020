#ifndef LISTENER_H
#define LISTENER_H

#include <bluedragon_propulsion/propulsion.h>
#include <bluedragon_description/temperature.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// temperature listener 
class temperature_listener
{
	private:
		bluedragon_description::temperature temp_temperature_msg;
		
	public:
		void temperature_callback(const bluedragon_description::temperature& msg)
		{
			temp_temperature_msg=msg;
		}
	    bluedragon_description::temperature get_temperature_msg(){return temp_temperature_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// imu listener 
class imu_listener
{
	private:
		sensor_msgs::Imu temp_imu_msg;
		
	public:
		void imu_callback(const sensor_msgs::Imu& msg)
		{
			temp_imu_msg=msg;
		}
	    sensor_msgs::Imu get_imu_msg(){return temp_imu_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// gps listener 
class gps_listener
{
	private:
		sensor_msgs::NavSatFix temp_gps_msg;
		
	public:
		void gps_callback(const sensor_msgs::NavSatFix& msg)
		{
			temp_gps_msg=msg;
		}
	    sensor_msgs::NavSatFix get_gps_msg(){return temp_gps_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// camera image listener
class image_listener
{
	private:
		sensor_msgs::Image temp_image_msg;
		
	public:
		void image_callback(const sensor_msgs::Image& msg)
		{
			temp_image_msg=msg;
		}
		sensor_msgs::Image get_image_msg(){return temp_image_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// point cloud listener 
class point_cloud_listener
{
	private:
		sensor_msgs::PointCloud2 temp_point_cloud_msg;
		
	public:
		void point_cloud_callback(const sensor_msgs::PointCloud2& msg)
		{
			temp_point_cloud_msg=msg;
		}
	    sensor_msgs::PointCloud2 get_point_cloud_msg(){return temp_point_cloud_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// odom combined listener 
class odom_combined_listener
{
	private:
		geometry_msgs::PoseWithCovarianceStamped temp_odom_combined_msg;
		
	public:
		void odom_combined_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
		{
			temp_odom_combined_msg=msg;
		}
		geometry_msgs::PoseWithCovarianceStamped get_odom_combined_msg(){return temp_odom_combined_msg;}
};

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// laser scan listener
class laser_scan_listener
{
	private:
		sensor_msgs::LaserScan temp_laser_scan_msg;
		
	public:
		void laser_scan_callback(const sensor_msgs::LaserScan& msg)
		{
			temp_laser_scan_msg=msg;
		}
		sensor_msgs::LaserScan get_laser_scan_msg(){return temp_laser_scan_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// odometry listener 
class odometry_listener
{
	private:
		nav_msgs::Odometry temp_odometry_msg;
		
	public:
		void odometry_callback(const nav_msgs::Odometry& msg)
		{
			temp_odometry_msg=msg;
		}
		nav_msgs::Odometry get_odometry_msg(){return temp_odometry_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// propulsion listener
class propulsion_listener
{
	private:
		bluedragon_propulsion::propulsion temp_propulsion_msg;
		
	public:
		void propulsion_callback(const bluedragon_propulsion::propulsion& msg)
		{
			temp_propulsion_msg=msg;
		}
		bluedragon_propulsion::propulsion get_propulsion_msg(){return temp_propulsion_msg;}	
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// ultrasound listener
class range_listener
{
	private:
		sensor_msgs::Range temp_range_msg;
	
	public:
		void range_callback(const sensor_msgs::Range& msg)
		{
			temp_range_msg=msg;
		}
		sensor_msgs::Range get_range_msg(){return temp_range_msg;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// map listener 
class map_listener
{
	private:
		nav_msgs::OccupancyGrid temp_map_msg;
		
	public:
		void map_callback(const nav_msgs::OccupancyGrid& msg)
		{
			temp_map_msg=msg;
		}
		nav_msgs::OccupancyGrid get_map_msg(){return temp_map_msg;}
};

#endif // LISTENER_H
