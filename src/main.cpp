#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <FirstGenLaser.hpp>
#include <SecondGenLaser.hpp>
#include <hls_lfcd_lds_driver/SetLidar.h>
#include <hls_lfcd_lds_driver/scan_ctrl.h>
#include "CYdLidar.h"
#include <signal.h>

using namespace ydlidar;

Device* laser;
int laserGen = 1;

bool lidarMotorCtrl(hls_lfcd_lds_driver::SetLidar::Request &req, hls_lfcd_lds_driver::SetLidar::Response &res)
{
	if(laser == nullptr)
		return false;

	if (req.switch_status != 0)
	{
		if (laser->shutting_down_)
		{
			ROS_INFO("\033[1m" "[lds driver] %s %d: Start Motor" "\033[0m", __FUNCTION__, __LINE__);
			laser->motor_start_flag_ = true;
			laser->motor_stop_flag_  = false;
			laser->shutting_down_ = false;
		}
	}
	else if (req.switch_status == 0)
	{
		ROS_INFO("\033[1m" "[lds driver] %s %d: Stop Motor" "\033[0m", __FUNCTION__, __LINE__);
		laser->motor_start_flag_ = false;
		laser->motor_stop_flag_  = true;
		laser->shutting_down_ = true;
	}
	res.message = "Driver command received:";
	res.message += (req.switch_status != 0 ? "true" : "false");
	res.success = static_cast<unsigned char>(true);
	return true;
}

void scanCtrlCb(const hls_lfcd_lds_driver::scan_ctrl::ConstPtr &msg)
{
	if (!msg->allow_publishing)
		laser->skip_scan_ = true;
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	if(laser == nullptr)
		return;
	//update odom position
	laser->now_x_ = msg->pose.pose.position.x;
	laser->now_y_ = msg->pose.pose.position.y;
	laser->now_yaw_ = tf::getYaw(msg->pose.pose.orientation);
//	ROS_INFO("now x: %f", laser->now_x_);

	if (laser->now_yaw_ < 0)
		laser->now_yaw_ += 2 * M_PI;

	laser->odom_update_time_ = ros::Time::now().toSec();

	if (!laser->shutting_down_ && laser->checkFresh(2, 1000))
	{
		laser->scanXY_mutex_.lock();
		std::vector<Double_Point> points_vec;
		//compensate ScanData
		laser->transform_now_ << cos(laser->now_yaw_), sin(laser->now_yaw_), -laser->now_y_ * sin(laser->now_yaw_) - laser->now_x_ * cos(laser->now_yaw_),
							-sin(laser->now_yaw_), cos(laser->now_yaw_), laser->now_x_ * sin(laser->now_yaw_) - laser->now_y_ * cos(laser->now_yaw_),
							0, 0, 1;
		Eigen::MatrixXd now_laser_matrix(laser->transform_now_  * laser->lidar_matrix_); // lidar_matrix_ in base_link coordinate
		//publish marker
		Double_Point point{};
		std::vector<int>::const_iterator ite = laser->noiseNum_.begin();
		for (int i = 0; i < now_laser_matrix.cols(); i++)
		{
			if (ite != laser->noiseNum_.end() && i == *ite)
			{
				ite++;
				continue;
			}
			point.x = now_laser_matrix(0, i);
			point.y = now_laser_matrix(1, i);
			if (fabs(point.x) < 4 && fabs(point.y) < 4)
				points_vec.push_back(point);
		}
		//publish scan
		laser->publishScanCompensate(laser->transform_lidar_baselink_.inverse() * now_laser_matrix,
									 msg->header.stamp.toSec());
		laser->pubPointMarker(&points_vec);
		laser->scanXY_mutex_.unlock();
	}
//	ROS_INFO("cb end");
}

void *scanCtrlRoutine(void *)
{
	pthread_detach(pthread_self());
	ros::Rate r(100);
	ROS_INFO("[lds driver] %s %d: scan_ctrl thread is up.", __FUNCTION__, __LINE__);
	while (ros::ok())
	{
		r.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	// Set the stdout buffer to one line.
	setvbuf(stdout, NULL, _IOLBF, 0);

	ros::init(argc, argv, "hlds_laser_publisher");
	ros::NodeHandle n;
	ros::NodeHandle nh_private("~");

	std::string port = "/dev/ttyS1";
	nh_private.param("port", port, std::string("/dev/ttyS1"));
	nh_private.param<int>("laserGen", laserGen, 1);
	boost::asio::io_service io;
	boost::asio::serial_port serial(io, port);

	//Init for laser instance
	if (laserGen == 1)
		laser = new LFCDLaserFirstGen(&serial);
	else if (laserGen == 2)
		laser = new LFCDLaserSecondGen(&serial);
	else if(laserGen == 3)
		laser = new CYdLidar(port);
	else
	{
		ROS_ERROR("[lds driver] Parameter laserGen is wrong! Please input 1 or 2");
		return -1;
	}

	pthread_t scan_ctrl_thread_id;
	int8_t tmp_ret = pthread_create(&scan_ctrl_thread_id, nullptr, scanCtrlRoutine, nullptr);
	while (tmp_ret != 0)
	{
		ROS_ERROR("[lds driver] %s %d: Create scan_ctrl_thread failed, retry!", __FUNCTION__, __LINE__);
		usleep(50000);
		tmp_ret = pthread_create(&scan_ctrl_thread_id, nullptr, scanCtrlRoutine, nullptr);
	}

	ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("/lds/rpms", 1000);
	laser->point_marker_pub_ = n.advertise<visualization_msgs::Marker>("/lds/lidarPoint", 1000);
	laser->scan_original_pub_ = n.advertise<sensor_msgs::LaserScan>("/lds/scanOriginal", 1000);
	laser->scan_linear_pub_ = n.advertise<sensor_msgs::LaserScan>("/lds/scanLinear", 1000);
	laser->scan_compensate_pub_ = n.advertise<sensor_msgs::LaserScan>("/lds/scanCompensate", 1000);
	auto motor_service = n.advertiseService("/lds/lidar_motor_ctrl", lidarMotorCtrl);
	ROS_WARN("[lds driver] lidarMotorCtrl service is up.");

	auto scan_ctrl_sub = n.subscribe("/pp/scan_ctrl", 1, &scanCtrlCb, ros::TransportHints().unreliable());
	auto odom_sub = n.subscribe("/odom", 2, &odomCb, ros::TransportHints().unreliable());

	nh_private.param<double>("LIDAR_OFFSET_X", laser->LIDAR_OFFSET_X_, 0);
	nh_private.param<double>("LIDAR_OFFSET_Y", laser->LIDAR_OFFSET_Y_, 0);
	nh_private.param<double>("LIDAR_OFFSET_THETA", laser->LIDAR_OFFSET_THETA_, 0);
	nh_private.param<int>("delay_when_republish", laser->delay_when_republish_, 1);
	nh_private.param("frame_id", laser->frame_id_, std::string("laser"));
#if LIDAR_BLOCK_RANGE_ENABLE
	nh_private.param<int>("block_angle_1", laser->block_angle_1_, -1);
	nh_private.param<int>("block_angle_2", laser->block_angle_2_, -1);
	nh_private.param<int>("block_angle_3", laser->block_angle_3_, -1);
	nh_private.param<int>("block_range", laser->block_range_, 15);
#endif

	auto tmp_theta = laser->LIDAR_OFFSET_THETA_ * M_PI / 180.0;
	laser->transform_lidar_baselink_ << cos(tmp_theta), -sin(tmp_theta), laser->LIDAR_OFFSET_X_,
						sin(tmp_theta), cos(tmp_theta), laser->LIDAR_OFFSET_Y_,
						0, 0, 1;
	ROS_WARN("tmp_theta:%.2f",tmp_theta);

	while (ros::ok())
	{
/*		laser->checkChangeLidarPower();

		if (laser->shutting_down_)
		{
			usleep(20000);
			continue;
		}*/

		sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
		scan->header.frame_id = laser->frame_id_;
		try
		{
			laser->poll(scan);
//			ROS_INFO("scan.ranges[65] = %.2f", scan->ranges[65]); // Left of robot.
//			ROS_INFO("scan.ranges[155] = %.2f", scan->ranges[155]); // Back of robot.
//			ROS_INFO("scan.ranges[245] = %.2f", scan->ranges[245]); // Right of robot.
			scan->header.stamp = ros::Time::now();
			std_msgs::UInt16 rpm;
			rpm.data = laser->rpms_;
			motor_pub.publish(rpm);
			laser->scan_original_pub_.publish(scan);
			laser->delayPub(&laser->scan_linear_pub_, scan);
		}
		//Power off laser when laser is in read data.
		catch (const char *msg)
		{
			ROS_WARN("%s",msg);
			continue;
		}
		//Time out when laser is in read data.
		catch (boost::system::system_error ex)
		{
			ROS_WARN("[lds driver] An exception was thrown: %s", ex.what());
			laser->readFalse();
			continue;
		}
	}

	if(laserGen == 3)
	{
		auto ptr = dynamic_cast<CYdLidar*>(laser);
		ptr->turnOff();
		ptr->disconnecting();
	}
	int retry_count = 1;
	while(!laser->lidarPmGpio('0') && retry_count < 4)
	{
		ROS_ERROR("[lds driver] %s %d: Power down lidar failed, retry.", __FUNCTION__, __LINE__);
		usleep(800000);
		retry_count++;
	}
	delete laser;
	return 0;
}
