#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <OnePointFiveGenLaser.hpp>
#include <SecondGenLaser.hpp>
#include <hls_lfcd_lds_driver/SetLidar.h>
#include <hls_lfcd_lds_driver/scan_ctrl.h>
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
			laser->lidarPmGpio('1');
			laser->shutting_down_ = false;
			laser->first_power_on_ = true;
		}
	}
	else if (req.switch_status == 0)
	{
		if(!laser->shutting_down_)
		{
			laser->lidarPmGpio('0');
			laser->shutting_down_ = true;
		}
	}
	res.message = "Driver ommand received:";
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
		laser->lidar_matrix_ = laser->transform_now_ * laser->transform_last_.inverse() * laser->lidar_matrix_; // lidar_matrix_ in base_link coordinate
		laser->transform_last_ = laser->transform_now_;
		//publish marker
		Double_Point point{};
		std::vector<int>::const_iterator ite = laser->noiseNum_.begin();
		for (int i = 0; i < laser->lidar_matrix_.cols(); i++)
		{
			if (ite != laser->noiseNum_.end() && i == *ite)
			{
				ite++;
				continue;
			}
			point.x = laser->lidar_matrix_(0, i);
			point.y = laser->lidar_matrix_(1, i);
			if (fabs(point.x) < 4 && fabs(point.y) < 4)
				points_vec.push_back(point);
		}
		//publish scan
		laser->publishScanCompensate(laser->transform_lidar_baselink_.inverse() * laser->lidar_matrix_);
		laser->pubPointMarker(&points_vec);
		laser->scanXY_mutex_.unlock();
	}
}

void *scanCtrlRoutine(void *)
{
	pthread_detach(pthread_self());
	ROS_INFO("\033[34mhls_lfcd_lds_driver/node.cpp. %s %d: scan_ctrl thread is up.\033[0m", __FUNCTION__, __LINE__);
	ros::spin();
}

int main(int argc, char **argv)
{
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
	else
	{
		ROS_ERROR("Parameter laserGen is wrong! Please input 1 or 2");
		return -1;
	}

	pthread_t scan_ctrl_thread_id;
	int8_t tmp_ret = pthread_create(&scan_ctrl_thread_id, nullptr, scanCtrlRoutine, nullptr);
	while (tmp_ret != 0)
	{
		ROS_ERROR("%s %d: Create scan_ctrl_thread failed, retry!", __FUNCTION__, __LINE__);
		usleep(50000);
		tmp_ret = pthread_create(&scan_ctrl_thread_id, nullptr, scanCtrlRoutine, nullptr);
	}

	ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("/lds/rpms", 1000);
	laser->point_marker_pub_ = n.advertise<visualization_msgs::Marker>("/lds/lidarPoint", 1000);
	laser->scan_original_pub_ = n.advertise<sensor_msgs::LaserScan>("/lds/scanOriginal", 1000);
	laser->scan_linear_pub_ = n.advertise<sensor_msgs::LaserScan>("/lds/scanLinear", 1000);
	laser->scan_compensate_pub_ = n.advertise<sensor_msgs::LaserScan>("/lds/scanCompensate", 1000);
	auto motor_service = n.advertiseService("/lds/lidar_motor_ctrl", lidarMotorCtrl);
	ROS_WARN("lidarMotorCtrl service is up.");

	auto scan_ctrl_sub = n.subscribe("/scan_ctrl", 1, &scanCtrlCb);
	auto odom_sub = n.subscribe("/odom", 1, &odomCb);

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

	while (ros::ok())
	{
		if (laser->shutting_down_)
		{
			usleep(20000);
			continue;
		}
		sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
		scan->header.frame_id = laser->frame_id_;
		try
		{
			laser->poll(scan);
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
			ROS_WARN("An exception was thrown: %s", ex.what());
			laser->readFalse();
			continue;
		}
	}

	int retry_count = 1;
	while(!laser->lidarPmGpio('0') && retry_count < 4)
	{
		ROS_ERROR("\033[34mrplidar_ros/node.cpp. %s %d: Power down lidar failed, retry.\033[0m", __FUNCTION__, __LINE__);
		usleep(800000);
		retry_count++;
	}
	delete laser;
	return 0;
}
