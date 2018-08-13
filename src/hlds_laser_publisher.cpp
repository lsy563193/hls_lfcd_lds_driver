/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Authors: SP Kong, JH Yang */
/* maintainer: Pyo */

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <hls_lfcd_lds_driver/lfcd_laser.h>
#include <hls_lfcd_lds_driver/SetLidar.h>
#include <hls_lfcd_lds_driver/scan_ctrl.h>
using namespace hls_lfcd_lds;
LFCDLaser* laser;
int laserGen = 1;

bool lidar_motor_ctrl(hls_lfcd_lds_driver::SetLidar::Request &req, hls_lfcd_lds_driver::SetLidar::Response &res)
{
	if(laser == NULL)
		return false;

	if (req.switch_status == true)
	{
		if (laser->lidar_status == OFF)
		{
			laser->lidar_pm_gpio('1');
			laser->first_power_on = true;
		}
	}
	else if (req.switch_status == false)
	{
		laser->lidar_pm_gpio('0');
	}
	res.message = "Driver ommand received:";
	res.message += (req.switch_status ? "true" : "false");
	res.success = true;
	laser->lidar_status = req.switch_status;
	return true;
}

void scan_ctrl_cb(const hls_lfcd_lds_driver::scan_ctrl::ConstPtr &msg)
{
	if (!msg->allow_publishing)
		laser->skip_scan = true;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	if(laser == NULL)
		return;
	//update odom position
	laser->now_x = msg->pose.pose.position.x;
	laser->now_y = msg->pose.pose.position.y;
	laser->now_yaw = tf::getYaw(msg->pose.pose.orientation);
	if (laser->now_yaw < 0)
		laser->now_yaw += 2 * M_PI;

	laser->odom_update_time = ros::Time::now().toSec();

	if (laser->lidar_status && laser->checkFresh(2, 2))
	{
		laser->scanXY_mutex_.lock();
		std::vector<Double_Point> points_vec;
		//compensate ScanData
		laser->t_now << cos(laser->now_yaw), sin(laser->now_yaw), -laser->now_y * sin(laser->now_yaw) - laser->now_x * cos(laser->now_yaw),
							-sin(laser->now_yaw), cos(laser->now_yaw), laser->now_x * sin(laser->now_yaw) - laser->now_y * cos(laser->now_yaw),
							0, 0, 1;
		laser->lidar_matrix = laser->t_now * laser->t_last.inverse() * laser->lidar_matrix; // lidar_matrix in base_link coordinate
		laser->t_last = laser->t_now;
		//publish marker
		Double_Point point;
		std::vector<int>::const_iterator ite = laser->noiseNum.begin();
		for (int i = 0; i < laser->lidar_matrix.cols(); i++)
		{
			if (ite != laser->noiseNum.end() && i == *ite)
			{
				ite++;
				continue;
			}
			point.x = laser->lidar_matrix(0, i);
			point.y = laser->lidar_matrix(1, i);
			if (fabs(point.x) < 4 && fabs(point.y) < 4)
				points_vec.push_back(point);
		}
		//publish scan
		laser->publish_scan_compensate(laser->t_lidar_baselink.inverse() * laser->lidar_matrix);
		laser->pubPointMarker(&points_vec);
		laser->scanXY_mutex_.unlock();
	}
}

void *scan_ctrl_routine(void *)
{
	pthread_detach(pthread_self());
	ROS_INFO("\033[34mhls_lfcd_lds_driver/node.cpp. %s %d: scan_ctrl thread is up.\033[0m", __FUNCTION__, __LINE__);
	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hlds_laser_publisher");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle nh_private("~");

	std::string port = "/dev/ttyS1";
	int baud_rate = 230400;
	std::string frame_id = "laser";
	boost::asio::io_service io;
	priv_nh.param("port", port, std::string("/dev/ttyS1"));
	priv_nh.param("baud_rate", baud_rate, 230400);
	priv_nh.param("frame_id", frame_id, std::string("laser"));
	nh_private.param<int>("laserGen",laserGen,1);

//Init for laser instance
	try
	{
		if (laserGen == 1)
			laser = new LFCDLaserFirstGen(port, baud_rate, io);
		else if (laserGen == 2)
			laser = new LFCDLaserSecondGen(port, baud_rate, io);
		else
			return -1;
	}
	catch (boost::system::system_error ex)
	{
		ROS_ERROR("An exception was thrown: %s", ex.what());
		return -1;
	}

	pthread_t scan_ctrl_thread_id;
	int8_t tmp_ret = pthread_create(&scan_ctrl_thread_id, NULL, scan_ctrl_routine, NULL);
	while (tmp_ret != 0)
	{
		ROS_ERROR("%s %d: Create scan_ctrl_thread failed, retry!", __FUNCTION__, __LINE__);
		usleep(50000);
		tmp_ret = pthread_create(&scan_ctrl_thread_id, NULL, scan_ctrl_routine, NULL);
	}

	ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms", 1000);
	laser->point_marker_pub = n.advertise<visualization_msgs::Marker>("lidarPoint", 1000);
	laser->scan_original_pub = n.advertise<sensor_msgs::LaserScan>("scanOriginal", 1000);
	laser->scan_linear_pub = n.advertise<sensor_msgs::LaserScan>("scanLinear", 1000);
	laser->scan_compensate_pub = n.advertise<sensor_msgs::LaserScan>("scanCompensate", 1000);
	auto motor_service = n.advertiseService("lidar_motor_ctrl", lidar_motor_ctrl);

	auto scan_ctrl_sub = n.subscribe("/scan_ctrl", 1, &scan_ctrl_cb);
	auto odom_sub = n.subscribe("/odom", 1, &odom_cb);
	ROS_WARN("lidar_motor_ctrl service is up.");

	nh_private.param<double>("LIDAR_OFFSET_X", laser->LIDAR_OFFSET_X, 0);
	nh_private.param<double>("LIDAR_OFFSET_Y", laser->LIDAR_OFFSET_Y, 0);
	nh_private.param<double>("LIDAR_OFFSET_THETA", laser->LIDAR_OFFSET_THETA, 0);
	nh_private.param<int>("delay_when_republish", laser->delay_when_republish, 1);
#if LIDAR_BLOCK_RANGE_ENABLE
	nh_private.param<int>("block_angle_1", laser->block_angle_1, -1);
	nh_private.param<int>("block_angle_2", laser->block_angle_2, -1);
	nh_private.param<int>("block_angle_3", laser->block_angle_3, -1);
	nh_private.param<int>("block_range",   laser->block_range, 15);
#endif

	auto tmp_theta = laser->LIDAR_OFFSET_THETA * M_PI / 180.0;
	laser->t_lidar_baselink <<	cos(tmp_theta), -sin(tmp_theta), laser->LIDAR_OFFSET_X,
															sin(tmp_theta), cos(tmp_theta),  laser->LIDAR_OFFSET_Y,
															0, 0, 1;
	std_msgs::UInt16 rpm;
	try
	{
		while (ros::ok())
		{
			if (laser->lidar_status == OFF)
			{
				usleep(20000);
				// printf("%s %d: continue.\n", __FUNCTION__, __LINE__);
				continue;
			}
			sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
			scan->header.frame_id = frame_id;
			laser->poll(scan);
			scan->header.stamp = ros::Time::now();

			rpm.data = laser->rpms;
			motor_pub.publish(rpm);
			laser->scan_original_pub.publish(scan);
			laser->delay_pub(&laser->scan_linear_pub, scan);
		}
		laser->closeLoop();
		return 0;
	}
	catch (boost::system::system_error ex)
	{
		ROS_ERROR("An exception was thrown: %s", ex.what());
		return -1;
	}
}
