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
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <hls_lfcd_lds_driver/lfcd_laser.h>
#include <hls_lfcd_lds_driver/SetLidar.h>
#include <hls_lfcd_lds_driver/scan_ctrl.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#define LIDAR_BLOCK_RANGE_ENABLE (1)

//#if LIDAR_BLOCK_RANGE_ENABLE
//#define _FOC (359) //FIRST obstical col
//#define _SOC (119) //SECOND obstical COL
//#define _TOC (239) //THIRD obstical COL

//#endif

#define DEG2RAD(x) ((x)*M_PI / 180.)

static int delay;
static int is_instance{};
static int delay_when_republish = 1;

static bool skip_scan = false;

// Debug
//bool motor_start_flag = true;
bool motor_start_flag = false;
bool motor_stop_flag = false;
bool first_power_on{false};
bool restart_flag{false};

#define ON true
#define OFF false
// debug
//bool lidar_status = ON;
bool lidar_status = OFF;
double lidar_stuck_time = 0;
int lidar_stuck_count = 0;

Eigen::MatrixXd lidar_matrix = Eigen::Matrix3d::Identity();
double now_x, now_y, now_yaw;
Eigen::Matrix3d t_now = Eigen::Matrix3d::Identity();
Eigen::Matrix3d t_last = Eigen::Matrix3d::Identity();
Eigen::Matrix3d t_lidar_baselink = Eigen::Matrix3d::Identity();
boost::mutex scanXY_mutex_;
ros::Publisher point_marker_pub;
ros::Publisher scan_compensate_pub;
double LIDAR_OFFSET_X, LIDAR_OFFSET_Y, LIDAR_OFFSET_THETA; // Rotation first,then translation
std::vector<int> noiseNum;
double scan_update_time;
double odom_update_time;

#if LIDAR_BLOCK_RANGE_ENABLE

int block_angle_1;
int block_angle_2;
int block_angle_3;
int block_range;
#endif

typedef struct
{
	double x;
	double y;
} Double_Point;

int read_line(int fd, char *buf)
{
	char temp;
	bool eof = false;
	int i = 0;
	do
	{
		switch (read(fd, &temp, 1))
		{
		case 0:
			eof = true;
			break;
		case -1:
			return -1;
		default:
		{
			buf[i++] = temp;
			//printf("%c", temp);
			if (temp == '\n' && i > 0)
			{
				buf[i - 1] = ' ';
				eof = true;
			}
		}
		}
	} while (i < 50 && !eof); // It should not be longer than 50, it is protection for dead loop.
	return i;
}

bool checkFresh(int type, int time)
{
	bool ret = false;
	if (type == 1) // odom update
	{
		ret = ros::Time::now().toSec() - odom_update_time < time;
	}
	else if (type == 2) // scan update
	{
		ret = ros::Time::now().toSec() - scan_update_time < time;
	}
	else
	{
		printf("input error,dont have this type");
	}
	return ret;
}
bool lidar_pm_gpio(char cmd)
{

	if (cmd != '1' && cmd != '0')
	{
		ROS_ERROR("hls_lfcd_lds_driver/node.cpp. %s %d: Wrong param: '%c', default set 1", __FUNCTION__, __LINE__, cmd);
		cmd = '1';
	}

	char w_buf[] = {cmd};
	char r_buf[50] = {0};

	/*		ap6212 : wl power state on
 *		ap6212 : wl power state off
 *		check	25th bit diff in " o 'n' " or "o 'f' f"
 */
	char r_val = (cmd == '1') ? 'n' : 'f';
	ROS_DEBUG("hls_lfcd_lds_driver/node.cpp. %s %d: Operate on gpio.", __FUNCTION__, __LINE__);
	int fd = open("/proc/driver/wifi-pm/power", O_RDWR);
	if (fd == -1)
	{
		ROS_ERROR("hls_lfcd_lds_driver/node.cpp. %s %d: Open file failed.", __FUNCTION__, __LINE__);
		return false;
	}
	if (write(fd, w_buf, 1) == -1)
	{
		ROS_ERROR("hls_lfcd_lds_driver/node.cpp. %s %d: Write file failed.", __FUNCTION__, __LINE__);
		return false;
	}
	usleep(200000);
	if (read_line(fd, r_buf) == -1)
	{
		ROS_ERROR("hls_lfcd_lds_driver/node.cpp. %s %d: Read file failed.", __FUNCTION__, __LINE__);
		return false;
	}
	close(fd);
	ROS_WARN("hls_lfcd_lds_driver/node.cpp. %s %d: Response: '%s'.", __FUNCTION__, __LINE__, r_buf);
	return r_buf[25] == r_val;
}

#if LIDAR_BLOCK_RANGE_ENABLE
int range_360(int x)
{
	while (x < 0)
		x += 360;
	while (x >= 360)
		x -= 360;
	return x;
}

bool check_within_range(int i, int start, int range)
{
	if (range == 0)
		return false;

	bool ret = false;
	if (start + range < 360)
	{
		if (i >= start && i <= start + range)
			ret = true;
	}
	else
	{
		for (int j = 0; j <= range; j++)
		{
			if (i == range_360(start + j))
			{
				ret = true;
				break;
			}
		}
	}
	return ret;
}

#endif

void pubPointMarker(std::vector<Double_Point> *point)
{
	visualization_msgs::Marker point_marker;
	geometry_msgs::Point lidar_points_;
	point_marker.ns = "lidarPoint";
	point_marker.id = 0;
	point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	point_marker.action = 0; //add
	point_marker.lifetime = ros::Duration(0);
	point_marker.scale.x = 0.03;
	point_marker.scale.y = 0.03;
	point_marker.scale.z = 0.03;
	point_marker.color.r = 1.0;
	point_marker.color.g = 0.0;
	point_marker.color.b = 0.0;
	point_marker.color.a = 1.0;
	point_marker.header.frame_id = "/base_link";
	point_marker.header.stamp = ros::Time::now();
	lidar_points_.x = 0.0;
	lidar_points_.y = 0.0;
	lidar_points_.z = 0.0;

	if (!(*point).empty())
	{
		for (std::vector<Double_Point>::iterator iter = (*point).begin(); iter != (*point).end(); ++iter)
		{
			lidar_points_.x = iter->x;
			lidar_points_.y = iter->y;
			point_marker.points.push_back(lidar_points_);
		}
		point_marker_pub.publish(point_marker);
		point_marker.points.clear();
	}
	else
	{
		point_marker.points.clear();
		point_marker_pub.publish(point_marker);
	}
}
void lidarDataFilter(const sensor_msgs::LaserScan lidarScanData, double delta)
{
	noiseNum.clear();
	bool isNoiseNow = false;
	bool isNoiseLast = false;
	for (int i = 0; i < 360; i++)
	{
		if (i == 0)
		{
			if (fabs(lidarScanData.ranges[359] - lidarScanData.ranges[0]) > delta ||
				fabs(lidarScanData.ranges[1] - lidarScanData.ranges[0]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[359]:%f,lidarDataNoise[0]:%f,lidarDataNoise[1]:%f",lidarScanData.ranges[359],lidarScanData.ranges[0],lidarScanData.ranges[1]);
				isNoiseNow = true;
			}
			else
			{
				isNoiseNow = false;
			}
		}
		else if (i == 359)
		{
			if (fabs(lidarScanData.ranges[359] - lidarScanData.ranges[358]) > delta ||
				fabs(lidarScanData.ranges[359] - lidarScanData.ranges[0]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[358]:%f,lidarDataNoise[359]:%f,lidarDataNoise[0]:%f",lidarScanData.ranges[358],lidarScanData.ranges[359],lidarScanData.ranges[0]);
				isNoiseNow = true;
			}
			else
			{
				isNoiseNow = false;
			}
		}
		else
		{
			if (fabs(lidarScanData.ranges[i + 1] - lidarScanData.ranges[i]) > delta ||
				fabs(lidarScanData.ranges[i - 1] - lidarScanData.ranges[i]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[%d]:%f,lidarDataNoise[%d]:%f,lidarDataNoise[%d]:%f",i-1,lidarScanData.ranges[i-1],i,lidarScanData.ranges[i],i+1,lidarScanData.ranges[i+1]);
				isNoiseNow = true;
			}
			else
			{
				isNoiseNow = false;
			}
		}
		if (isNoiseLast == true)
			noiseNum.push_back(i - 1);
		isNoiseLast = isNoiseNow;
	}
}
void publish_scan_compensate(Eigen::MatrixXd lidar_matrix)
{
	float angle_min = DEG2RAD(0.0f);
	float angle_max = DEG2RAD(359.0f);
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "laser";
	if (angle_max > angle_min)
	{
		scan_msg.angle_min = M_PI - angle_max;
		scan_msg.angle_max = M_PI - angle_min;
	}
	else
	{
		scan_msg.angle_min = M_PI - angle_min;
		scan_msg.angle_max = M_PI - angle_max;
	}
	scan_msg.angle_increment =
		(scan_msg.angle_max - scan_msg.angle_min) / (double)(360 - 1);
	scan_msg.range_min = 0.15;
	scan_msg.range_max = 3.5;
	scan_msg.intensities.resize(360);
	scan_msg.ranges.resize(360);

	for (int i = 0; i < lidar_matrix.cols(); i++)
	{
		double x = lidar_matrix(0, i);
		double y = lidar_matrix(1, i);
		double distance = sqrt(x * x + y * y);
		if (distance < 10)
		{
			double angle = atan(y / x) * 180.0 / M_PI;
			if (angle > 0)
			{
				if (x <= 0)
				{ //first block
					angle = fabs(angle) - 1.0;
				}
				else
				{ // third block
					angle = 179.0 + fabs(angle);
				}
			}
			else
			{
				if (x >= 0)
				{ //second block
					angle = 179.0 - fabs(angle);
				}
				else
				{ //forth block
					angle = 359.0 - fabs(angle);
				}
			}
			int theta = (int)round(angle);
			if (theta == (-1))
				scan_msg.ranges[0] = distance;
			else
			{
				scan_msg.ranges[theta] = distance;
				//				ROS_ERROR("diff:%lf,scan:%lf,lidar:%lf", scan_msg.ranges[theta] - lidarScanData_.ranges[theta], scan_msg.ranges[theta],lidarScanData_.ranges[theta]);
			}
		}
	}
	for (int i = 0; i < scan_msg.ranges.size(); i++)
	{
		if (scan_msg.ranges[i] == 0 || scan_msg.ranges[i] > 10)
			scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
	}
	// scan_compensate_pub.publish(scan_msg);
}
void update_compensate_lidarMatrix(sensor_msgs::LaserScan lidarScanData_)
{
	scanXY_mutex_.lock();
	double th = 0;
	Eigen::Vector3d coordinate = Eigen::Vector3d::Zero();
	lidar_matrix.resize(3, 360);
	lidarDataFilter(lidarScanData_, 0.02);
	for (int i = 0; i < 360; i++)
	{
		th = i * 1.0 + 181.0;
		coordinate(0) = cos(th * M_PI / 180.0) * lidarScanData_.ranges[i];
		coordinate(1) = sin(th * M_PI / 180.0) * lidarScanData_.ranges[i];
		coordinate(2) = 1.0;
		lidar_matrix.col(i) = coordinate;
	}

	t_last << cos(now_yaw), sin(now_yaw), -now_y * sin(now_yaw) - now_x * cos(now_yaw),
		-sin(now_yaw), cos(now_yaw), now_x * sin(now_yaw) - now_y * cos(now_yaw),
		0, 0, 1;

	publish_scan_compensate(lidar_matrix);
	lidar_matrix = t_lidar_baselink * lidar_matrix; // in base_link coordinate
													//publish marker
	std::vector<Double_Point> points_vec;
	Double_Point point;
	std::vector<int>::const_iterator ite = noiseNum.begin();
	for (int i = 0; i < lidar_matrix.cols(); i++)
	{
		if (ite != noiseNum.end() && i == *ite)
		{
			ite++;
			continue;
		}
		point.x = lidar_matrix(0, i);
		point.y = lidar_matrix(1, i);
		if (fabs(point.x) < 20 && fabs(point.y) < 20)
			points_vec.push_back(point);
	}
	pubPointMarker(&points_vec);
	scanXY_mutex_.unlock();
}

// void publish_scan(ros::Publisher *pub_linear,ros::Publisher *pub_original,
//                   rplidar_response_measurement_node_t *nodes,
//                   size_t node_count, ros::Time start,
//                   double scan_time, bool inverted,
//                   float angle_min, float angle_max,
//                   std::string frame_id)
// {
// 	sensor_msgs::LaserScan scan_msg;

// 	scan_msg.header.stamp = start;
// 	scan_msg.header.frame_id = frame_id;

// 	bool reversed = (angle_max > angle_min);
// 	if ( reversed ) {
// 		scan_msg.angle_min =  M_PI - angle_max;
// 		scan_msg.angle_max =  M_PI - angle_min;
// 	} else {
// 		scan_msg.angle_min =  M_PI - angle_min;
// 		scan_msg.angle_max =  M_PI - angle_max;
// 	}
// 	scan_msg.angle_increment =
// 		(scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

// 	scan_msg.scan_time = scan_time;
// 	scan_msg.time_increment = scan_time / (double)(node_count-1);
// 	scan_msg.range_min = 0.15;
// 	scan_msg.range_max = 3.5;

// 	scan_msg.intensities.resize(node_count);
// 	scan_msg.ranges.resize(node_count);

// 	bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
// 	if (!reverse_data) {
// 		for (size_t i = 0; i < node_count; i++) {
// 			float read_value = (float) nodes[i].distance_q2/4.0f/1000;
// 			if (read_value < 0.12 || read_value > 3.5)
// 				scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
// 			else{
// #if LIDAR_BLOCK_RANGE_ENABLE
// 				if (block_angle_1 != -1 && check_within_range(i, block_angle_1, block_range))
// 					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
// 				else if (block_angle_2 != -1 && check_within_range(i, block_angle_2, block_range))
// 					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
// 				else if (block_angle_3 != -1 && check_within_range(i, block_angle_3, block_range))
// 					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
// 				else
// 					scan_msg.ranges[i] = read_value;
// #else
// 				scan_msg.ranges[i] = read_value;
// #endif
// 			}
// 			scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2);
// 		}
// 	} else {
// 		for (size_t i = 0; i < node_count; i++) {
// 			float read_value = (float)nodes[i].distance_q2/4.0f/1000;
// 			if (read_value < 0.12 || read_value > 3.5)
// 				scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
// 			else{
// #if LIDAR_BLOCK_RANGE_ENABLE
// 				if (block_angle_1 != -1 && check_within_range(i, block_angle_1, block_range))
// 					scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
// 				else if (block_angle_2 != -1 && check_within_range(i, block_angle_2, block_range))
// 					scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
// 				else if (block_angle_3 != -1 && check_within_range(i, block_angle_3, block_range))
// 					scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
// 				else
// 					scan_msg.ranges[node_count-1-i] = read_value;
// #else
// 				scan_msg.ranges[node_count-1-i] = read_value;
// #endif
// 			}
// 			scan_msg.intensities[node_count-1-i] = (float) (nodes[i].sync_quality >> 2);
// 		}
// 	}

// 	pub_original->publish(scan_msg);
// //	ROS_INFO("%s %d: Published scan.", __FUNCTION__, __LINE__);
// 	if (skip_scan)
// 	{
// 		skip_scan = false;
// 		delay = 0;
// 	}
// 	else
// 	{
// 		if (delay > delay_when_republish) {
// 			pub_linear->publish(scan_msg);
// 			if(checkFresh(1,2))
// 				update_compensate_lidarMatrix(scan_msg);
// 			scan_update_time = ros::Time::now().toSec();
// 		}
// 		else
// 		{
// 			if (delay <= delay_when_republish)
// 				delay++;
// 		}
// 	}
// }

bool lidar_motor_ctrl(hls_lfcd_lds_driver::SetLidar::Request &req, hls_lfcd_lds_driver::SetLidar::Response &res)
{
	if (!is_instance)
	{
		res.message = "drive not ready";
		return false;
	}
	if (req.switch_status == true)
	{
		if (lidar_status == OFF)
		{
			motor_start_flag = true;
			motor_stop_flag = false;
		}
	}
	else if (req.switch_status == false)
	{
		motor_stop_flag = true;
		motor_start_flag = false;
	}
	res.message = "Driver ommand received:";
	res.message += (req.switch_status ? "true":"false");
	res.success = true;
	lidar_status = req.switch_status;
	return true;
}

void scan_ctrl_cb(const hls_lfcd_lds_driver::scan_ctrl::ConstPtr &msg)
{
	if (!msg->allow_publishing)
		skip_scan = true;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	//update odom position
	odom_update_time = ros::Time::now().toSec();
	now_x = msg->pose.pose.position.x;
	now_y = msg->pose.pose.position.y;
	now_yaw = tf::getYaw(msg->pose.pose.orientation);
	if (now_yaw < 0)
		now_yaw += 2 * M_PI;

	if (lidar_status && checkFresh(2, 2))
	{
		scanXY_mutex_.lock();
		std::vector<Double_Point> points_vec;
		//compensate ScanData
		t_now << cos(now_yaw), sin(now_yaw), -now_y * sin(now_yaw) - now_x * cos(now_yaw),
			-sin(now_yaw), cos(now_yaw), now_x * sin(now_yaw) - now_y * cos(now_yaw),
			0, 0, 1;
		lidar_matrix = t_now * t_last.inverse() * lidar_matrix; // lidar_matrix in base_link coordinate
		t_last = t_now;
		//publish marker
		Double_Point point;
		std::vector<int>::const_iterator ite = noiseNum.begin();
		for (int i = 0; i < lidar_matrix.cols(); i++)
		{
			if (ite != noiseNum.end() && i == *ite)
			{
				ite++;
				continue;
			}
			point.x = lidar_matrix(0, i);
			point.y = lidar_matrix(1, i);
			if (fabs(point.x) < 4 && fabs(point.y) < 4)
				points_vec.push_back(point);
		}
		//publish scan
		publish_scan_compensate(t_lidar_baselink.inverse() * lidar_matrix);
		pubPointMarker(&points_vec);
		scanXY_mutex_.unlock();
	}
}

void *scan_ctrl_routine(void *)
{
	pthread_detach(pthread_self());
	ROS_INFO("\033[34mhls_lfcd_lds_driver/node.cpp. %s %d: scan_ctrl thread is up.\033[0m", __FUNCTION__, __LINE__);
	ros::spin();
}

void read_false()
{
	int stuck_time_tolerance = lidar_stuck_count + 1;
	// If lidar is physically stopped, try to restart it.
	//			ROS_INFO("lidar_status(%d) motor_start_flag(%d)\n", lidar_status, motor_start_flag);
	if (first_power_on || stuck_time_tolerance > 6)
		stuck_time_tolerance = 6;

	if (lidar_status && !motor_start_flag)
	{
		if (lidar_stuck_time == 0)
			lidar_stuck_time = ros::Time::now().toSec();
		auto diff_time = ros::Time::now().toSec() - lidar_stuck_time;
		// ROS_WARN("hls_lfcd_lds_driver/node.cpp. %s %d: grabScanData no ok for %.2fs, tolerance %ds.",
		//  __FUNCTION__, __LINE__,  diff_time, stuck_time_tolerance);
		ROS_INFO("~~~~~~~~~~~~~~~~~~~~Stuck timeout(%d,%d).", diff_time, stuck_time_tolerance);
		if (diff_time > stuck_time_tolerance) //time tolerance for seconds
		{
			ROS_WARN("Stuck timeout(%d,%d).", diff_time, stuck_time_tolerance);
			lidar_stuck_time = 0;
			lidar_stuck_count++;
			lidar_pm_gpio('0');
			// In case power up and down too fast.
			usleep(800000);
			lidar_pm_gpio('1');
			restart_flag = true;
			ROS_WARN("Restart motor.");
		}
	}
}
bool read_success()
{
	if (first_power_on)
	{
		first_power_on = false;
		ROS_WARN("hls_lfcd_lds_driver/node.cpp. %s %d: grabScanData ok:.", __FUNCTION__, __LINE__);
	}
	if (restart_flag)
	{
		restart_flag = false;
		ROS_WARN("hls_lfcd_lds_driver/node.cpp. %s %d: grabScanData ok:", __FUNCTION__, __LINE__);
	}
	lidar_stuck_time = 0;
	lidar_stuck_count = 0;
}
void delay_pub(ros::Publisher *pub_linear, sensor_msgs::LaserScan::Ptr scan_msg)
{
	ROS_INFO("%s %d: Published scan.", __FUNCTION__, __LINE__);
	if (skip_scan)
	{
		skip_scan = false;
		delay = 0;
	}
	else
	{
		if (delay > delay_when_republish)
		{
			ROS_INFO("Published scan2.");
			pub_linear->publish(*scan_msg);
			if (checkFresh(1, 2))
				update_compensate_lidarMatrix(*scan_msg);
			scan_update_time = ros::Time::now().toSec();
		}
		else
		{
			if (delay <= delay_when_republish)
				delay++;
		}
	}
}

void power_off()
{
	int try_cnt;
	while (!lidar_pm_gpio('0') && try_cnt < 4)
	{
		ROS_ERROR("\033[34mhls_lfcd_lds_driver/node.cpp. %s %d: Power down lidar failed, retry.\033[0m", __FUNCTION__, __LINE__);
		// In case power up and down too fast.
		usleep(800000);
		try_cnt++;
	}
}

namespace hls_lfcd_lds
{
LFCDLaser::LFCDLaser(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io)
	: port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
	serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

	// Below command is not required after firmware upgrade (2017.10)
	// boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}

LFCDLaser::~LFCDLaser()
{
	boost::asio::write(serial_, boost::asio::buffer("e", 1)); // stop motor
															  //   boost::asio::write(serial_, boost::asio::buffer("e", 1));  // start motor
	power_off();
}

void LFCDLaser::poll(sensor_msgs::LaserScan::Ptr scan)
{
	uint8_t temp_char;
	uint8_t start_count = 0;
	bool got_scan = false;
	boost::array<uint8_t, 2520> raw_bytes;
	uint8_t good_sets = 0;
	uint32_t motor_speed = 0;
	rpms = 0;
	int index;

	while (!shutting_down_ && !got_scan)
	{
		// Wait until first data sync of frame: 0xFA, 0xA0
		try
		{
			boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
		}
		catch (boost::system::system_error ex)
		{
			// if(ex == boost::asio::read::eof)
			read_false();
			continue;
		}

		read_success();
		if (start_count == 0)
		{
			if (raw_bytes[start_count] == 0xFA)
			{
				start_count = 1;
			}
		}
		else if (start_count == 1)
		{
			if (raw_bytes[start_count] == 0xA0)
			{
				start_count = 0;

				ROS_INFO("// Now that entire start sequence has been found, read in the rest of the message");
				got_scan = true;
				try
				{
					boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 2518));
				}
				catch (boost::system::system_error ex)
				{
					// if(ex == boost::asio::read::eof)
					read_false();
					continue;
				}
				read_success();
				scan->angle_increment = (2.0 * M_PI / 360.0);
				scan->angle_min = 0.0;
				scan->angle_max = 2.0 * M_PI - scan->angle_increment;
				scan->range_min = 0.12;
				scan->range_max = 3.5;
				scan->ranges.resize(360);
				scan->intensities.resize(360);

				//read data in sets of 6
				for (uint16_t i = 0; i < raw_bytes.size(); i = i + 42)
				{
					if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) //&& CRC check
					{
						// ROS_INFO("// pass CRC check");
						good_sets++;
						motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]; //accumulate count for avg. time increment
						rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

						for (uint16_t j = i + 4; j < i + 40; j = j + 6)
						{
							index = 6 * (i / 42) + (j - 4 - i) / 6;

							// Four bytes per reading
							uint8_t byte0 = raw_bytes[j];
							uint8_t byte1 = raw_bytes[j + 1];
							uint8_t byte2 = raw_bytes[j + 2];
							uint8_t byte3 = raw_bytes[j + 3];

							// Remaining bits are the range in mm
							uint16_t intensity = (byte1 << 8) + byte0;
							int index_convert;
							if (index >= 0 && index < 180)
							{
								index_convert = 179 - index;
							}
							else
							{
								index_convert = 180 - index >= 0 ? 0 : 540 - index;
							}
							// Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
							// uint16_t intensity = (byte3 << 8) + byte2;
							uint16_t range = (byte3 << 8) + byte2;
#if LIDAR_BLOCK_RANGE_ENABLE
							if(range / 1000.0 < scan->range_min || range / 1000.0 > scan->range_max)
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else if (block_angle_1 != -1 && check_within_range(index, block_angle_1, block_range))
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else if (block_angle_2 != -1 && check_within_range(index, block_angle_2, block_range))
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else if (block_angle_3 != -1 && check_within_range(index, block_angle_3, block_range))
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else
								scan->ranges[index_convert] = range / 1000.0;
#else
							// scan->ranges[node_count-1-i] = read_value;
							scan->ranges[index_convert - index] = range / 1000.0;
#endif
							scan->intensities[index_convert] = intensity;
						}
					}
				}

				scan->time_increment = motor_speed / good_sets / 1e8;
			}
			else
			{
				start_count = 0;
			}
		}
	}
}
} // namespace hls_lfcd_lds

void start_lidar()
{
	if (motor_start_flag)
	{
		motor_start_flag = false;
		auto try_cnt = 1;
		while (!lidar_pm_gpio('1') && try_cnt < 4)
		{
			ROS_ERROR("Power up lidar failed, retry");
			lidar_pm_gpio('0');
			// In case power up and down too fast.
			usleep(800000);
			try_cnt++;
		}
		if (try_cnt >= 4)
			ROS_ERROR("Power up lidar failed");
		else
		{
			// drv->startMotor();
			// drv->resumeScan();
			ROS_INFO("Power up lidar succeed.");
			first_power_on = true;
		}
	}
}

void try_power_up()
{
	int try_cnt = 1;
	while (!lidar_pm_gpio('1') && try_cnt < 2)
	{
		ROS_ERROR("Init lidar power failed, retry.");
		lidar_pm_gpio('0');
		// In case power up and down too fast.
		usleep(800000);
		try_cnt++;
	}
}

void pm_try_power_down()
{
	auto try_cnt = 1;
	while (!lidar_pm_gpio('0') && try_cnt < 2)
	{
		ROS_ERROR("Shutdown lidar power failed, retry");
		try_cnt++;
		// In case power up and down too fast.
		usleep(800000);
	}
	//printf("RPLIDAR running on ROS package hls_lfcd_lds_driver\n"
	//       "SDK Version: "RPLIDAR_SDK_VERSION"\n");
}
void pm_try_power_down2()
{
	auto try_cnt = 1;
	while (!lidar_pm_gpio('0') && try_cnt < 4)
	{
		ROS_WARN("Power down lidar failed, retry.");
		// In case power up and down too fast.
		usleep(800000);
		try_cnt++;
	}
	if (try_cnt >= 4)
		ROS_ERROR("Power down lidar failed");
	else
		ROS_INFO("Power down lidar succeed");
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "hlds_laser_publisher");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::string port;
	int baud_rate;
	std::string frame_id;

	std_msgs::UInt16 rpms;

	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
	priv_nh.param("baud_rate", baud_rate, 230400);
	priv_nh.param("frame_id", frame_id, std::string("laser"));

	pthread_t scan_ctrl_thread_id;
	int8_t tmp_ret = pthread_create(&scan_ctrl_thread_id, NULL, scan_ctrl_routine, NULL);
	while (tmp_ret != 0)
	{
		ROS_ERROR("%s %d: Create scan_ctrl_thread failed, retry!", __FUNCTION__, __LINE__);
		usleep(50000);
		tmp_ret = pthread_create(&scan_ctrl_thread_id, NULL, scan_ctrl_routine, NULL);
	}

	try_power_up();
	bool inverted = false;
	bool angle_compensate = true;

	// ros::Publisher scan_linear_pub = nh.advertise<sensor_msgs::LaserScan>("scanLinear", 1000);
	ros::Publisher scan_original_pub = n.advertise<sensor_msgs::LaserScan>("scanOriginal", 1000);
	scan_compensate_pub = n.advertise<sensor_msgs::LaserScan>("scanCompensate", 1000);
	point_marker_pub = n.advertise<visualization_msgs::Marker>("lidarPoint", 1000);
	ros::NodeHandle nh_private("~");
	nh_private.param<bool>("inverted", inverted, false);
	nh_private.param<bool>("angle_compensate", angle_compensate, true);
	nh_private.param<double>("LIDAR_OFFSET_X", LIDAR_OFFSET_X, 0);
	nh_private.param<double>("LIDAR_OFFSET_Y", LIDAR_OFFSET_Y, 0);
	nh_private.param<double>("LIDAR_OFFSET_THETA", LIDAR_OFFSET_THETA, 0);
#if LIDAR_BLOCK_RANGE_ENABLE
	nh_private.param<int>("block_angle_1", block_angle_1, -1);
	nh_private.param<int>("block_angle_2", block_angle_2, -1);
	nh_private.param<int>("block_angle_3", block_angle_3, -1);
	nh_private.param<int>("block_range", block_range, 15);
#endif

	//nh_private.param<bool>("publish_when_turn", publish_when_turn, true);
	nh_private.param<int>("delay_when_republish", delay_when_republish, 1);
	auto scan_ctrl_sub = n.subscribe("/scan_ctrl", 1, &scan_ctrl_cb);
	auto odom_sub = n.subscribe("/odom", 1, &odom_cb);
	auto motor_service = n.advertiseService("lidar_motor_ctrl", lidar_motor_ctrl);
	ROS_WARN("lidar_motor_ctrl service is up.");
	auto tmp_theta = LIDAR_OFFSET_THETA * M_PI / 180.0;
	t_lidar_baselink << cos(tmp_theta), -sin(tmp_theta), LIDAR_OFFSET_X,
		sin(tmp_theta), cos(tmp_theta), LIDAR_OFFSET_Y,
		0, 0, 1;

	pm_try_power_down();

	boost::asio::io_service io;

	try
	{
		hls_lfcd_lds::LFCDLaser laser(port, baud_rate, io);
		is_instance = true;
		ros::Publisher scan_linear_pub = n.advertise<sensor_msgs::LaserScan>("scanLinear", 1000);
		ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms", 1000);

		while (ros::ok())
		{
			if (motor_stop_flag)
			{
				motor_stop_flag = false;
				// drv->pauseScan();
				//			drv->stopMotor();
				pm_try_power_down2();
			}

			start_lidar();

			if (lidar_status == OFF)
			{
				usleep(20000);
							// printf("%s %d: continue.\n", __FUNCTION__, __LINE__);
				continue;
			}

			sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
			scan->header.frame_id = frame_id;
			laser.poll(scan);
			scan->header.stamp = ros::Time::now();
			rpms.data = laser.rpms;

			motor_pub.publish(rpms);
			scan_original_pub.publish(scan);
			delay_pub(&scan_linear_pub, scan);
		}
		laser.close();

		return 0;
	}
	catch (boost::system::system_error ex)
	{
		ROS_ERROR("An exception was thrown: %s", ex.what());
		return -1;
	}
}
