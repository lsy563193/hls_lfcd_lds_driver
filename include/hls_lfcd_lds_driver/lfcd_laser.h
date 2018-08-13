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
#ifndef LFCD_LASER_H
#define LFCD_LASER_H
#define LIDAR_BLOCK_RANGE_ENABLE (1)
#define DEG2RAD(x) ((x)*M_PI / 180.)
#define ON true
#define OFF false

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <boost/thread/mutex.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef struct
{
	double x;
	double y;
} Double_Point;
namespace hls_lfcd_lds
{
class LFCDLaser{
public:
	LFCDLaser(){

	}
	~LFCDLaser(){
	}
		/**
	* @brief Close the driver down and prevent the polling loop from advancing
	*/
	virtual void closeLoop() { shutting_down_ = true; }
		/**
	* @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
	* @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
	*/
	virtual void poll(sensor_msgs::LaserScan::Ptr scan) = 0;
	virtual void startMotor() = 0;
	virtual void stopMotor() = 0;
	virtual void read_false();
	virtual bool read_success();
	virtual void readWithTimeout
						(boost::asio::serial_port& s, const boost::asio::mutable_buffers_1 & buffers, const boost::asio::deadline_timer::duration_type& expiry_time);
	virtual int read_line(int fd, char *buf);
	virtual bool lidar_pm_gpio(char cmd);
	virtual void lidarDataFilter(const sensor_msgs::LaserScan lidarScanData, double delta);
	virtual void publish_scan_compensate(Eigen::MatrixXd lidar_matrix);
	virtual void update_compensate_lidarMatrix(sensor_msgs::LaserScan lidarScanData_);
	virtual void delay_pub(ros::Publisher *pub_linear, sensor_msgs::LaserScan::Ptr scan_msg);
	virtual bool checkFresh(int type, int time);
	virtual int range_360(int x);
	virtual bool check_within_range(int i, int start, int range);
	virtual void pubPointMarker(std::vector<Double_Point> *point);

	uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an LFCD packet
	std::string port_; ///< @brief The serial port the driver is attached to
	uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
	bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
	uint16_t motor_speed_; ///< @brief current motor speed as reported by the LFCD.

	double lidar_stuck_time = 0;
	int lidar_stuck_count = 0;
	bool skip_scan = false;
	int delay;
	int delay_when_republish = 1;
	bool first_power_on{false};
	bool restart_flag{false};
	bool lidar_status = OFF;
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

	Eigen::MatrixXd lidar_matrix = Eigen::Matrix3d::Identity();
	double now_x, now_y, now_yaw;
	Eigen::Matrix3d t_now = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d t_last = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d t_lidar_baselink = Eigen::Matrix3d::Identity();

	boost::mutex scanXY_mutex_;
	ros::Publisher point_marker_pub;
	ros::Publisher scan_linear_pub;
	ros::Publisher scan_original_pub;
	ros::Publisher scan_compensate_pub;
};
class LFCDLaserFirstGen : public LFCDLaser
{
public:
	/**
	* @brief Constructs a new LFCDLaser attached to the given serial port
	* @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
	* @param baud_rate The baud rate to open the serial port at.
	* @param io Boost ASIO IO Service to use when creating the serial port object
	*/
	LFCDLaserFirstGen(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);
	~LFCDLaserFirstGen();

	virtual void poll(sensor_msgs::LaserScan::Ptr scan);
	virtual void startMotor(){ boost::asio::write(serial_, boost::asio::buffer("b", 1));}
	virtual void stopMotor(){ boost::asio::write(serial_, boost::asio::buffer("e", 1));}

private:
	boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
};

class LFCDLaserSecondGen: public LFCDLaser{
public:
	/**
	* @brief Constructs a new LFCDLaser attached to the given serial port
	* @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
	* @param baud_rate The baud rate to open the serial port at.
	* @param io Boost ASIO IO Service to use when creating the serial port object
	*/
	LFCDLaserSecondGen(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);
	~LFCDLaserSecondGen();
	virtual void poll(sensor_msgs::LaserScan::Ptr scan);
	virtual void startMotor(){ boost::asio::write(serial_, boost::asio::buffer("b", 1));}
	virtual void stopMotor(){ boost::asio::write(serial_, boost::asio::buffer("e", 1));}

private:
	boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
};
}
#endif
