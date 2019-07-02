#ifndef LFCD_LASER_H
#define LFCD_LASER_H
#define LIDAR_BLOCK_RANGE_ENABLE (1)
#define ON true
#define OFF false
#define RAD2DEG(x) ((x)*180/M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)

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

class Device
{
public:
	Device()
	{
	}
	~Device()
	{
		stopMotor();
		lidarPmGpio('0');
	}
	/**
	* @brief Close the driver down and prevent the polling loop from advancing
	*/
	virtual void setPollShutDown(bool val)
	{
		shutting_down_ = val;
	}
	/**
	* @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
	* @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
	*/
	virtual void poll(sensor_msgs::LaserScan::Ptr scan) = 0;
	virtual void startMotor()
	{
		boost::asio::write(*serial_, boost::asio::buffer("b", 1));
	}
	virtual void stopMotor()
	{
		boost::asio::write(*serial_, boost::asio::buffer("e", 1));
	}
	virtual void readFalse();
	virtual bool readSuccess();
	virtual void readWithTimeout
						(boost::asio::serial_port* s, const boost::asio::mutable_buffers_1 & buffers, const boost::asio::deadline_timer::duration_type& expiry_time);
	virtual int readLine(int fd, char *buf);
	virtual bool lidarPmGpio(char cmd);
	virtual void lidarDataFilter(double delta);
	virtual void publishScanCompensate(Eigen::MatrixXd lidar_matrix, double odom_time_stamp);
	virtual void updateCompensateLidarMatrix();
	virtual void delayPub();
	virtual bool checkFresh(int type, int time);
	virtual int convertAngleRange(int x);
	virtual bool checkWithinRange(int i, int start, int range);
	virtual void blockLidarPoint();
	virtual void pubPointMarker(std::vector<Double_Point> *point);
	virtual void checkChangeLidarPower();

	boost::asio::serial_port* serial_{NULL}; ///< @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
	uint16_t rpms_; ///< @brief RPMS derived from the rpm bytes in an LFCD packet
	uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
	bool shutting_down_{true}; ///< @brief Flag for whether the driver is supposed to be shutting down or not uint16_t motor_speed_;

	double lidar_stuck_time_ = 0;
	int lidar_stuck_count_ = 0;
	bool skip_scan_ = false;
	int delay_ = 0;
	int delay_when_republish_ = 1;
	bool first_power_on_{false};
	bool motor_start_flag_{false};
	bool motor_stop_flag_{true};
	bool restart_{false};
	double lidar_to_baselink_transform_rotation_degree{};
	std::vector<int> noiseNum_;
	double scan_update_time_;
	double odom_update_time_;
	int angle_min_{};
	int angle_max_{};

#if LIDAR_BLOCK_RANGE_ENABLE
	int block_angle_1_{};
	int block_angle_2_{};
	int block_angle_3_{};
	int block_angle_4_{};
	int block_angle_5_{};
	int block_angle_6_{};
	int block_range_{};
#endif

	//Robot coordinate(x,y,yaw)
	double now_x_, now_y_, now_yaw_;
	//Store lidar data as Decare coordinate in lidar_matrix_. Which the third dimension is 1.
	Eigen::MatrixXd lidar_matrix_ = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d transform_now_ = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d transform_world_to_baselink_ = Eigen::Matrix3d::Identity();
	// Rotation first,then translation
	Eigen::Matrix3d transform_lidar_baselink_ = Eigen::Matrix3d::Identity();

	boost::mutex scanXY_mutex_;
	ros::Publisher point_marker_pub_;
	ros::Publisher scan_linear_pub_;
	ros::Publisher scan_original_pub_;
	ros::Publisher scan_compensate_pub_;
	sensor_msgs::LaserScan::Ptr p_scan_;
};
#endif
