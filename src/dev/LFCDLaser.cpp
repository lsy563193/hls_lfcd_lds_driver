// // Created by pierre on 18-8-8. //
#include "hls_lfcd_lds_driver/lfcd_laser.h"
extern int laserGen;
namespace hls_lfcd_lds{
void LFCDLaser::readWithTimeout
					(boost::asio::serial_port& s, const boost::asio::mutable_buffers_1 & buffers, const boost::asio::deadline_timer::duration_type& expiry_time)
{
	boost::optional<boost::system::error_code> timer_result;
	boost::asio::deadline_timer timer(s.get_io_service());
	timer.expires_from_now(expiry_time);
	timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });

	boost::optional<boost::system::error_code> read_result;
	boost::asio::async_read(s, buffers, [&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });

	s.get_io_service().reset();
	while (s.get_io_service().run_one())
	{
		if (read_result)
			timer.cancel();
		else if (timer_result)
			s.cancel();
	}

	if (*read_result)
		throw boost::system::system_error(*read_result);
}

bool LFCDLaser::lidar_pm_gpio(char cmd)
{
	if(cmd == '0' && laserGen == 2)
		stopMotor();

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
	fsync(fd);
	close(fd);
	if(cmd == '1' && laserGen == 2)
		startMotor();
	ROS_WARN("hls_lfcd_lds_driver/node.cpp. %s %d: Response: '%s'.", __FUNCTION__, __LINE__, r_buf);
	return r_buf[25] == r_val;
}
int  LFCDLaser::read_line(int fd, char *buf)
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
void LFCDLaser::read_false()
{
	int stuck_time_tolerance = lidar_stuck_count + 1;
	// If lidar is physically stopped, try to restart it.
	//			ROS_INFO("lidar_status(%d) motor_start_flag(%d)\n", lidar_status, motor_start_flag);
	if (first_power_on || stuck_time_tolerance > 6)
		stuck_time_tolerance = 6;

	if (lidar_status)
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
bool LFCDLaser::read_success()
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
void LFCDLaser::lidarDataFilter(const sensor_msgs::LaserScan lidarScanData, double delta)
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
			} else
			{
				isNoiseNow = false;
			}
		} else if (i == 359)
		{
			if (fabs(lidarScanData.ranges[359] - lidarScanData.ranges[358]) > delta ||
					fabs(lidarScanData.ranges[359] - lidarScanData.ranges[0]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[358]:%f,lidarDataNoise[359]:%f,lidarDataNoise[0]:%f",lidarScanData.ranges[358],lidarScanData.ranges[359],lidarScanData.ranges[0]);
				isNoiseNow = true;
			} else
			{
				isNoiseNow = false;
			}
		} else
		{
			if (fabs(lidarScanData.ranges[i + 1] - lidarScanData.ranges[i]) > delta ||
					fabs(lidarScanData.ranges[i - 1] - lidarScanData.ranges[i]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[%d]:%f,lidarDataNoise[%d]:%f,lidarDataNoise[%d]:%f",i-1,lidarScanData.ranges[i-1],i,lidarScanData.ranges[i],i+1,lidarScanData.ranges[i+1]);
				isNoiseNow = true;
			} else
			{
				isNoiseNow = false;
			}
		}
		if (isNoiseLast == true)
			noiseNum.push_back(i - 1);
		isNoiseLast = isNoiseNow;
	}
}

void LFCDLaser::publish_scan_compensate(Eigen::MatrixXd lidar_matrix)
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
	} else
	{
		scan_msg.angle_min = M_PI - angle_min;
		scan_msg.angle_max = M_PI - angle_max;
	}
	scan_msg.angle_increment =
						(scan_msg.angle_max - scan_msg.angle_min) / (double) (360 - 1);
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
				} else
				{ // third block
					angle = 179.0 + fabs(angle);
				}
			} else
			{
				if (x >= 0)
				{ //second block
					angle = 179.0 - fabs(angle);
				} else
				{ //forth block
					angle = 359.0 - fabs(angle);
				}
			}
			int theta = (int) round(angle);
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

void LFCDLaser::update_compensate_lidarMatrix(sensor_msgs::LaserScan lidarScanData_)
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

void LFCDLaser::delay_pub(ros::Publisher *pub_linear, sensor_msgs::LaserScan::Ptr scan_msg)
{
//	ROS_INFO("%s %d: Published scan.", __FUNCTION__, __LINE__);
	if (skip_scan)
	{
		skip_scan = false;
		delay = 0;
	} else
	{
		if (delay > delay_when_republish)
		{
//			ROS_INFO("Published scan2.");
			pub_linear->publish(*scan_msg);
			if (checkFresh(1, 2))
				update_compensate_lidarMatrix(*scan_msg);
			scan_update_time = ros::Time::now().toSec();
		} else
		{
			if (delay <= delay_when_republish)
				delay++;
		}
	}
}

bool LFCDLaser::checkFresh(int type, int time)
{
	bool ret = false;
	if (type == 1) // odom update
	{
		ret = ros::Time::now().toSec() - odom_update_time < time;
	} else if (type == 2) // scan update
	{
		ret = ros::Time::now().toSec() - scan_update_time < time;
	} else
	{
		printf("input error,dont have this type");
	}
	return ret;
}

#if LIDAR_BLOCK_RANGE_ENABLE
int  LFCDLaser::range_360(int x)
{
	while (x < 0)
		x += 360;
	while (x >= 360)
		x -= 360;
	return x;
}

bool LFCDLaser::check_within_range(int i, int start, int range)
{
	if (range == 0)
		return false;

	bool ret = false;
	if (start + range < 360)
	{
		if (i >= start && i <= start + range)
			ret = true;
	} else
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

void LFCDLaser::pubPointMarker(std::vector<Double_Point> *point)
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
	} else
	{
		point_marker.points.clear();
		point_marker_pub.publish(point_marker);
	}
}


}

