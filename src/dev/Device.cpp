// // Created by pierre on 18-8-8. //
#include "Device.h"
extern int laserGen;
const std::string laser_calibration_file = "/opt/ros/indigo/share/pp/laser_calibration";

void Device::readWithTimeout(boost::asio::serial_port* s, const boost::asio::mutable_buffers_1 & buffers,
		const boost::asio::deadline_timer::duration_type& expiry_time)
{
	/*int s_ret = 0;
	//memset(t_buf,0,size_of_path);
	fd_set read_fd_set;
	struct timeval timeout;
	timeout.tv_sec = 5;
	timeout.tv_usec = 0;// ms
	size_t length = 0;

	while (1)
	{
		FD_ZERO(&read_fd_set);
		FD_SET(STDIN_FILENO, &read_fd_set);

		s_ret = select(FD_SETSIZE, &read_fd_set, NULL, NULL, &timeout);
		if (s_ret < 0)
		{
			FD_CLR(STDIN_FILENO, &read_fd_set);
			throw "[lds driver] Select error!!!.";
		} else if (s_ret == 0)
		{
			FD_CLR(STDIN_FILENO, &read_fd_set);
			throw "[lds driver] timeout!!!.";
		} else if (s_ret > 0)
		{
			if (FD_ISSET(STDIN_FILENO, &read_fd_set))
			{
				auto tmp_ret = ioctl(STDIN_FILENO, FIONREAD, &length);
				if (tmp_ret == -1)
				{
					FD_CLR(STDIN_FILENO, &read_fd_set);
					throw "[lds driver] ioctl return -1.";
				}
				FD_CLR(STDIN_FILENO, &read_fd_set);
				break;
			}
		}
	}
	boost::optional<boost::system::error_code> read_result;
	boost::asio::async_read(*s, buffers, [&read_result](const boost::system::error_code &error, size_t)
	{
		read_result.reset(error);
	});*/

	boost::optional<boost::system::error_code> timer_result;
	boost::asio::deadline_timer timer(s->get_io_service());
	timer.expires_from_now(expiry_time);
	timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });

	boost::optional<boost::system::error_code> read_result;
	boost::asio::async_read(*s, buffers, [&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });

	s->get_io_service().reset();
	while (s->get_io_service().run_one())
	{
		if (read_result)
			timer.cancel();
		else if (timer_result)
			s->cancel();
	}

	if (shutting_down_)
		throw "[lds driver] Power off lidar when lidar in read.";
	if (*read_result)
		throw boost::system::system_error(*read_result);
}

bool Device::lidarPmGpio(char cmd)
{
	if(cmd == '0' && laserGen == 2)
		stopMotor();

	if (cmd != '1' && cmd != '0')
	{
		ROS_ERROR("[lds driver] %s %d: Wrong param: '%c', default set 1", __FUNCTION__, __LINE__, cmd);
		cmd = '1';
	}

	char w_buf[] = {cmd};
	char r_buf[50] = {0};

	/*		ap6212 : wl power state on
 *		ap6212 : wl power state off
 *		check	25th bit diff in " o 'n' " or "o 'f' f"
 */
	char r_val = (cmd == '1') ? 'n' : 'f';
	ROS_DEBUG("[lds driver] %s %d: Operate on gpio.", __FUNCTION__, __LINE__);
	int fd = open("/proc/driver/wifi-pm/power", O_RDWR);
	if (fd == -1)
	{
		ROS_ERROR("[lds driver] %s %d: Open file failed.", __FUNCTION__, __LINE__);
		return false;
	}
	if (write(fd, w_buf, 1) == -1)
	{
		ROS_ERROR("[lds driver] %s %d: Write file failed.", __FUNCTION__, __LINE__);
		return false;
	}
	usleep(200000);
	if (readLine(fd, r_buf) == -1)
	{
		ROS_ERROR("[lds driver] %s %d: Read file failed.", __FUNCTION__, __LINE__);
		return false;
	}
	fsync(fd);
	close(fd);
	if(cmd == '1' && laserGen == 2)
		startMotor();
	ROS_WARN("[lds driver] %s %d: Command: %c, Response: '%s'.", __FUNCTION__, __LINE__, cmd, r_buf);
	return r_buf[25] == r_val;
}

int  Device::readLine(int fd, char *buf)
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

void Device::checkChangeLidarPower()
{
	if (motor_start_flag_)
	{
		motor_start_flag_ = false;
		int retry_count = 1;
		while (!lidarPmGpio('1') && retry_count < 4)
		{
			ROS_ERROR("[lds driver] %s %d: Power up lidar failed, retry.", __FUNCTION__, __LINE__);
			usleep(800000);
			retry_count++;
		}
		if (retry_count >= 4)
			ROS_ERROR("[lds driver] %s %d: Power up lidar failed.", __FUNCTION__, __LINE__);
		else
		{
			first_power_on_ = true;
			ROS_INFO("[lds driver] %s %d: Power up lidar succeed.", __FUNCTION__, __LINE__);
		}
	}
	if (motor_stop_flag_)
	{
		motor_stop_flag_ = false;
		int retry_count = 1;
		while (!lidarPmGpio('0') && retry_count < 4)
		{
			ROS_ERROR("[lds driver] %s %d: Power down lidar failed, retry.", __FUNCTION__, __LINE__);
			usleep(800000);
			retry_count++;
		}
		if (retry_count >= 4)
			ROS_ERROR("[lds driver] %s %d: Power down lidar failed.", __FUNCTION__, __LINE__);
		else
			ROS_INFO("[lds driver] %s %d: Power down lidar succeed.", __FUNCTION__, __LINE__);
	}
}

void Device::readFalse()
{
	int stuck_time_tolerance = lidar_stuck_count_ + 1;
	// If lidar is physically stopped, try to restart it.
	if (first_power_on_ || stuck_time_tolerance > 6)
		stuck_time_tolerance = 6;

	if (!shutting_down_)
	{
		if (lidar_stuck_time_ == 0)
			lidar_stuck_time_ = ros::Time::now().toSec();
		auto diff_time = ros::Time::now().toSec() - lidar_stuck_time_;
		// ROS_WARN("[lds driver] %s %d: grabScanData no ok for %.2fs, tolerance %ds.",
		//  __FUNCTION__, __LINE__,  diff_time, stuck_time_tolerance);
		ROS_WARN("[lds driver] %s %d: Stuck timeout(%.2lf,%d).", __FUNCTION__, __LINE__, diff_time,
				stuck_time_tolerance);
		if (diff_time > stuck_time_tolerance) //time tolerance for seconds
		{
			lidar_stuck_time_ = 0;
			lidar_stuck_count_++;
			int retry_count = 1;
			while(!lidarPmGpio('0') && retry_count < 4)
			{
				ROS_ERROR("[lds driver] %s %d: Power down lidar failed, retry.", __FUNCTION__, __LINE__);
				usleep(800000);
				retry_count++;
			}
			// In case power up and down too fast.
			usleep(800000);
			retry_count = 0;
			while(!lidarPmGpio('1') && retry_count < 4)
			{
				ROS_ERROR("[lds driver] %s %d: Power up lidar failed, retry.", __FUNCTION__, __LINE__);
				usleep(800000);
				retry_count++;
			}
			restart_ = true;
		}
	}
}

bool Device::readSuccess()
{
	if (first_power_on_)
	{
		first_power_on_ = false;
		ROS_WARN("[lds driver] %s %d: First time read data succeeded.", __FUNCTION__, __LINE__);
	}
	if (restart_)
	{
		restart_ = false;
		ROS_WARN("[lds driver] %s %d: Restart read data succeeded.", __FUNCTION__, __LINE__);
	}
	lidar_stuck_time_ = 0;
	lidar_stuck_count_ = 0;
}

void Device::lidarDataFilter(double delta)
{
	noiseNum_.clear();
	bool isNoiseNow = false;
	bool isNoiseLast = false;
	for (int i = 0; i < p_scan_->ranges.size(); i++)
	{
		if (i == 0)
		{
			if (fabs(p_scan_->ranges[p_scan_->ranges.size() - 1] - p_scan_->ranges[0]) > delta ||
					fabs(p_scan_->ranges[1] - p_scan_->ranges[0]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[359]:%f,lidarDataNoise[0]:%f,lidarDataNoise[1]:%f",p_scan_->ranges[359],p_scan_->ranges[0],p_scan_->ranges[1]);
				isNoiseNow = true;
			} else
			{
				isNoiseNow = false;
			}
		} else if (i == 359)
		{
			if (fabs(p_scan_->ranges[p_scan_->ranges.size() - 1] - p_scan_->ranges[p_scan_->ranges.size() - 2]) > delta ||
					fabs(p_scan_->ranges[p_scan_->ranges.size() - 1] - p_scan_->ranges[0]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[358]:%f,lidarDataNoise[359]:%f,lidarDataNoise[0]:%f",p_scan_->ranges[358],p_scan_->ranges[359],p_scan_->ranges[0]);
				isNoiseNow = true;
			} else
			{
				isNoiseNow = false;
			}
		} else
		{
			if (fabs(p_scan_->ranges[i + 1] - p_scan_->ranges[i]) > delta ||
					fabs(p_scan_->ranges[i - 1] - p_scan_->ranges[i]) > delta)
			{
				//				ROS_WARN("lidarDataNoise[%d]:%f,lidarDataNoise[%d]:%f,lidarDataNoise[%d]:%f",i-1,p_scan_->ranges[i-1],i,p_scan_->ranges[i],i+1,p_scan_->ranges[i+1]);
				isNoiseNow = true;
			} else
			{
				isNoiseNow = false;
			}
		}
		if (isNoiseLast == true)
			noiseNum_.push_back(i - 1);
		isNoiseLast = isNoiseNow;
	}
}

void Device::publishScanCompensate(Eigen::MatrixXd lidar_matrix, double odom_time_stamp)
{
	sensor_msgs::LaserScan scan_msg;
//	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.stamp = ros::Time(odom_time_stamp);
	scan_msg.header.frame_id = p_scan_->header.frame_id;
	scan_msg.angle_increment = p_scan_->angle_increment;
	scan_msg.angle_min = p_scan_->angle_min;
	scan_msg.angle_max = p_scan_->angle_max;
	scan_msg.range_min = p_scan_->range_min;
	scan_msg.range_max = p_scan_->range_max;
	scan_msg.intensities.resize(lidar_matrix.cols());
	scan_msg.ranges.resize(lidar_matrix.cols());
	for (int i = 0; i < lidar_matrix.cols(); i++)
	{
		double x = lidar_matrix(0, i);
		double y = lidar_matrix(1, i);
		float distance = static_cast<float>(sqrt(x * x + y * y));
		if (distance < 10)
		{
			double angle = RAD2DEG(atan(y / x));
			if (x > 0 && y > 0)
			{
				angle = fabs(angle) - 1.0;
			}
			else if (x < 0 && y > 0)
			{
				angle = 179 - fabs(angle);
			}
			else if (x < 0 && y < 0)
			{
				angle = 179 + fabs(angle);
			}
			else
			{
				angle = 359 - fabs(angle);
			}
			int theta = (int) round(angle);

			if (theta == (-1))
				scan_msg.ranges[0] = distance;
			else
			{
				if (theta < 0 || theta > 359)
				{
					ROS_ERROR("%s %d, Warning! Vector index is exceed! Index:%d", __FUNCTION__, __LINE__, theta);
					return;
				}
				scan_msg.ranges[theta] = distance;
				//				ROS_ERROR("diff:%lf,scan:%lf,lidar:%lf", scan_msg.ranges[theta] - lidarScanData_.ranges[theta], scan_msg.ranges[theta],lidarScanData_.ranges[theta]);
			}
		}
	}
	for (int i = 0; i < scan_msg.ranges.size(); i++)
	{
		if (scan_msg.ranges[i] == 0 || scan_msg.ranges[i] > 10)
		{
			scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
		}
	}
	scan_compensate_pub_.publish(scan_msg);
//	ROS_INFO("%s %d: Publish compensate at %f.", __FUNCTION__, __LINE__, scan_msg.header.stamp.toSec());
}

void Device::updateCompensateLidarMatrix()
{
	scanXY_mutex_.lock();
	Eigen::Vector3d coordinate = Eigen::Vector3d::Zero();
	lidar_matrix_.resize(3, p_scan_->ranges.size());
//	lidarDataFilter(0.02);
	for (int i = 0; i < p_scan_->ranges.size(); i++)
	{
		coordinate(0) = std::cos(i * p_scan_->angle_increment) * p_scan_->ranges[i];
		coordinate(1) = std::sin(i * p_scan_->angle_increment) * p_scan_->ranges[i];
		coordinate(2) = 1.0;
		lidar_matrix_.col(i) = coordinate;
	}

	transform_world_to_baselink_ << cos(now_yaw_), sin(now_yaw_), -now_y_ * sin(now_yaw_) - now_x_ * cos(now_yaw_),
						-sin(now_yaw_), cos(now_yaw_), now_x_ * sin(now_yaw_) - now_y_ * cos(now_yaw_),
						0, 0, 1;

//	publishScanCompensate(lidar_matrix_);
	lidar_matrix_ = transform_world_to_baselink_.inverse() * transform_lidar_baselink_ * lidar_matrix_; // in base_link coordinate
	//publish marker
/*	std::vector<Double_Point> points_vec;
	Double_Point point;
	std::vector<int>::const_iterator ite = noiseNum_.begin();
	for (int i = 0; i < lidar_matrix_.cols(); i++)
	{
		if (ite != noiseNum_.end() && i == *ite)
		{
			ite++;
			continue;
		}
		point.x = lidar_matrix_(0, i);
		point.y = lidar_matrix_(1, i);
		if (fabs(point.x) < 20 && fabs(point.y) < 20)
			points_vec.push_back(point);
	}
	pubPointMarker(&points_vec);*/
	scanXY_mutex_.unlock();
}

void Device::delayPub()
{
	if (skip_scan_)
	{
		skip_scan_ = false;
		delay_ = 0;
	}
	else
	{
		if (delay_ > delay_when_republish_)
		{
			scan_linear_pub_.publish(*p_scan_);
			updateCompensateLidarMatrix();
			scan_update_time_ = ros::Time::now().toSec();
		}
		else
		{
			if (delay_ <= delay_when_republish_)
				delay_++;
		}
	}
}

bool Device::checkFresh(int type, int time)
{
	bool ret = false;
	if (type == 1) // odom update
	{
		ret = ros::Time::now().toSec() - odom_update_time_ < time;
	}
	else if (type == 2) // scan update
	{
		ret = ros::Time::now().toSec() - scan_update_time_ < time;
	}
	else
	{
		printf("input error,dont have this type");
	}
	return ret;
}

#if LIDAR_BLOCK_RANGE_ENABLE
int Device::convertAngleRange(int x)
{
	while (x < 0)
		x += 360;
	while (x >= 360)
		x -= 360;
	return x;
}

bool Device::checkWithinRange(int i, int start, int range)
{
	if (range == 0 || (start == -1))
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
			if (i == convertAngleRange(start + j))
			{
				ret = true;
				break;
			}
		}
	}
	return ret;
}

void Device::blockLidarPoint()
{
/*	int n_first, n_second, n_third;
	int size;
	ros::NodeHandle n;
	n.getParam("n_first", n_first);
	n.getParam("n_second", n_second);
	n.getParam("n_third", n_third);
	n.getParam("size", size);*/
	for (int i = 0; i < p_scan_->ranges.size(); i++)
	{
		// j stands for ranges[i] is at j degree direction of robot.
		auto j = static_cast<int>(std::round(RAD2DEG(p_scan_->angle_min) + i * RAD2DEG(p_scan_->angle_increment) +
					lidar_to_baselink_transform_rotation_degree));

		j = convertAngleRange(j);
		if (checkWithinRange(j,block_angle_1_,block_range_) ||
			checkWithinRange(j,block_angle_2_,block_range_) ||
			checkWithinRange(j,block_angle_3_,block_range_) ||
			checkWithinRange(j,block_angle_4_,block_range_) ||
			checkWithinRange(j,block_angle_5_,block_range_) ||
			checkWithinRange(j,block_angle_6_,block_range_))
		{
			p_scan_->ranges[i] = std::numeric_limits<float>::infinity();
//			printf("i: %d, j: %d\n", i, j);
		}
	}
}
#endif
void Device::pubPointMarker(std::vector<Double_Point> *point)
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
	point_marker.header.frame_id = "base_link";
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
		point_marker_pub_.publish(point_marker);
		point_marker.points.clear();
	} else
	{
		point_marker.points.clear();
		point_marker_pub_.publish(point_marker);
	}
}

