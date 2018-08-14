//
// Created by pierre on 18-8-8.
//
#include "hls_lfcd_lds_driver/lfcd_laser.h"
namespace hls_lfcd_lds
{
void LFCDLaserFirstGen::poll(sensor_msgs::LaserScan::Ptr scan)
{
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
			readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[start_count], 1),boost::posix_time::seconds( 1 ));
		}
		catch (boost::system::system_error ex)
		{
			ROS_ERROR("%d,An exception was thrown: %s", __LINE__,ex.what());
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
				try
				{
					readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[2], 2518),boost::posix_time::seconds( 1 ));
				}
				catch (boost::system::system_error ex)
				{
					ROS_ERROR("%d,An exception was thrown: %s", __LINE__,ex.what());
					read_false();
					start_count = 0;
					continue;
				}
				got_scan = true;
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
							if(range / 1000.0 < scan->range_min || range / 1000.0 > scan->range_max)
							{
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
								scan->intensities[index_convert] = intensity;
								continue;
							}
#if LIDAR_BLOCK_RANGE_ENABLE
							if (block_angle_1 != -1 && check_within_range(index, block_angle_1, block_range))
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else if (block_angle_2 != -1 && check_within_range(index, block_angle_2, block_range))
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else if (block_angle_3 != -1 && check_within_range(index, block_angle_3, block_range))
								scan->ranges[index_convert] = std::numeric_limits<float>::infinity();
							else
								scan->ranges[index_convert] = range / 1000.0;
#else
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
}
