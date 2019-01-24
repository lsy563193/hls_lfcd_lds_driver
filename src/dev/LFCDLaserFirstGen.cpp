//
// Created by pierre on 18-8-8.
//
#include "FirstGenLaser.hpp"
void LFCDLaserFirstGen::poll(sensor_msgs::LaserScan::Ptr scan)
{
	uint8_t start_count = 0;
	bool got_scan = false;
	boost::array<uint8_t, 2520> raw_bytes;
	uint8_t good_sets = 0;
	uint32_t motor_speed = 0;
	rpms_ = 0;
	int index;

	while (!shutting_down_ && !got_scan)
	{
		// Wait until first data sync of frame: 0xFA, 0xA0
		readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[start_count], 1),boost::posix_time::seconds( 1 ));
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
				readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[2], 2518),boost::posix_time::seconds( 1 ));
				got_scan = true;
				readSuccess();
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
						rpms_ = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

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
							// Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
							// uint16_t intensity = (byte3 << 8) + byte2;
							uint16_t range = (byte3 << 8) + byte2;
							if(range / 1000.0 < scan->range_min || range / 1000.0 > scan->range_max)
							{
								scan->ranges[359-index] = std::numeric_limits<float>::infinity();
								scan->intensities[359-index] = intensity;
								continue;
							}
#if LIDAR_BLOCK_RANGE_ENABLE
							if (block_angle_1_ != -1 && checkWithinRange(index, block_angle_1_, block_range_))
								scan->ranges[359-index] = std::numeric_limits<float>::infinity();
							else if (block_angle_2_ != -1 && checkWithinRange(index, block_angle_2_, block_range_))
								scan->ranges[359-index] = std::numeric_limits<float>::infinity();
							else if (block_angle_3_ != -1 && checkWithinRange(index, block_angle_3_, block_range_))
								scan->ranges[359-index] = std::numeric_limits<float>::infinity();
							else
								scan->ranges[359-index] = range / 1000.0;
#else
							scan->ranges[359-index] = range / 1000.0;
#endif
							scan->intensities[359-index] = intensity;
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
