//
// Created by pierre on 18-8-8.
//
#include "SecondGenLaser.hpp"
void LFCDLaserSecondGen::poll(sensor_msgs::LaserScan::Ptr scan)
{
	uint8_t start_count = 0;
	bool got_scan = false;
	boost::array<uint8_t, 1980> raw_bytes;
	uint8_t good_sets = 0;
	uint32_t motor_speed = 0;
	rpms_ = 0;
	int index;

	while (!got_scan)
	{
		// Wait until first data sync of frame: 0xFA, 0xA0
		readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[start_count], 1), boost::posix_time::seconds( 1 ));
		readSuccess();
		if (start_count == 0)
		{
			if (raw_bytes[start_count] == 0xFA)
			{
				start_count = 1;
			}
		} else if (start_count == 1)
		{
			if (raw_bytes[start_count] == 0xA0)
			{
				start_count = 0;
				// Now that entire start sequence has been found, read in the rest of the message
				readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[2], 1978), boost::posix_time::seconds(1));
				got_scan = true;
				readSuccess();
				scan->angle_min = 0.0;
				scan->angle_increment = (2.0 * M_PI / 360.0);
				scan->angle_max = 2.0 * M_PI - scan->angle_increment;
				scan->range_min = 0.12;
				scan->range_max = 5.0;
				scan->ranges.resize(360);
				scan->intensities.resize(360);

				//read data in sets of 4
				for (uint16_t i = 0; i < raw_bytes.size(); i = i + 22)
				{
					if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22))
					{
						good_sets++;
						motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
						rpms_ = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

						for (uint16_t j = i + 4; j < i + 20; j = j + 4)
						{
							index = 4 * (i / 22) + (j - 4 - i) / 4;

							// Four bytes per reading
							uint8_t byte0 = raw_bytes[j];
							uint8_t byte1 = raw_bytes[j + 1];
							uint8_t byte2 = raw_bytes[j + 2];
							uint8_t byte3 = raw_bytes[j + 3];

							// Remaining bits are the range in mm
							uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
							if ((359 - index) < 0 || (359 - index) > 359)
							{
								ROS_ERROR("%s %d, Warning! Vector index is exceed! Index:%d", __FUNCTION__, __LINE__, index);
								throw "[lds driver] Warning! Vector index is exceed!";
							}
							// Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
							uint16_t intensity = (byte3 << 8) + byte2;
							if (range / 1000.0 < scan->range_min || range / 1000.0 > scan->range_max)
							{
								scan->ranges[359 - index] = std::numeric_limits<float>::infinity();
								scan->intensities[359 - index] = intensity;
								continue;
							}

							scan->ranges[359 - index] = range / 1000.0;
							scan->intensities[359 - index] = intensity;
						}
					}
				}
				scan->time_increment = motor_speed / good_sets / 1e8;
			} else
			{
				start_count = 0;
			}
		}

		if (shutting_down_)
			throw "[lds driver] Power off lidar when lidar in poll.";
	}
}

