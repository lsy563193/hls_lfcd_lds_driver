//
// Created by pierre on 18-8-8.
//
#include "FirstGenLaser.hpp"
void LFCDLaserFirstGen::poll(sensor_msgs::LaserScan::Ptr scan)
{
	bool got_scan = false;
	uint16_t package_len = 42;
	uint16_t package_count_for_one_scan = 60;
	const int buffer_size = 2520; // = package_count_for_one_scan * package_len
	boost::array<uint8_t, buffer_size> raw_bytes;
	uint8_t good_sets = 0;
	uint32_t motor_speed = 0;
	rpms_ = 0;
	int index;

	uint8_t start_angle_index = 0xA0;
	uint8_t start_angle_index_offset = 24;
	start_angle_index += start_angle_index_offset;

	uint16_t init_sync_byte_offset = start_angle_index_offset * package_len;
	uint16_t init_check_angle_byte_offset = init_sync_byte_offset + 1;

	uint16_t read_offset = init_sync_byte_offset;

	while (!got_scan)
	{
		// Wait until first data sync of frame: 0xFA, start_angle_index
		readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[read_offset], 1),boost::posix_time::seconds( 1 ));
		if (read_offset == init_sync_byte_offset)
		{
			if (raw_bytes[read_offset] == 0xFA)
			{
//				printf("FA pass %02x\n", raw_bytes[read_offset]);
				read_offset = init_check_angle_byte_offset;
			}
		}
		else if (read_offset == init_check_angle_byte_offset)
		{
			if (raw_bytes[read_offset] == start_angle_index)
			{
//				printf("Angle pass %02x\n", raw_bytes[read_offset]);
				scan->angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
				scan->angle_min = 0.0;
				scan->angle_max = static_cast<float>(2.0 * M_PI - scan->angle_increment);
				scan->range_min = 0.12;
				scan->range_max = 3.5;
				scan->ranges.resize(360);
				scan->intensities.resize(360);

//				readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[2], 2518),boost::posix_time::seconds( 1 ));

				//read data in sets of 6
//				for (uint16_t i = 0; i < raw_bytes.size(); i = static_cast<uint16_t>(i + 42))
				uint16_t read_pack_count = 0;
				for (auto i = init_sync_byte_offset;;)
				{
					readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[i + 2], 40),boost::posix_time::seconds( 1 ));
//					printf("Read at %d\n", i + 2);
					read_pack_count++;
					//Check sum
					uint8_t check_sum = 0;
					for(uint8_t j = 0; j < 40; j++)
						check_sum += raw_bytes[j + 42 * (i / 42)];
					check_sum = static_cast<uint8_t>(0xff - check_sum);

//					if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) //&& CRC check
					if (check_sum == raw_bytes[40 + 42 * (i / 42)] || check_sum == raw_bytes[41 +  42 * (i / 42)])
					{
						// ROS_INFO("// pass CRC check");
						good_sets++;
						motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]; //accumulate count for avg. time increment
						rpms_ = static_cast<uint16_t>((raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10);

						for (auto j = static_cast<uint16_t>(i + 4); j < i + 40; j = static_cast<uint16_t>(j + 6))
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

							if ((359 - index) < 0 || (359 - index) > 359)
							{
								ROS_ERROR("%s %d, Warning! Vector index is exceed! Index:%d", __FUNCTION__, __LINE__, index);
								throw "[lds driver] Warning! Vector index is exceed!";
							}
							if(range / 1000.0 < scan->range_min || range / 1000.0 > scan->range_max)
							{
								scan->ranges[359-index] = std::numeric_limits<float>::infinity();
								scan->intensities[359-index] = intensity;
								continue;
							}
							scan->ranges[359-index] = range / 1000.0;
							scan->intensities[359-index] = intensity;
						}
					}
					else
						throw "Check sum crc error!";

					i += 42;
					if (i >= raw_bytes.size())
						i -= raw_bytes.size();
					if (read_pack_count < package_count_for_one_scan)
					{
						readWithTimeout(serial_, boost::asio::buffer(&raw_bytes[i], 2),
										boost::posix_time::seconds(1));
//						printf("Read finish2[%02x][%02x].\n", raw_bytes[i], raw_bytes[i + 1]);

//						if (!(raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42))) //&& CRC check
//							throw "CRC error!";
					}
					else
						break;
				}
				scan->time_increment = static_cast<float>(motor_speed / good_sets / 1e8);

				got_scan = true;
				readSuccess();
			}
			else
			{
				read_offset = init_sync_byte_offset;
			}
		}

		if (shutting_down_)
			throw "[lds driver] Power off lidar when lidar in poll.";
	}
}
