//
// Created by pierre on 18-8-23.
//

#ifndef HLS_LFCD_LDS_DRIVER_SECONDGENLASER_HPP
#define HLS_LFCD_LDS_DRIVER_SECONDGENLASER_HPP

#include "Device.h"
class LFCDLaserSecondGen : public Device
{
public:
	LFCDLaserSecondGen(boost::asio::serial_port *serial)
	{
		serial_ = serial;
		baud_rate_ = 115200;
		serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	};

	~LFCDLaserSecondGen();

	virtual void poll(sensor_msgs::LaserScan::Ptr scan);
};
#endif //HLS_LFCD_LDS_DRIVER_SECONDGENLASER_HPP
