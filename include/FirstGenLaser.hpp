//
// Created by pierre on 18-8-23.
//
// This is for LDS 1.5.

#ifndef HLS_LFCD_LDS_DRIVER_FIRSTGENLASER_HPP
#define HLS_LFCD_LDS_DRIVER_FIRSTGENLASER_HPP
#include "Device.h"
class LFCDLaserFirstGen : public Device
{
public:
	LFCDLaserFirstGen(boost::asio::serial_port *serial)
	{
		serial_ = serial;
		baud_rate_ = 230400;
		serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	}

	~LFCDLaserFirstGen()
	{}

	virtual void poll(sensor_msgs::LaserScan::Ptr scan);
};
#endif //HLS_LFCD_LDS_DRIVER_FIRSTGENLASER_HPP
