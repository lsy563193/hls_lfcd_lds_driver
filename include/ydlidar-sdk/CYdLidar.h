﻿
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>
#include "Device.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif
#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\

using namespace ydlidar;


class YDLIDAR_API CYdLidar : public Device{
  PropertyBuilderByName(float, MaxRange, private) ///< 设置和获取激光最大测距范围
  PropertyBuilderByName(float, MinRange, private) ///< 设置和获取激光最小测距范围
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度

  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(bool, Reversion, private) ///< 设置和获取是否旋转激光180度
  PropertyBuilderByName(bool, AutoReconnect, private) ///< 设置异常是否自动重新连接

  PropertyBuilderByName(int, SerialBaudrate, private) ///< 设置和获取激光通讯波特率
  PropertyBuilderByName(std::string, SerialPort, private) ///< 设置和获取激光端口号
  PropertyBuilderByName(std::vector<float>, IgnoreArray, private) ///< 设置和获取激光剔除点


 public:
  CYdLidar(const string& port); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.

  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs
  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /** Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  virtual void poll(sensor_msgs::LaserScan::Ptr scan_msg);



 private:
  bool isScanning;
  int node_counts ;
  double each_angle;
  YDlidarDriver *lidarPtr;
};	// End of class

