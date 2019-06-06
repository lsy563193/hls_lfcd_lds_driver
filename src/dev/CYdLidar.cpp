#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <angles.h>

using namespace std;
using namespace ydlidar;
using namespace impl;
using namespace angles;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(const string& port, const int& angle_min, const int& angle_max): lidarPtr(nullptr)
{
	m_SerialPort = "";
	m_SerialBaudrate = 115200;
	m_FixedResolution = false;
	m_AutoReconnect = true;
	m_MaxAngle = angle_max;
	m_MinAngle = angle_min;
	m_MaxRange = 8.0;
	m_MinRange = 0.08;
	isScanning = false;
	node_counts = 360;
	each_angle = 0.5;
	m_AbnormalCheckCount = 2;
	m_ScanFrequency = 8.0;
	m_IgnoreArray.clear();
//	lidarPmGpio('0');
//	usleep(100000);
	lidarPmGpio('1');
	usleep(100000);

	setSerialPort(port);
	setSerialBaudrate(115200);
	setMaxRange(8.0);
	setMinRange(0.08);
	setMaxAngle(angle_max);
	setMinAngle(angle_min);
	setFixedResolution(true);
	setAutoReconnect(true);
	if (initialize())
	{
		printf("%s %d initalize succeed\n",__FUNCTION__, __LINE__);
		turnOn();
	}
	else
		printf("%s %d initalize failed\n",__FUNCTION__, __LINE__);
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(1000 / m_ScanFrequency);
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);
  size_t all_nodes_counts = node_counts;

  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    op_result = lidarPtr->ascendScanData(nodes, count);
    tim_scan_start = nodes[0].stamp;
    tim_scan_end   = nodes[count - 1].stamp;;

    if (IS_OK(op_result) && count > 1) {
      if (!m_FixedResolution) {
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;
      }

      each_angle = 360.0 / all_nodes_counts;
      node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
      memset(angle_compensate_nodes, 0, all_nodes_counts * sizeof(node_info));
      unsigned int i = 0;

      for (; i < count; i++) {
        if (nodes[i].distance_q2 != 0) {
          float angle = (float)((nodes[i].angle_q6_checkbit >>
                                 LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
          int inter = (int)(angle / each_angle);
          float angle_pre = angle - inter * each_angle;
          float angle_next = (inter + 1) * each_angle - angle;

          if (angle_pre < angle_next) {
            if (inter < all_nodes_counts) {
              angle_compensate_nodes[inter] = nodes[i];
            }
          } else {
            if (inter < all_nodes_counts - 1) {
              angle_compensate_nodes[inter + 1] = nodes[i];
            }
          }
        }

        if (nodes[i].stamp < tim_scan_start) {
          tim_scan_start = nodes[i].stamp;
        }

        if (nodes[i].stamp > tim_scan_end) {
          tim_scan_end = nodes[i].stamp;
        }
      }


      LaserScan scan_msg;

      if (m_MaxAngle < m_MinAngle) {
        float temp = m_MinAngle;
        m_MinAngle = m_MaxAngle;
        m_MaxAngle = temp;
      }


      int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
      int angle_start = 180 + m_MinAngle;
      int node_start = all_nodes_counts * (angle_start / 360.0f);

      scan_msg.ranges.resize(counts, 0.0);
      scan_msg.intensities.resize(counts, 0.0);
      float range = 0.0;
      float intensity = 0.0;
      int index = 0;

      for (size_t i = 0; i < all_nodes_counts; i++) {
        range = (float)angle_compensate_nodes[i].distance_q2 / 4.f / 1000;
        intensity = (float)(angle_compensate_nodes[i].sync_quality);

        if (i < all_nodes_counts / 2) {
          index = all_nodes_counts / 2 - 1 - i;
        } else {
          index = all_nodes_counts - 1 - (i - all_nodes_counts / 2);
        }

        if (m_IgnoreArray.size() != 0) {
          float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >>
                                 LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          if (angle > 180) {
            angle = 360 - angle;
          } else {
            angle = -angle;
          }

          for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
            if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j + 1])) {
              range = 0.0;
              break;
            }
          }
        }

        if (range > m_MaxRange || range < m_MinRange) {
          range = std::numeric_limits<float>::infinity();
        }

        int pos = index - node_start ;

        if (0 <= pos && pos < counts) {
          scan_msg.ranges[pos] =  range;
          scan_msg.intensities[pos] = intensity;
        }
      }




      double scan_time = (tim_scan_end - tim_scan_start);
      scan_time /= 1e9;
      scan_msg.system_time_stamp = tim_scan_start;
      scan_msg.self_time_stamp = tim_scan_start;
      scan_msg.config.min_angle = from_degrees(m_MinAngle);
      scan_msg.config.max_angle = from_degrees(m_MaxAngle);

      if (scan_msg.config.max_angle - scan_msg.config.min_angle == 2 * M_PI) {
        scan_msg.config.ang_increment = (scan_msg.config.max_angle -
                                         scan_msg.config.min_angle) /
                                        (double)counts;
        scan_msg.config.time_increment = scan_time / (double)counts;
      } else {
        scan_msg.config.ang_increment = (scan_msg.config.max_angle -
                                         scan_msg.config.min_angle) /
                                        (double)(counts - 1);
        scan_msg.config.time_increment = scan_time / (double)(counts - 1);
      }

      scan_msg.config.scan_time = scan_time;
      scan_msg.config.min_range = m_MinRange;
      scan_msg.config.max_range = m_MaxRange;
      outscan = scan_msg;
      delete[] angle_compensate_nodes;
      return true;


    }

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  fprintf(stdout, "TurnOn........\n");
  fflush(stdout);

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {
  node_info nodes[2048];
  size_t   count = _countof(nodes);
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    op_result =  lidarPtr->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
      return false;
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}


/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!checkCOMMs()) {
    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr)
    return false;

  if (isScanning && lidarPtr->isscanning())
    return true;

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  if (!checkCOMMs()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
    fflush(stderr);
    return false;
  }

  return true;
}

void CYdLidar::poll(sensor_msgs::LaserScan::Ptr scan_msg)
{
	bool hardError;
	LaserScan scan;
	doProcessSimple(scan, hardError);
	ros::Time start_scan_time;
	start_scan_time.sec = scan.system_time_stamp / 1000000000ul;
	start_scan_time.nsec = scan.system_time_stamp % 1000000000ul;
	scan_msg->header.stamp = start_scan_time;
	scan_msg->angle_min = scan.config.min_angle;
	scan_msg->angle_max = scan.config.max_angle;
	scan_msg->angle_increment = scan.config.ang_increment;
	scan_msg->scan_time = scan.config.scan_time;
	scan_msg->time_increment = scan.config.time_increment;
	scan_msg->range_min = scan.config.min_range;
	scan_msg->range_max = scan.config.max_range;

	scan_msg->ranges = scan.ranges;
	scan_msg->intensities = scan.intensities;
//	printf("min: %.3f, max: %.3f, angle_increment: %.4f, size: %d\n", scan_msg->angle_min, scan_msg->angle_max,
//		   scan_msg->angle_increment, scan_msg->ranges.size());
}
