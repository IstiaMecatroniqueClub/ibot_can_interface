#ifndef PARSERCAN2ROS4IMU_H
#define PARSERCAN2ROS4IMU_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib/IParserCan2Ros.h"
#include "../../IT_CAN.h"
#include "../../lib.h"

#include "../../define.h"

class ParserCan2Ros4IMU : public IParserCan2Ros<sensor_msgs::Imu>
{
public:
  ParserCan2Ros4IMU(ros::NodeHandle& node, const char* name = "ibot_can_imu", const unsigned int size = 100,
                    IT_CAN* it_can=NULL, const size_t count_delay=100);
  virtual ~ParserCan2Ros4IMU();

  // IParserCan2Ros interface
protected:
  /**
   * @brief parse can frame to ros message
   * @param frame the can frame
   * @param ros_val the ros msg data
   * @return bool if data must be sent on ros topic or not
   */
  virtual bool parse(const can_frame &frame, sensor_msgs::Imu *ros_val);

private:
  /**
   * @brief m_init boolean to know if the parser is synchronized with sensor
   */
  bool                m_init;
  /**
   * @brief m_data_type is the next data (0-1-2 from roll, pitch, yaw)
   */
  uint8_t             m_data_type;
  /**
   * @brief m_local_value local ros message data to save value before sending (quaternion)
   */
  sensor_msgs::Imu    m_local_value;
  /**
   * @brief m_pitch, m_roll, m_yaw, euler angles in radians
   */
  float               m_pitch, m_roll, m_yaw;

  /**
   * @brief m_it_can local access for sending RTR frame to blink the sensor board
   */
  IT_CAN*             m_it_can;
  /**
   * @brief m_check_can_frame prepared RTR frame for blinking
   */
  can_frame           m_check_can_frame;
  /**
   * @brief m_count_delay const number of packets between blinking
   */
  const size_t        m_count_delay;
  /**
   * @brief m_count current number of packets
   */
  size_t              m_count;



};

#endif // PARSERCAN2ROS4IMU_H
