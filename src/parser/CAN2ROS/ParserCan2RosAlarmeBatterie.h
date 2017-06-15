#ifndef PARSERCAN2ROSALARMEBATTERIE_H
#define PARSERCAN2ROSALARMEBATTERIE_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib/IParserCan2Ros.h"


struct Alarme_Batterie_Config {
public:
  Alarme_Batterie_Config(float warn=14.4f, float error=14.0f) : warning_voltage(warn), error_voltage(error) { }
  float warning_voltage;
  float error_voltage;
};

class ParserCan2RosAlarmeBatterie : public IParserCan2Ros<std_msgs::Float32>
{
public:
  ParserCan2RosAlarmeBatterie(NodeHandle &node, const char *name, const unsigned int size, const float analog_coef, const Alarme_Batterie_Config* alarme_config=NULL);
  virtual ~ParserCan2RosAlarmeBatterie();

  /**
   * @brief last_value simple getter to get the last Battery voltage :
   * @return
   */
  float last_value() const;

  // IParserCan2Ros interface
protected:
  /**
   * @brief parse Parser to convert CAN frame to Float32 ROS message
   * @param frame the CAN frame
   * @param ros_val the Float32 ROS message representing the battery voltage
   * @return true if the data must be sent
   */
  bool parse(const can_frame &frame, std_msgs::Float32 *ros_val);

private:
  /**
   * @brief m_analog_coef the coef to convert decimal value obtained by
   * Ananlog to Digital converter and resistors bridge to voltage in V
   */
  const float                     m_analog_coef;
  /**
   * @brief m_alarme_config const config representing the Warning and the Error threshold
   */
  const Alarme_Batterie_Config*   m_alarme_config;
  /**
   * @brief m_last_value the last voltage obtained
   */
  float                           m_last_value;

};

#endif // PARSERCAN2ROSALARMEBATTERIE_H
