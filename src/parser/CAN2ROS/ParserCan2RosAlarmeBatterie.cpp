#include "ParserCan2RosAlarmeBatterie.h"

using namespace ros;
using namespace std_msgs;

ParserCan2RosAlarmeBatterie::ParserCan2RosAlarmeBatterie(NodeHandle &node, const char *name, const unsigned int size, const float analog_coef, const Alarme_Batterie_Config *alarme_config)
  : IParserCan2Ros<Float32>(node,name,size), m_analog_coef(analog_coef), m_alarme_config(alarme_config), m_last_value(-1.0f)
{

}

ParserCan2RosAlarmeBatterie::~ParserCan2RosAlarmeBatterie()
{

}

float ParserCan2RosAlarmeBatterie::last_value() const
{
  return m_last_value;
}

bool ParserCan2RosAlarmeBatterie::parse(const can_frame &frame, std_msgs::Float32 *ros_val)
{
  /**
   * @brief int_val local decimal value
   */
  uint16_t  int_val;
  int_val = (uint16_t)frame.data[0]<<8|(uint16_t)frame.data[1];
  //memcpy(&int_val,frame.data,2);
  /// Convert decimal to [V] value and copy it locally :
  ros_val->data = m_last_value = m_analog_coef * int_val;

  /// If alarm is not configured, then we don't show any ROS Log
  if(m_alarme_config != NULL)
  {
    if(m_last_value < m_alarme_config->warning_voltage)       ROS_WARN("BATTERIE LOW VOLTAGE %2.2fV",m_last_value);
    else if(m_last_value < m_alarme_config->error_voltage)    ROS_ERROR("BATTERIE VERY LOW VOLTAGE %2.2fV",m_last_value);
  }

  /// This message must be sent every time a message is received !
  return true;
}

