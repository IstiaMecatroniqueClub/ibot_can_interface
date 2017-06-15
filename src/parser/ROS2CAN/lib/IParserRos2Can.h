#ifndef IPARSERROS2CAN_H
#define IPARSERROS2CAN_H

#include <ros/ros.h>

#include "../../../IT_CAN.h"

template <class T>
class IParserRos2Can
{
public:
  IParserRos2Can(IT_CAN* it_can, ros::NodeHandle& node, const char* name, const size_t size);
  virtual ~IParserRos2Can();


protected:
  /**
   * @brief parse Parser to convert ROS data message to can_frame
   * @param data  the ROS data
   * @return  the can_frame
   */
  virtual can_frame       parse(const T& data) = 0;

private:
  /**
   * @brief m_interface_can Local access to the CAN Interface (for sending data)
   */
  IT_CAN*                 m_interface_can;
  /**
   * @brief m_subscriber ROS Data Receiver (manage callback)
   */
  ros::Subscriber         m_subscriber;

  /**
   * @brief process local callback function to process ROS message from Topic
   * @param data
   */
  void                    process(const T& data);
};

#include "IParserRos2Can.tpp"

#endif // IPARSERROS2CAN_H
