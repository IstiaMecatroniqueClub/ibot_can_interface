#ifndef IPARSERCAN2ROS_H
#define IPARSERCAN2ROS_H

#include <ros/ros.h>
#include <ros/publisher.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "IParserFromCAN.h"


template <class T_ros>
class IParserCan2Ros : public IParserFromCAN
{
public:
  IParserCan2Ros(ros::NodeHandle& node, const char* name, const unsigned int size);
  virtual ~IParserCan2Ros();

  /**
   * @brief topic_name Getter on the topic name
   * @return the name of the topic
   */
  const char*     topic_name() const;

  /**
   * @brief process function to process the can_frame and publish to ROS
   * @param frame the can_frame
   */
  virtual void            process(const struct can_frame& frame);

protected:
  /**
   * @brief parse abstract function to convert can_frame to ROS std message
   * @param frame
   * @return
   */
  virtual bool           parse(const can_frame& frame, T_ros* ros_val)       = 0 ;

private:
  /**
   * @brief m_publisher ROS publisher on topic
   */
  ros::Publisher   m_publisher;
};

#include "IParserCan2Ros.tpp"

#endif // IPARSERCAN2ROS_H
