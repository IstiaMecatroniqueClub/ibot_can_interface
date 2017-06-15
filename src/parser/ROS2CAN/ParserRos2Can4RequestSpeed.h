#ifndef PARSERROS2CAN4REQUESTSPEED_H
#define PARSERROS2CAN4REQUESTSPEED_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "lib/IParserRos2Can.h"

#define     WHEEL_PERIMETER_MILLIMETER    282.74f
#define     M_RAD_PER_ROTATION            6283.185f
#define     COEF_MM_PER_S_TO_MRAD_PER_SEC M_RAD_PER_ROTATION/WHEEL_PERIMETER_MILLIMETER

class ParserRos2Can4RequestSpeed : public IParserRos2Can<geometry_msgs::Twist>
{
public:
  ParserRos2Can4RequestSpeed(IT_CAN* it_can, const canid_t can_id, ros::NodeHandle& node, const char* name, const size_t size, const double isLeft, const float vel_max_mm_s = 100);
  virtual ~ParserRos2Can4RequestSpeed();

  // IParserRos2Can interface
  float             max_velocity_mm_per_s() const;
  void              setMax_velocity_mm_per_s(float max_velocity_mm_per_s);

protected:
  can_frame         parse(const geometry_msgs::Twist &data);

private:
  const canid_t     m_can_id;
  const bool        m_is_left;

  ros::Subscriber   m_max_velocity_subscriber_mm_per_s;
  float             m_max_velocity_mm_per_s;
  void              velocity_mm_per_s_callback(std_msgs::Float32 max_velocity_mm_per_s);
};

#endif // PARSERROS2CAN4REQUESTSPEED_H
