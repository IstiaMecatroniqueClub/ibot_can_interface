#include "ParserRos2Can4RequestSpeed.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;

ParserRos2Can4RequestSpeed::ParserRos2Can4RequestSpeed(IT_CAN *it_can, const canid_t can_id, NodeHandle &node, const char *name,
                                                       const size_t size, const double isLeft, const float vel_max_mm_s)
  : IParserRos2Can<Twist>(it_can,node,name,size), m_can_id(can_id), m_is_left(isLeft),
    m_max_velocity_subscriber_mm_per_s(node.subscribe("ibot_can_max_velocity_mm_s",10,&ParserRos2Can4RequestSpeed::velocity_mm_per_s_callback,this)),
    m_max_velocity_mm_per_s(vel_max_mm_s)
{

}

ParserRos2Can4RequestSpeed::~ParserRos2Can4RequestSpeed()
{

}

can_frame ParserRos2Can4RequestSpeed::parse(const geometry_msgs::Twist &data)
{
  // Conversion de mm_per_second to mrad_per_second :

  ROS_INFO("New COMMANDE !!!");

  float fspeed;
//  fspeed = data.linear.x + data.angular.z;
  if(m_is_left)
    fspeed = data.linear.x - data.angular.z;
  else
    fspeed = data.linear.x + data.angular.z;

  fspeed *= m_max_velocity_mm_per_s;

  int16_t speed = (int16_t)(fspeed * COEF_MM_PER_S_TO_MRAD_PER_SEC);

  uint8_t a_speed[3];
  speed < 0 ? a_speed[0] = 1 : a_speed[0] = 0;
  speed = abs(speed);
  a_speed[1] = (uint8_t)(speed >> 8);
  a_speed[2] = (uint8_t)(speed);

  ROS_INFO("Create Data frame with id 0x%x and 2 data %d %d",m_can_id,a_speed[0],a_speed[1]);

  return generateDataFrame(m_can_id,3,a_speed);
}

float ParserRos2Can4RequestSpeed::max_velocity_mm_per_s() const
{
  return m_max_velocity_mm_per_s;
}

void ParserRos2Can4RequestSpeed::setMax_velocity_mm_per_s(float max_velocity_mm_per_s)
{
  m_max_velocity_mm_per_s = std::min(500.0f,max_velocity_mm_per_s);
}

void ParserRos2Can4RequestSpeed::velocity_mm_per_s_callback(std_msgs::Float32 max_velocity_mm_per_s)
{
  setMax_velocity_mm_per_s(max_velocity_mm_per_s.data);
  ROS_INFO("Max Velocity changed to %2.2f mm per second",m_max_velocity_mm_per_s);
}

