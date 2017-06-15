#include "ParserCan2ServiceRos4Ultrason.h"

using namespace std;
using namespace ros;
using namespace std_msgs;

ParserCan2ServiceRos4Ultrason::ParserCan2ServiceRos4Ultrason(IT_CAN* it_can, ros::NodeHandle& node, const char* name, const canid_t id_request_can, const canid_t id_response_can)
  : IParserCan2ServiceRos<ibot_can_interface::ultrasonic::Request,ibot_can_interface::ultrasonic::Response>(it_can,node,name,id_request_can,id_response_can)
{
}

ParserCan2ServiceRos4Ultrason::~ParserCan2ServiceRos4Ultrason()
{

}

ibot_can_interface::ultrasonic::Response ParserCan2ServiceRos4Ultrason::parse_response(const can_frame &resp)
{
  ibot_can_interface::ultrasonic::Response ros_resp;
  if(resp.can_dlc != 2)
  {
    ROS_WARN("Ultrason Id %d Error data size %d normarl is 2",resp.can_id,resp.can_dlc);
    ros_resp.range = -1.0f;
    return ros_resp;
  }

  uint16_t    val_mm = 0;
  val_mm = ((uint16_t)resp.data[0] << 8) | (uint16_t)resp.data[1];

  if(val_mm == 1)
  {
    ROS_WARN("Ultrason Id %d Error when sending pulse",resp.can_id);
    ros_resp.range = -2.0f;
    return ros_resp;
  }

  if(val_mm == 2)
  {
    ROS_WARN("Ultrason Id %d Error when waiting returned pulse",resp.can_id);
    ros_resp.range = -3.0f;
    return ros_resp;
  }

  ros_resp.range = (float)val_mm  / 1000.0f;
  return ros_resp;
}
