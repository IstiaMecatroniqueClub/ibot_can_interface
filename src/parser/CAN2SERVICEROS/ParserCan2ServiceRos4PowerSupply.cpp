#include "ParserCan2ServiceRos4PowerSupply.h"

ParserCan2ServiceRos4PowerSupply::ParserCan2ServiceRos4PowerSupply(IT_CAN* it_can, ros::NodeHandle& node, const char* name, const canid_t id_request_can, const canid_t id_response_can)
  : IParserCan2ServiceRos<ibot_can_interface::powersupply::Request,ibot_can_interface::powersupply::Response>(it_can,node,name,id_request_can,id_response_can)
{

}

ParserCan2ServiceRos4PowerSupply::~ParserCan2ServiceRos4PowerSupply()
{

}

ibot_can_interface::powersupply::Response ParserCan2ServiceRos4PowerSupply::parse_response(const can_frame &resp)
{
  ibot_can_interface::powersupply::Response ros_resp;

  /// Check if data packet size is OK
  if(resp.can_dlc != 8){
    ROS_WARN("Error in PowerSupply Service with DataSize %d",resp.can_dlc);
    ros_resp.voltage_5v = ros_resp.voltage_12v = ros_resp.voltage_vbat = ros_resp.current_ibat = -1.0f;
    return ros_resp;
  }

  /**
   * @brief v12v, v5v, vbat, ibat are local voltages
   */
  uint16_t v12v, v5v, vbat, ibat;

  /// Copy data from can_frame :
  v12v = (uint16_t)resp.data[0]<<8 | (uint16_t)resp.data[1];
  v5v  = (uint16_t)resp.data[2]<<8 | (uint16_t)resp.data[3];
  vbat = (uint16_t)resp.data[4]<<8 | (uint16_t)resp.data[5];
  ibat = (uint16_t)resp.data[6]<<8 | (uint16_t)resp.data[7];

//  memcpy(&v12v,resp.data,2);
//  memcpy(&v5v,resp.data+2,2);
//  memcpy(&vbat,resp.data+4,2);
//  memcpy(&ibat,resp.data+6,2);

  /// Use coef to convert decimal value to analog value :
  ros_resp.voltage_12v = v12v * CONVERSION_12V_TO_5V;
  ros_resp.voltage_5v = v5v * CONVERSION_5V_TO_5V;
  ros_resp.voltage_vbat = vbat * CONVERSION_VBAT_TO_5V;
  ros_resp.current_ibat = ibat * COEF_IBAT_TO_5V - OFFSET_IBAT_TO_5V;

  return ros_resp;
}
