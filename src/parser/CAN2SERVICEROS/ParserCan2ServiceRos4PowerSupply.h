#ifndef PARSERCAN2SERVICEROS4POWERSUPPLY_H
#define PARSERCAN2SERVICEROS4POWERSUPPLY_H

#include <ros/ros.h>
#include "../../lib.h"

#include "lib/IParserCan2ServiceRos.h"
#include "ibot_can_interface/powersupply.h"
#include "../../define.h"

class ParserCan2ServiceRos4PowerSupply : public IParserCan2ServiceRos<ibot_can_interface::powersupply::Request,ibot_can_interface::powersupply::Response>
{
public:
  ParserCan2ServiceRos4PowerSupply(IT_CAN *it_can, NodeHandle &node, const char *name, const canid_t id_request_can, const canid_t id_response_can);
  virtual ~ParserCan2ServiceRos4PowerSupply();

  // IParserCan2ServiceRos interface
protected:
  /**
   * @brief parse_response Parser to convert can_frame to Service::Response
   * @param resp the can_frame
   * @return the ROS response of this service
   */
  ibot_can_interface::powersupply::Response parse_response(const can_frame &resp);
};

#endif // PARSERCAN2SERVICEROS4POWERSUPPLY_H
