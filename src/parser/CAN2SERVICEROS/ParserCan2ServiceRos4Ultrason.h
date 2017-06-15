#ifndef PARSERCAN2SERVICEROS4ULTRASON_H
#define PARSERCAN2SERVICEROS4ULTRASON_H

#include <ros/ros.h>

#include "../../lib.h"


#include "lib/IParserCan2ServiceRos.h"

#include "ibot_can_interface/ultrasonic.h"

class ParserCan2ServiceRos4Ultrason : public IParserCan2ServiceRos<ibot_can_interface::ultrasonic::Request,ibot_can_interface::ultrasonic::Response>
{
public:
  ParserCan2ServiceRos4Ultrason(IT_CAN* it_can, ros::NodeHandle& node, const char* name, const canid_t id_request_can, const canid_t id_response_can);
  virtual ~ParserCan2ServiceRos4Ultrason();

  // IParserCan2ServiceRos interface
protected:
  /**
   * @brief parse_response Parser to convert can_frame to ROS Service Response for ultrasonic sensor
   * @param resp the can_frame
   * @return the response data like a range in [m]
   */
  virtual ibot_can_interface::ultrasonic::Response                     parse_response(const can_frame &resp);


};

#endif // PARSERCAN2SERVICEROS4ULTRASON_H
