#ifndef IPARSERCAN2SERVICEROS_H
#define IPARSERCAN2SERVICEROS_H

#include <ros/ros.h>

#include <mutex>
#include <condition_variable>

#include "../../../IT_CAN.h"
#include "../../../lib.h"

#include "../../CAN2ROS/lib/IParserFromCAN.h"
#include "../../CAN2ROS/lib/IParserCan2Ros.h"


template <class T_req, class T_resp>
class IParserCan2ServiceRos : public IParserFromCAN
{
public:
  IParserCan2ServiceRos(IT_CAN* it_can, ros::NodeHandle& node, const char* name, const canid_t id_request_can,
                        const canid_t id_response_can, const unsigned int timeout_ms = 1000);

  /**
   * @brief last_resp getter function to get the last response obtained :
   * @return the last value
   */
  T_resp last_resp() const;

  /**
   * @brief id_request_can  getter function
   * @return the can_id of the request CAN frame
   */
  canid_t id_request_can() const;

  /**
   * @brief id_response_can  getter function
   * @return the can_id of the response CAN frame
   */
  canid_t id_response_can() const;

protected:
  can_frame                     m_can_frame_request;

  /**
   * @brief parse_response Abstract parser response CAN -> ROS
   * @param resp the response CAN frame
   * @return the response object for ROS
   */
  virtual T_resp                parse_response(const can_frame& resp) = 0;

private:
  /**
   * @brief m_interface_can the pointer to the CAN interface
   */
  IT_CAN*                       m_interface_can;

  /**
   * @brief m_id_request_can the request CAN id (with RTR)
   */
  canid_t                 m_id_request_can;

  /**
   * @brief m_id_response_can the response CAN id
   */
  canid_t                 m_id_response_can;

  /**
   * @brief m_service_server the ROS service server,
   * manage listening request from ROS and send response to ROS
   */
  ros::ServiceServer            m_service_server;

  /**
   * @brief m_last_resp local copy of the response after parsing CAN -> ROS,
   * sent by request's function after waiting response
   */
  T_resp                        m_last_resp;

  //  _____________________________________________________
  //  ::: Synchro between ROS Service and CAN Interface :::

  /**
   * @brief m_mutex local mutex for waiting
   */
  mutable std::mutex            m_mutex;

  /**
   * @brief m_cond_var to manage wait and notify_all functions
   */
  std::condition_variable       m_cond_var;

  /**
   * @brief m_unique_lock object to share mutex
   */
  std::unique_lock<std::mutex>  m_unique_lock;

  /**
   * @brief m_timeout_ms timeout in milliseconds for waiting data response frame :
   */
  const unsigned int            m_timeout_ms;

  /**
   * @brief request function passed to ServiceServer like callback
   * to manage sending CAN Request and receiving CAN Response
   * @param req the ROS request
   * @return the ROS response
   */
  bool                        request(T_req& req, T_resp& resp);

  // IParserFromCAN interface
  /**
   * @brief process function called by IParserFromCAN to convert CAN frame to ROS object
   * @param frame
   */
  virtual void                  process(const can_frame &frame);
};

#include "IParserCan2ServiceRos.tpp"

#endif // IPARSERCAN2SERVICEROS_H
