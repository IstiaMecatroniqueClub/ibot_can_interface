#include "IParserCan2ServiceRos.h"

using namespace std;
using namespace ros;

template<class T_req, class T_resp>
IParserCan2ServiceRos<T_req,T_resp>::IParserCan2ServiceRos(IT_CAN *it_can, NodeHandle &node, const char *name,
                                                           const canid_t id_request_can, const canid_t id_response_can, const unsigned int timeout_ms)
  : IParserFromCAN(),
    m_interface_can(it_can),
    m_unique_lock(unique_lock<std::mutex>(m_mutex)),
    m_id_request_can(id_request_can), m_id_response_can(id_response_can), m_timeout_ms(timeout_ms),
    m_can_frame_request(generateRemoteFrame(id_request_can))
{
  m_service_server = node.advertiseService(name,&IParserCan2ServiceRos<T_req,T_resp>::request,this);
}

template<class T_req, class T_resp>
T_resp IParserCan2ServiceRos<T_req,T_resp>::last_resp() const
{
  return m_last_resp;
}

template<class T_req, class T_resp>
canid_t IParserCan2ServiceRos<T_req,T_resp>::id_request_can() const
{
  return m_id_request_can;
}

template<class T_req, class T_resp>
canid_t IParserCan2ServiceRos<T_req,T_resp>::id_response_can() const
{
  return m_id_response_can;
}

template<class T_req, class T_resp>
bool IParserCan2ServiceRos<T_req,T_resp>::request(T_req &req, T_resp &resp)
{
  // Send CAN frame using CAN Interface :
  while(m_interface_can->send_frame(m_can_frame_request) != 0){
    ROS_WARN("Error when sending request can frame id %d", m_id_request_can);
    usleep(1000);    // If error encountered, just sleep 10ms and retry in infinite loop
  }

  // Wait response on CAN interface :
  if(m_cond_var.wait_for(m_unique_lock, chrono::milliseconds(m_timeout_ms)) == std::cv_status::timeout)
  {
    ROS_WARN("Timeout on request id %d",m_id_request_can);
    m_last_resp = T_resp(); // Reset data and send it ...
  }

  resp = m_last_resp;
  // Return value obtained on the IParserFromCAN parent
  return true;
}

template<class T_req, class T_resp>
void IParserCan2ServiceRos<T_req,T_resp>::process(const can_frame &frame)
{
  // Just parse CAN frame and save it in the local object
  m_last_resp = parse_response(frame);
  // Notify Service function to finish Request function
  m_cond_var.notify_all();
}

