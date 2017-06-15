#ifndef IT_CAN_H
#define IT_CAN_H

#include <ros/ros.h>

#include <thread>
#include <vector>

#include <errno.h> // for errno, ENOBUFS
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h> // for close()

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <mutex>

#include "define.h"

#include "lib.h"

#include "parser/CAN2ROS/lib/IParserFromCAN.h"

/** Flags for CAN frame
**/
#define     ERROR_FRAME     (1 << 29)
#define     RTR_FRAME       (1 << 30)

/**
 * @brief The IT_CAN class
 * This class is an interface between CAN socket and ROS service/topic.
 * Contains a separated thread to listen the CAN socket and to use parsers to convert this frame into ROS std message.
 */
class IT_CAN
{
public:
  IT_CAN();
  virtual ~IT_CAN();


  void  add_parser(canid_t id, IParserFromCAN* parser);

  /**
   * @brief start to start the internal thread
   * @return 0 if success
   *         -1 if an error occurred
   */
  char  start();

  /**
   * @brief stop to start the internal thread
   * @return 0 if success
   *         1 if an error occurred
   */
  char  stop();

  /**
   * @brief send function to send can_frame using an internal sending buffer
   * @param frame the can_frame
   */
  char  send_frame(const struct can_frame& frame);


protected:


private:
  /**
   * @brief m_recv_thread the internal thread
   */
  std::thread*      m_recv_thread;

  /**
   * @brief m_running boolean representing the internal state of the CAN interface
   */
  bool              m_running;

  /**
   * @brief recv_thread_func the function executed by the previous thread
   */
  void              recv_thread_func();

  mutable std::mutex    m_mutex_send;

  //  _____________________
  //  ::: CAN interface :::
  /**
   * @brief m_can_socket the CAN socket descriptor
   */
  int                   m_can_socket;

  struct sockaddr_can   m_can_addr;
  struct ifreq          m_can_ifr;

  /**
   * @brief initialise_can function for initialize CAN interface
   */
  void                  initialise_can();

  /**
   * @brief recv_frame
   * @param frame
   */
  char recv_frame(can_frame *frame);

  //  ______________________
  //  ::: PARSER CAN2ROS :::
  /// Table to automatically choose the parser by can_id
  std::map<canid_t,IParserFromCAN*> m_map_parser_can2ros;


};

#endif // IT_CAN_H
