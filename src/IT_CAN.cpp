#include "IT_CAN.h"

using namespace std;
using namespace ros;

IT_CAN::IT_CAN()
  : m_recv_thread(NULL), m_running(false)
{
  initialise_can();
}

IT_CAN::~IT_CAN()
{
  stop();
}

void IT_CAN::add_parser(canid_t id, IParserFromCAN *parser)
{
  m_map_parser_can2ros.emplace(pair<canid_t,IParserFromCAN*>(id,parser));
}

char IT_CAN::start()
{
  if(m_recv_thread == NULL)
  {
    m_running = true;
    m_recv_thread = new std::thread(&IT_CAN::recv_thread_func,this);
    ROS_INFO("IT_CAN thread STARTED");
    return 0;
  } else {
    ROS_WARN("IT_CAN thread error while starting (is it started ?)");
    return -1;
  }
}

char IT_CAN::stop()
{
  if(m_recv_thread != NULL)
  {
    m_running = false;
    m_recv_thread->join();
    delete m_recv_thread;
    m_recv_thread = NULL;
    ROS_INFO("IT_CAN thread STOPPED");
    return 0;
  } else {
    ROS_WARN("IT_CAN thread error while stopping (is it stopped ?)");
    return -1;
  }
}

char IT_CAN::send_frame(const can_frame &frame)
{
  int ret;

  m_mutex_send.lock();

  while( (ret = send(m_can_socket, &frame, sizeof(frame), 0)) != sizeof(frame))
  {
    if (ret < 0) {
      if (errno != ENOBUFS) {
        //perror("send failed");
        ROS_ERROR("send failed");
        m_mutex_send.unlock();
        return -1;
      } else {
        //printf("N");
        ROS_WARN("send failed");
        //return -2;
        //fflush(stdout);
      }
    } else {
      //fprintf(stderr, "send returned %d", ret);
      ROS_ERROR("send returned %d", ret);
      m_mutex_send.unlock();
      return -3;
    }
  }
  ROS_INFO("Commande ID 0x%x sent",frame.can_id);
  m_mutex_send.unlock();
  return 0;
}

void IT_CAN::recv_thread_func()
{
  //int ret;
  struct can_frame frame;

  while(ros::ok() && m_running)
  {
    if(recv_frame(&frame)){
      fprintf(stderr, "\nError while receiving message!\n\n");
      //ROS_ERROR("Error while receiving message!");
      continue;
    }

    auto it = m_map_parser_can2ros.find(frame.can_id);
    if(it != m_map_parser_can2ros.end())
    {
      it->second->process(frame);
    } else {
      ROS_WARN("Unrecognized frame with id %X with %d data",
               frame.can_id,frame.can_dlc);
    }
  }

}


//        ______________________
//        ::: Tool Functions :::

void IT_CAN::initialise_can()
{
  ros::NodeHandle tmp_node;
  std::string port_name;
  if(!tmp_node.getParam("can_port_name",port_name))
    port_name = CAN_DEFAULT_PORT_NAME;

  if ((m_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("socket");
    ROS_ERROR("IT_CAN failed to open socket");
    exit(EXIT_FAILURE);
  }

  m_can_addr.can_family = AF_CAN;

  strcpy(m_can_ifr.ifr_name, port_name.c_str());
  if (ioctl(m_can_socket, SIOCGIFINDEX, &m_can_ifr) < 0) {
    perror("SIOCGIFINDEX");
    ROS_ERROR("IT_CAN failed to SIOCGIFINDEX");
    exit(EXIT_FAILURE);
  }
  m_can_addr.can_ifindex = m_can_ifr.ifr_ifindex;

  if (bind(m_can_socket, (struct sockaddr *)&m_can_addr, sizeof(m_can_addr)) < 0) {
    perror("bind");
    ROS_ERROR("IT_CAN failed to bind socket");
    exit(EXIT_FAILURE);
  }
}

char IT_CAN::recv_frame(can_frame *frame)
{
  int ret;

  ret = recv(m_can_socket, frame, sizeof(*frame), 0);
  if (ret != sizeof(*frame)) {
    if (ret < 0) {
      perror("recv failed");
      //ROS_WARN("recv failed");
    } else {
      fprintf(stderr, "recv returned %d", ret);
    }
    return -1;
  }
  return 0;
}
