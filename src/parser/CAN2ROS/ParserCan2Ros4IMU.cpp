#include "ParserCan2Ros4IMU.h"

using namespace std;
using namespace sensor_msgs;

ParserCan2Ros4IMU::ParserCan2Ros4IMU(NodeHandle &node, const char *name, const unsigned int size, IT_CAN *it_can, const size_t count_delay)
  : IParserCan2Ros<Imu>(node,name,size), m_init(true), m_data_type(0),
    m_it_can(it_can), m_count(0), m_count_delay(count_delay)
{
  std::string can_id_blink;
  ros::NodeHandle tmp_node;
  if(!tmp_node.getParam("can_imu_blink_id",can_id_blink))
    can_id_blink = std::to_string(CAN_ID_IMU_TEST);
  m_check_can_frame = generateRemoteFrame(stoi(can_id_blink,0,16));
}

ParserCan2Ros4IMU::~ParserCan2Ros4IMU()
{

}

bool ParserCan2Ros4IMU::parse(const can_frame &frame, Imu *ros_val)
{
  static uint8_t error = 0; // Initialized with no_error;

  /**
    Check if the data packet size is correct :
  **/
  if(frame.can_dlc != 5)
  {
    // Error : size frame not equal to normal size :
    ROS_WARN("IMU Frame %s error of size %d",topic_name(),frame.can_dlc);
    error = 1;
    return false;
  }

  /**
   * @brief loc_data_type the data type of this frame (for checking with local state)
   */
  uint8_t loc_data_type = frame.data[0];
  //ROS_INFO("Frame IMU id %d / Correct is %d",loc_data_type,m_data_type);

  // Waiting first frame :
  /**
    If the parser is not initialized, we must wait a frame with datatype equal to 0
  **/
  if(m_init)
  {
    if(loc_data_type != 0)  return false;
    else {
      m_init = false;
      ROS_INFO("Frame IMU id %d synchronized",frame.can_id);
    }
  }

  // Error 2 processing at t+1 :
  /**
    If when we received the last frame, local and frame datatype is not equal,
    AND if current datatype is 0, then we reinitialize the parser
  **/
  if(error == 2 && loc_data_type == 0)
  {
    // re-Initialize the parser to the first data type :
    m_data_type = 0;
  }

  /**
    Checking if local data type is not equal to the frame data type :
  **/
  if(loc_data_type != m_data_type)
  {
    // Error : frame type (Pitch/Roll/Yaw) is not correct
    ROS_WARN("IMU Frame %s type (Pitch/Roll/Yaw) is not correct (frame %d / correct %d)",
             topic_name(),loc_data_type,m_data_type);
    error = 2;
    return false;
  }

  /**
    Then process the frame value :
  **/
  switch(loc_data_type)
  {
  case 0 :  /// Save value in local copy :
    memcpy(&m_pitch,frame.data+1,sizeof(float));
    m_local_value.header.stamp = ros::Time::now();
    break;
  case 1 :  /// Save value in local copy :
    memcpy(&m_roll,frame.data+1,sizeof(float));
    break;
  case 2 :
    /// Save value in local copy :
    memcpy(&m_yaw,frame.data+1,sizeof(float));

    ROS_INFO("pitch %2.2f roll %2.2f yaw %2.2f",m_pitch,m_roll,m_yaw);

    // Transform Pitch/Roll/Yaw to quaternion and copy it to msg :
    /// Transform euler angles to quaternion :
    double t0 = std::cos(m_yaw * 0.5);
    double t1 = std::sin(m_yaw * 0.5);
    double t2 = std::cos(m_roll * 0.5);
    double t3 = std::sin(m_roll * 0.5);
    double t4 = std::cos(m_pitch * 0.5);
    double t5 = std::sin(m_pitch * 0.5);

    m_local_value.orientation.w = (t0 * t2 * t4 + t1 * t3 * t5);
    m_local_value.orientation.x = (t0 * t3 * t4 - t1 * t2 * t5);
    m_local_value.orientation.y = (t0 * t2 * t5 + t1 * t3 * t4);
    m_local_value.orientation.z = (t1 * t2 * t4 - t0 * t3 * t5);

    break;
  }

  /// Determine the next data type :
  m_data_type = (m_data_type+1) % 3;  // 0 : Pitch / 1 : Roll / 2 : Yaw
  error = 0;

  /// If the blinking debug process is ON :
  if(m_it_can != NULL)
  {
    /// Send RTR frame to the sensor board :
    m_count = (m_count+1)%m_count_delay;
    if(m_count == 0)
    {
      m_it_can->send_frame(m_check_can_frame);
//      can_frame loc_cpy = frame;
//      if (parse_canframe(buff, &loc_cpy)) { // convert a buffer to a frame
//          //fprintf(stderr, "\nWrong CAN-frame format!\n\n");
//          ROS_WARN("IMU_CHECK Wrong CAN-frame format!");
//      } else {
//        m_it_can->send_frame(m_check_can_frame);
//      }
    }
  }

  /// If we have a full object, then we request to send it :
  if(m_data_type == 0){  // If next frame is id_0 then send value now :
    *ros_val = m_local_value;
    return true;
  }else
    return false;
}
