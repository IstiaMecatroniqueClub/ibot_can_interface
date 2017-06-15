#include "IParserCan2Ros.h"

using namespace std;
using namespace ros;

template<class T_ros>
IParserCan2Ros<T_ros>::IParserCan2Ros(NodeHandle &node, const char *name, const unsigned int size)
  : IParserFromCAN(), m_publisher(node.advertise<T_ros>(name,size))
{

}

template<class T_ros>
IParserCan2Ros<T_ros>::~IParserCan2Ros()
{

}

template<class T_ros>
const char *IParserCan2Ros<T_ros>::topic_name() const
{
  return m_publisher.getTopic().c_str();
}

template<class T_ros>
void IParserCan2Ros<T_ros>::process(const can_frame &frame)
{
  T_ros data;
  bool sending = parse(frame, &data);
  // If sending == true the ros object must be send
  if(sending)
    m_publisher.publish(data);
}
