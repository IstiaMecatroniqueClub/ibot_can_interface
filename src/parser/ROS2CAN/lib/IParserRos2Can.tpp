#include "IParserRos2Can.h"

template<class T>
IParserRos2Can<T>::IParserRos2Can(IT_CAN *it_can, ros::NodeHandle &node, const char *name, const size_t size)
  : m_interface_can(it_can),
    m_subscriber(node.subscribe(name,size,&IParserRos2Can<T>::process,this))
{

}

template<class T>
IParserRos2Can<T>::~IParserRos2Can()
{

}

template<class T>
void IParserRos2Can<T>::process(const T &data)
{
  m_interface_can->send_frame(parse(data));
}
