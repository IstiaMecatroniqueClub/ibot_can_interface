#ifndef IPARSERFROMCAN_H
#define IPARSERFROMCAN_H

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class IParserFromCAN
{
public:
  IParserFromCAN();
  virtual ~IParserFromCAN();

  /**
   * @brief process function to process the can_frame
   * @param frame the can_frame
   */
  virtual void process(const struct can_frame& frame)     = 0;
};

#endif // IPARSERFROMCAN_H
