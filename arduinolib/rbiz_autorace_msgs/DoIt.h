#ifndef _ROS_rbiz_autorace_msgs_DoIt_h
#define _ROS_rbiz_autorace_msgs_DoIt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rbiz_autorace_msgs
{

  class DoIt : public ros::Msg
  {
    public:
      typedef uint8_t _doIt_type;
      _doIt_type doIt;
      enum { DO_IT_VALUE =  0 };

    DoIt():
      doIt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->doIt >> (8 * 0)) & 0xFF;
      offset += sizeof(this->doIt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->doIt =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->doIt);
     return offset;
    }

    const char * getType(){ return "rbiz_autorace_msgs/DoIt"; };
    const char * getMD5(){ return "98175395676ca2bf6ccd98bb43881af6"; };

  };

}
#endif