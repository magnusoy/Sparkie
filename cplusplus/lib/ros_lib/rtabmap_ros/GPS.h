#ifndef _ROS_rtabmap_ros_GPS_h
#define _ROS_rtabmap_ros_GPS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rtabmap_ros
{

  class GPS : public ros::Msg
  {
    public:
      typedef float _stamp_type;
      _stamp_type stamp;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef float _error_type;
      _error_type error;
      typedef float _bearing_type;
      _bearing_type bearing;

    GPS():
      stamp(0),
      longitude(0),
      latitude(0),
      altitude(0),
      error(0),
      bearing(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->stamp);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->error);
      offset += serializeAvrFloat64(outbuffer + offset, this->bearing);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->stamp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bearing));
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/GPS"; };
    const char * getMD5(){ return "0acde0d09a1ab74993cf4e41ff4eae49"; };

  };

}
#endif
