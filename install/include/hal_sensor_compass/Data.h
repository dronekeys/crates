/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/asymingt/Workspace/Source/crates/src/hal_sensor_compass/msg/Data.msg
 *
 */


#ifndef HAL_SENSOR_COMPASS_MESSAGE_DATA_H
#define HAL_SENSOR_COMPASS_MESSAGE_DATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hal_sensor_compass
{
template <class ContainerAllocator>
struct Data_
{
  typedef Data_<ContainerAllocator> Type;

  Data_()
    : t(0.0)
    , m_x(0.0)
    , m_y(0.0)
    , m_z(0.0)  {
    }
  Data_(const ContainerAllocator& _alloc)
    : t(0.0)
    , m_x(0.0)
    , m_y(0.0)
    , m_z(0.0)  {
    }



   typedef double _t_type;
  _t_type t;

   typedef double _m_x_type;
  _m_x_type m_x;

   typedef double _m_y_type;
  _m_y_type m_y;

   typedef double _m_z_type;
  _m_z_type m_z;




  typedef boost::shared_ptr< ::hal_sensor_compass::Data_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hal_sensor_compass::Data_<ContainerAllocator> const> ConstPtr;

}; // struct Data_

typedef ::hal_sensor_compass::Data_<std::allocator<void> > Data;

typedef boost::shared_ptr< ::hal_sensor_compass::Data > DataPtr;
typedef boost::shared_ptr< ::hal_sensor_compass::Data const> DataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hal_sensor_compass::Data_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hal_sensor_compass::Data_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hal_sensor_compass

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'hal_sensor_compass': ['/home/asymingt/Workspace/Source/crates/src/hal_sensor_compass/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hal_sensor_compass::Data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hal_sensor_compass::Data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hal_sensor_compass::Data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hal_sensor_compass::Data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hal_sensor_compass::Data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hal_sensor_compass::Data_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hal_sensor_compass::Data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "453eabca5f989f35ac0e6c6a1047b129";
  }

  static const char* value(const ::hal_sensor_compass::Data_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x453eabca5f989f35ULL;
  static const uint64_t static_value2 = 0xac0e6c6a1047b129ULL;
};

template<class ContainerAllocator>
struct DataType< ::hal_sensor_compass::Data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hal_sensor_compass/Data";
  }

  static const char* value(const ::hal_sensor_compass::Data_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hal_sensor_compass::Data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 t     # Time at which measurement was taken\n\
float64 m_x     # Body frame magnetic field strength X\n\
float64 m_y     # Body frame magnetic field strength Y\n\
float64 m_z     # Body frame magnetic field strength Z\n\
";
  }

  static const char* value(const ::hal_sensor_compass::Data_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hal_sensor_compass::Data_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.t);
      stream.next(m.m_x);
      stream.next(m.m_y);
      stream.next(m.m_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Data_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hal_sensor_compass::Data_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hal_sensor_compass::Data_<ContainerAllocator>& v)
  {
    s << indent << "t: ";
    Printer<double>::stream(s, indent + "  ", v.t);
    s << indent << "m_x: ";
    Printer<double>::stream(s, indent + "  ", v.m_x);
    s << indent << "m_y: ";
    Printer<double>::stream(s, indent + "  ", v.m_y);
    s << indent << "m_z: ";
    Printer<double>::stream(s, indent + "  ", v.m_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAL_SENSOR_COMPASS_MESSAGE_DATA_H
