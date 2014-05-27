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
 * Auto-generated by genmsg_cpp from file /home/asymingt/Workspace/Source/crates/src/hal_quadrotor/srv/VelocityHeight.srv
 *
 */


#ifndef HAL_QUADROTOR_MESSAGE_VELOCITYHEIGHTREQUEST_H
#define HAL_QUADROTOR_MESSAGE_VELOCITYHEIGHTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hal_quadrotor
{
template <class ContainerAllocator>
struct VelocityHeightRequest_
{
  typedef VelocityHeightRequest_<ContainerAllocator> Type;

  VelocityHeightRequest_()
    : u(0.0)
    , v(0.0)
    , z(0.0)
    , yaw(0.0)  {
    }
  VelocityHeightRequest_(const ContainerAllocator& _alloc)
    : u(0.0)
    , v(0.0)
    , z(0.0)
    , yaw(0.0)  {
    }



   typedef double _u_type;
  _u_type u;

   typedef double _v_type;
  _v_type v;

   typedef double _z_type;
  _z_type z;

   typedef double _yaw_type;
  _yaw_type yaw;




  typedef boost::shared_ptr< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> const> ConstPtr;

}; // struct VelocityHeightRequest_

typedef ::hal_quadrotor::VelocityHeightRequest_<std::allocator<void> > VelocityHeightRequest;

typedef boost::shared_ptr< ::hal_quadrotor::VelocityHeightRequest > VelocityHeightRequestPtr;
typedef boost::shared_ptr< ::hal_quadrotor::VelocityHeightRequest const> VelocityHeightRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hal_quadrotor

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'hal_quadrotor': ['/home/asymingt/Workspace/Source/crates/src/hal_quadrotor/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8346db72e78e073320fd8035eed9a548";
  }

  static const char* value(const ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8346db72e78e0733ULL;
  static const uint64_t static_value2 = 0x20fd8035eed9a548ULL;
};

template<class ContainerAllocator>
struct DataType< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hal_quadrotor/VelocityHeightRequest";
  }

  static const char* value(const ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
float64 u\n\
float64 v\n\
float64 z\n\
float64 yaw\n\
";
  }

  static const char* value(const ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.u);
      stream.next(m.v);
      stream.next(m.z);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct VelocityHeightRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hal_quadrotor::VelocityHeightRequest_<ContainerAllocator>& v)
  {
    s << indent << "u: ";
    Printer<double>::stream(s, indent + "  ", v.u);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAL_QUADROTOR_MESSAGE_VELOCITYHEIGHTREQUEST_H
