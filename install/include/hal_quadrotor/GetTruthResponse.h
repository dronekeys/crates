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
 * Auto-generated by genmsg_cpp from file /home/asymingt/Workspace/Source/crates/src/hal_quadrotor/srv/GetTruth.srv
 *
 */


#ifndef HAL_QUADROTOR_MESSAGE_GETTRUTHRESPONSE_H
#define HAL_QUADROTOR_MESSAGE_GETTRUTHRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hal_quadrotor/State.h>

namespace hal_quadrotor
{
template <class ContainerAllocator>
struct GetTruthResponse_
{
  typedef GetTruthResponse_<ContainerAllocator> Type;

  GetTruthResponse_()
    : state()  {
    }
  GetTruthResponse_(const ContainerAllocator& _alloc)
    : state(_alloc)  {
    }



   typedef  ::hal_quadrotor::State_<ContainerAllocator>  _state_type;
  _state_type state;




  typedef boost::shared_ptr< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetTruthResponse_

typedef ::hal_quadrotor::GetTruthResponse_<std::allocator<void> > GetTruthResponse;

typedef boost::shared_ptr< ::hal_quadrotor::GetTruthResponse > GetTruthResponsePtr;
typedef boost::shared_ptr< ::hal_quadrotor::GetTruthResponse const> GetTruthResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f2033d5defdcc91d37ab097f3c9ec53b";
  }

  static const char* value(const ::hal_quadrotor::GetTruthResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf2033d5defdcc91dULL;
  static const uint64_t static_value2 = 0x37ab097f3c9ec53bULL;
};

template<class ContainerAllocator>
struct DataType< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hal_quadrotor/GetTruthResponse";
  }

  static const char* value(const ::hal_quadrotor::GetTruthResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hal_quadrotor/State state\n\
\n\
================================================================================\n\
MSG: hal_quadrotor/State\n\
float64 t	    	# Time stamp\n\
float64 x	    	# n-frame X position (X == +East)\n\
float64 y	   		# n-frame Y position (Y == +North)\n\
float64 z	    	# n-frame Z position (Z == +Up)\n\
float64 roll	    # n-frame roll (anti-clockwise about X)\n\
float64 pitch	    # n-frame pitch (anti-clockwise about Y)\n\
float64 yaw	    	# n-frame yaw (anti-clockwise about Z)\n\
float64 u	    	# b-frame X velocity\n\
float64 v	    	# b-frame Y velocity\n\
float64 w	    	# b-frame Z velocity\n\
float64 p	    	# b-frame roll angular velocity\n\
float64 q	    	# b-frame pitch angular velocity\n\
float64 r	    	# b-frame yaw angular velocity\n\
float64 thrust	    # Current thrust force\n\
float64 remaining	# Flight time remaining\n\
";
  }

  static const char* value(const ::hal_quadrotor::GetTruthResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetTruthResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hal_quadrotor::GetTruthResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hal_quadrotor::GetTruthResponse_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    s << std::endl;
    Printer< ::hal_quadrotor::State_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAL_QUADROTOR_MESSAGE_GETTRUTHRESPONSE_H
