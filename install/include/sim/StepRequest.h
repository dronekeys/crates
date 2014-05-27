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
 * Auto-generated by genmsg_cpp from file /home/asymingt/Workspace/Source/crates/src/sim/srv/Step.srv
 *
 */


#ifndef SIM_MESSAGE_STEPREQUEST_H
#define SIM_MESSAGE_STEPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sim
{
template <class ContainerAllocator>
struct StepRequest_
{
  typedef StepRequest_<ContainerAllocator> Type;

  StepRequest_()
    : num_steps(0)  {
    }
  StepRequest_(const ContainerAllocator& _alloc)
    : num_steps(0)  {
    }



   typedef uint32_t _num_steps_type;
  _num_steps_type num_steps;




  typedef boost::shared_ptr< ::sim::StepRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sim::StepRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StepRequest_

typedef ::sim::StepRequest_<std::allocator<void> > StepRequest;

typedef boost::shared_ptr< ::sim::StepRequest > StepRequestPtr;
typedef boost::shared_ptr< ::sim::StepRequest const> StepRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sim::StepRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sim::StepRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sim

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sim': ['/home/asymingt/Workspace/Source/crates/src/sim/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sim::StepRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sim::StepRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sim::StepRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sim::StepRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sim::StepRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sim::StepRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sim::StepRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c00bc357dda60c1b215e553c5d1cdb5b";
  }

  static const char* value(const ::sim::StepRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc00bc357dda60c1bULL;
  static const uint64_t static_value2 = 0x215e553c5d1cdb5bULL;
};

template<class ContainerAllocator>
struct DataType< ::sim::StepRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sim/StepRequest";
  }

  static const char* value(const ::sim::StepRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sim::StepRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 num_steps\n\
";
  }

  static const char* value(const ::sim::StepRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sim::StepRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num_steps);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct StepRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sim::StepRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sim::StepRequest_<ContainerAllocator>& v)
  {
    s << indent << "num_steps: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_steps);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIM_MESSAGE_STEPREQUEST_H