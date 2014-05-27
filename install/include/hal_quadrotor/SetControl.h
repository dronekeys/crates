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
 * Auto-generated by gensrv_cpp from file /home/asymingt/Workspace/Source/crates/src/hal_quadrotor/srv/SetControl.srv
 *
 */


#ifndef HAL_QUADROTOR_MESSAGE_SETCONTROL_H
#define HAL_QUADROTOR_MESSAGE_SETCONTROL_H

#include <ros/service_traits.h>


#include <hal_quadrotor/SetControlRequest.h>
#include <hal_quadrotor/SetControlResponse.h>


namespace hal_quadrotor
{

struct SetControl
{

typedef SetControlRequest Request;
typedef SetControlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetControl
} // namespace hal_quadrotor


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hal_quadrotor::SetControl > {
  static const char* value()
  {
    return "9768c483ed82dc040e3bed71672eb4ef";
  }

  static const char* value(const ::hal_quadrotor::SetControl&) { return value(); }
};

template<>
struct DataType< ::hal_quadrotor::SetControl > {
  static const char* value()
  {
    return "hal_quadrotor/SetControl";
  }

  static const char* value(const ::hal_quadrotor::SetControl&) { return value(); }
};


// service_traits::MD5Sum< ::hal_quadrotor::SetControlRequest> should match 
// service_traits::MD5Sum< ::hal_quadrotor::SetControl > 
template<>
struct MD5Sum< ::hal_quadrotor::SetControlRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hal_quadrotor::SetControl >::value();
  }
  static const char* value(const ::hal_quadrotor::SetControlRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hal_quadrotor::SetControlRequest> should match 
// service_traits::DataType< ::hal_quadrotor::SetControl > 
template<>
struct DataType< ::hal_quadrotor::SetControlRequest>
{
  static const char* value()
  {
    return DataType< ::hal_quadrotor::SetControl >::value();
  }
  static const char* value(const ::hal_quadrotor::SetControlRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hal_quadrotor::SetControlResponse> should match 
// service_traits::MD5Sum< ::hal_quadrotor::SetControl > 
template<>
struct MD5Sum< ::hal_quadrotor::SetControlResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hal_quadrotor::SetControl >::value();
  }
  static const char* value(const ::hal_quadrotor::SetControlResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hal_quadrotor::SetControlResponse> should match 
// service_traits::DataType< ::hal_quadrotor::SetControl > 
template<>
struct DataType< ::hal_quadrotor::SetControlResponse>
{
  static const char* value()
  {
    return DataType< ::hal_quadrotor::SetControl >::value();
  }
  static const char* value(const ::hal_quadrotor::SetControlResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HAL_QUADROTOR_MESSAGE_SETCONTROL_H