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
 * Auto-generated by gensrv_cpp from file /home/asymingt/Workspace/Source/crates/src/hal_quadrotor/srv/Hover.srv
 *
 */


#ifndef HAL_QUADROTOR_MESSAGE_HOVER_H
#define HAL_QUADROTOR_MESSAGE_HOVER_H

#include <ros/service_traits.h>


#include <hal_quadrotor/HoverRequest.h>
#include <hal_quadrotor/HoverResponse.h>


namespace hal_quadrotor
{

struct Hover
{

typedef HoverRequest Request;
typedef HoverResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Hover
} // namespace hal_quadrotor


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hal_quadrotor::Hover > {
  static const char* value()
  {
    return "38b8954d32a849f31d78416b12bff5d1";
  }

  static const char* value(const ::hal_quadrotor::Hover&) { return value(); }
};

template<>
struct DataType< ::hal_quadrotor::Hover > {
  static const char* value()
  {
    return "hal_quadrotor/Hover";
  }

  static const char* value(const ::hal_quadrotor::Hover&) { return value(); }
};


// service_traits::MD5Sum< ::hal_quadrotor::HoverRequest> should match 
// service_traits::MD5Sum< ::hal_quadrotor::Hover > 
template<>
struct MD5Sum< ::hal_quadrotor::HoverRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hal_quadrotor::Hover >::value();
  }
  static const char* value(const ::hal_quadrotor::HoverRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hal_quadrotor::HoverRequest> should match 
// service_traits::DataType< ::hal_quadrotor::Hover > 
template<>
struct DataType< ::hal_quadrotor::HoverRequest>
{
  static const char* value()
  {
    return DataType< ::hal_quadrotor::Hover >::value();
  }
  static const char* value(const ::hal_quadrotor::HoverRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hal_quadrotor::HoverResponse> should match 
// service_traits::MD5Sum< ::hal_quadrotor::Hover > 
template<>
struct MD5Sum< ::hal_quadrotor::HoverResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hal_quadrotor::Hover >::value();
  }
  static const char* value(const ::hal_quadrotor::HoverResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hal_quadrotor::HoverResponse> should match 
// service_traits::DataType< ::hal_quadrotor::Hover > 
template<>
struct DataType< ::hal_quadrotor::HoverResponse>
{
  static const char* value()
  {
    return DataType< ::hal_quadrotor::Hover >::value();
  }
  static const char* value(const ::hal_quadrotor::HoverResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HAL_QUADROTOR_MESSAGE_HOVER_H
