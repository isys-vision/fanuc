/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FLATHEADERS
#include <simple_message/messages/mikado_dynamic_joints_message.h>
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::simple_message::mikado_classes;

namespace industrial
{
namespace simple_message
{
namespace mikado_messages
{

MikadoDynamicJointsMessage::MikadoDynamicJointsMessage(void)
{
  this->init();
}

MikadoDynamicJointsMessage::~MikadoDynamicJointsMessage(void)
{

}

bool MikadoDynamicJointsMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();

  // unload eos bytes when init from Message
  int end_of_msg;
  data.unload(end_of_msg);
  int num_jts = (data.getBufferSize() - 4) / 4;

  this->init();
  this->setCommType(msg.getCommType());
  this->joints_.setNumJoints(num_jts);

  if (data.unload(this->joints_))
  {
    if (data.unload(this->sequence_)){
      rtn = true;
    } else {
      rtn = false;
      LOG_ERROR("Failed to unload sequence data");
    }
  }
  else
  {
    LOG_ERROR("Failed to unload mikado dynamic joints data");
  }
  return rtn;
}

bool MikadoDynamicJointsMessage::init(industrial::simple_message::SimpleMessage & msg, bool is_sending_is_robot_moving)
{
  bool rtn = false;
  ByteArray data = msg.getData();

  // unload eos bytes when init from Message
  int end_of_msg;
  data.unload(end_of_msg);
  int num_jts = (data.getBufferSize() - 4) / 4;

  this->init();
  this->setCommType(msg.getCommType());
  this->joints_.setNumJoints(num_jts);
  if(is_sending_is_robot_moving){
    this->joints_.setIsSendIsRobotMoving(true);
  }

  if (data.unload(this->joints_))
  {
    if (data.unload(this->sequence_)){
      rtn = true;
    } else {
      rtn = false;
      LOG_ERROR("Failed to unload sequence data");
    }
  }
  else
  {
    LOG_ERROR("Failed to unload mikado dynamic joints data");
  }
  return rtn;
}

void MikadoDynamicJointsMessage::init(MikadoDynamicJoints & joints)
{
  this->init();
  this->joints_.copyFrom(joints);
}

void MikadoDynamicJointsMessage::init()
{
  this->setMessageType(industrial::simple_message::mikado_messages::MikadoMessageType::MIKADO_DYNAMIC_JOINTS_MSG);
  this->setSequence(0);
  this->joints_.init();
}

bool MikadoDynamicJointsMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mikado dynamic joints message load");
  if (buffer->load(this->sequence_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to unload sequence data");
    }
  if (buffer->load(this->joints_))
  {
    rtn = true;
    buffer->load(simple_message::mikado_messages::EoM_bytes);
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load mikado dynamic joints data");
  }
  return rtn;
}

bool MikadoDynamicJointsMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mikado dynamic joints message unload");

  char c;
  buffer->unload(&c, 4);
  if (buffer->unload(this->joints_))
  {
    if (buffer->unload(this->sequence_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to unload sequence data");
    }
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload mikado dynamic joints data");
  }
  return rtn;
}

}
}
}

