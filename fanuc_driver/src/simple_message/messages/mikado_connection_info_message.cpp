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
#include "simple_message/messages/mikado_connection_info_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace industrial
{
namespace simple_message
{
namespace mikado_messages
{

MikadoConnectionInfoMessage::MikadoConnectionInfoMessage(void)
{
  this->init();
}

MikadoConnectionInfoMessage::~MikadoConnectionInfoMessage(void)
{

}

bool MikadoConnectionInfoMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();

  // unload eos bytes when init from Message
  int end_of_msg;
  data.unload(end_of_msg);

  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->connection_info_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload robot connection_info data");
  }
  return rtn;
}

void MikadoConnectionInfoMessage::init(MikadoConnectionInfo & connection_info)
{
  this->init();
  this->connection_info_.copyFrom(connection_info);
}

void MikadoConnectionInfoMessage::init()
{
  this->setMessageType(industrial::simple_message::mikado_messages::MikadoMessageType::MIKADO_CONNECTION_INFO_MSG);
}

bool MikadoConnectionInfoMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing robot connection_info message load");


  if (buffer->load(this->connection_info_))
  {
    buffer->load(simple_message::mikado_messages::EoM_bytes);
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load robot connection_info data");
  }
  return rtn;
}

bool MikadoConnectionInfoMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing robot connection_info message unload");

  // unload eos bytes when init from Message
  char eos;
  buffer->unload(&eos, 4);

  if (buffer->unload(this->connection_info_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload robot connection_info data");
  }
  return rtn;
}

}
}
}

