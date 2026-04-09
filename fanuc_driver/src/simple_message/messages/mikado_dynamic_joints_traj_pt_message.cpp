/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
#include <simple_message/messages/mikado_dynamic_joints_traj_pt_message.h>
#include "simple_message/joint_data.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "mikado_dynamic_joints_traj_pt_message.h"
#include "joint_data.h"
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

MikadoDynamicJointsTrajPtMessage::MikadoDynamicJointsTrajPtMessage(void)
{
  this->init();
}

MikadoDynamicJointsTrajPtMessage::~MikadoDynamicJointsTrajPtMessage(void)
{

}

bool MikadoDynamicJointsTrajPtMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  LOG_ERROR("Cannot init Dynamic Message from simple message without information about number of joints, is duration used and is velocity used. Please use other init method");
  return false;
}

bool MikadoDynamicJointsTrajPtMessage::init(industrial::simple_message::SimpleMessage & msg, int num_jts, bool is_duration, bool is_velocity)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  MikadoDynamicJoints jts;
  jts.init();
  jts.setNumJoints(num_jts);

  // unload eos bytes when init from Message
  int end_of_msg;
  data.unload(end_of_msg);
  this->init();

  this->setCommType(msg.getCommType());
  this->point_.setPositions(jts);
  if (data.unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

void MikadoDynamicJointsTrajPtMessage::init(MikadoDynamicJointsTrajPt & point)
{
  this->init();
  this->point_.copyFrom(point);
}

void MikadoDynamicJointsTrajPtMessage::init()
{
  this->setMessageType(industrial::simple_message::mikado_messages::MikadoMessageType::MIKADO_DYNAMIC_JOINTS_TRAJ_PT_MSG);
  this->point_.init();
}


bool MikadoDynamicJointsTrajPtMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj. pt. message load");
  if (buffer->load(this->point_))
  {
    rtn = true;
    buffer->load(simple_message::mikado_messages::EoM_bytes);
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint traj. pt data");
  }
  return rtn;
}

bool MikadoDynamicJointsTrajPtMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj pt message unload");

  char c;
  buffer->unload(&c, 4);
  if (buffer->unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

}
}
}
