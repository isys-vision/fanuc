/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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
#include "simple_message/classes/mikado_dynamic_joints.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_data.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::simple_message::mikado_classes;

namespace industrial
{
namespace simple_message
{
namespace mikado_classes
{

MikadoDynamicJoints::MikadoDynamicJoints(void)
{
  this->init();
}
MikadoDynamicJoints::~MikadoDynamicJoints(void)
{

}

void MikadoDynamicJoints::init(shared_int num_joints, shared_bool is_send_is_robot_moving)
{
  this->num_joints_ = num_joints;
  this->joints_.resize(num_joints);
  this->is_send_is_robot_moving = is_send_is_robot_moving;
  for (int i = 0; i < this->getNumJoints(); i++)
  {
    this->setJoint(i, 0.0);
  }
}

industrial::shared_types::shared_bool is_send_is_robot_moving;
industrial::shared_types::shared_bool is_robot_moving;

bool MikadoDynamicJoints::setIsRobotMoving(shared_bool is_robot_moving){
    return this->is_robot_moving = is_robot_moving;
}

bool MikadoDynamicJoints::setIsSendIsRobotMoving(shared_bool is_send_is_robot_moving){
    return this->is_send_is_robot_moving = is_send_is_robot_moving;
}

bool MikadoDynamicJoints::setNumJoints(shared_int num_joints){
    this->joints_.resize(num_joints);
    return this->num_joints_ = num_joints;
}

bool MikadoDynamicJoints::setJoint(shared_int index, shared_real value)
{
  bool rtn = false;

  if (index < this->getNumJoints())
  {
    this->joints_[index] = value;
    rtn = true;
  }
  else
  {
    LOG_ERROR("Arg index: %d, is greater than size: %d", index, this->getNumJoints());
    rtn = false;
  }
  return rtn;
}

bool MikadoDynamicJoints::getJoint(shared_int index, shared_real & value) const
{
  bool rtn = false;

  if (index < this->getNumJoints())
  {
    value = this->joints_[index];
    rtn = true;
  }
  else
  {
    LOG_ERROR("Arg index: %d, is greater than size: %d", index, this->getNumJoints());
    rtn = false;
  }
  return rtn;
}

shared_real MikadoDynamicJoints::getJoint(shared_int index) const
{
  shared_real rtn = -1.0;
  this->getJoint(index, rtn);
  return rtn;
}


void MikadoDynamicJoints::copyFrom(MikadoDynamicJoints &src)
{
  shared_real value = 0.0;
  this->setNumJoints(src.getNumJoints());
  this->setIsSendIsRobotMoving(src.getIsSendIsRobotMoving());
  this->setIsRobotMoving(src.getIsRobotMoving());

  for (int i = 0; i < this->getNumJoints(); i++)
  {
    src.getJoint(i, value);
    this->setJoint(i, value);
  }
}

bool MikadoDynamicJoints::operator==(MikadoDynamicJoints &rhs)
{
  bool rtn = true;

  shared_real lhsvalue, rhsvalue;
  if(this->getNumJoints() != rhs.getNumJoints() ||
     this->getIsSendIsRobotMoving() != rhs.getIsSendIsRobotMoving() ||
     this->getIsRobotMoving() != rhs.getIsRobotMoving()
  ){
    return false;
  }

  for (int i = 0; i < this->getNumJoints(); i++)
  {
    this->getJoint(i, lhsvalue);
    rhs.getJoint(i, rhsvalue);
    if (lhsvalue != rhsvalue)
    {
      rtn = false;
      break;
    }
  }
  return rtn;

}

bool MikadoDynamicJoints::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  LOG_COMM("Executing joint args load");
  for (int i = 0; i < this->getNumJoints(); i++)
  {
    this->getJoint(i, value);
    rtn = buffer->load(value);
    if (!rtn)
    {
      LOG_ERROR("Failed to load joint args data");
      break;
    }
  }
  if(this->getIsSendIsRobotMoving()){
    if(!buffer->load(this->getIsRobotMoving())){
      LOG_ERROR("Failed to load is_robot_moving data");
    }
  }
  return rtn;
}

bool MikadoDynamicJoints::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  if(this->getIsSendIsRobotMoving()){
    shared_bool is_robot_moving;
    if(!buffer->unload(is_robot_moving)){
      LOG_ERROR("Failed to load is_robot_moving data");
    }
    this->setIsRobotMoving(is_robot_moving);
  }

  for (int i = this->getNumJoints() - 1; i >= 0; i--)
  {
    rtn = buffer->unload(value);
    if (!rtn)
    {
      LOG_ERROR("Failed to unload message arg: %d from data[%d]", i, buffer->getBufferSize());
      break;
    }
    this->setJoint(i, value);
  }
  return rtn;
}

}
}
}

