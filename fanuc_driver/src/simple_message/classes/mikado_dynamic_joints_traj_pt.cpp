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
#include <simple_message/classes/mikado_dynamic_joints_traj_pt.h>
#include <simple_message/classes/mikado_dynamic_joints.h>
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "mikado_dynamic_joints_traj_pt.h"
#include "mikado_dynamic_joints.h"
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

MikadoDynamicJointsTrajPt::MikadoDynamicJointsTrajPt(void)
{
  this->init();
}
MikadoDynamicJointsTrajPt::~MikadoDynamicJointsTrajPt(void)
{

}

void MikadoDynamicJointsTrajPt::init()
{
  this->sequence_ = 0;
  this->duration_ = 0;
  this->velocity_ = 0;
  this->positions_.init();
  this->valid_fields_ = 0;
}

void MikadoDynamicJointsTrajPt::init(industrial::shared_types::shared_int sequence,
          industrial::shared_types::shared_real duration,
          industrial::shared_types::shared_real velocity,
          MikadoDynamicJoints & positions)
{
  this->valid_fields_ = 0;
  this->setSequence(sequence);
  this->setVelocity(velocity);
  this->setDuration(duration);
  this->setPositions(positions);
}

void MikadoDynamicJointsTrajPt::copyFrom(MikadoDynamicJointsTrajPt &src)
{
  this->setSequence(src.getSequence());
  this->velocity_ = src.getVelocity();
  this->duration_ = src.getDuration();
  src.getPositions(this->positions_);
  this->valid_fields_ = src.valid_fields_;
}

bool MikadoDynamicJointsTrajPt::operator==(MikadoDynamicJointsTrajPt &rhs)
{
  return this->sequence_ == rhs.sequence_ &&
         (this->duration_ == rhs.duration_) &&
         (this->velocity_ == rhs.velocity_) &&
         (this->positions_ == rhs.positions_) &&
         (this->valid_fields_ == rhs.valid_fields_);
}

bool MikadoDynamicJointsTrajPt::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint trajectory point load");

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load joint traj. pt. sequence number");
    return false;
  }

  if(is_valid(DynamicJointsValidFieldType::DURATION)){
    if (!buffer->load(this->duration_))
    {
      LOG_ERROR("Failed to load joint traj. pt. duration");
      return false;
    }
  }

  if(is_valid(DynamicJointsValidFieldType::VELOCITY)){
    if (!buffer->load(this->velocity_))
    {
      LOG_ERROR("Failed to load joint traj. pt. velocity");
      return false;
    }
  }

  if (!this->positions_.load(buffer))
  {
    LOG_ERROR("Failed to load joint traj. pt. positions");
    return false;
  }

  LOG_COMM("Trajectory point successfully loaded");
  return true;
}

bool MikadoDynamicJointsTrajPt::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint traj. pt. unload");

  if (!this->positions_.unload(buffer))
  {
    LOG_ERROR("Failed to unload joint traj. pt. positions");
    return false;
  }
  LOG_WARN("VELOCITY FIELD VALID: %d", is_valid(DynamicJointsValidFieldType::VELOCITY));
  LOG_WARN("DURATION FIELD VALID: %d", is_valid(DynamicJointsValidFieldType::DURATION));

  if(is_valid(DynamicJointsValidFieldType::VELOCITY)){
    LOG_WARN("VELOCITY FIELD VALID -> UNLOADING");
    if (!buffer->unload(this->velocity_))
    {
      LOG_ERROR("Failed to unload joint traj. pt. velocity");
      return false;
    }
  }

  if(is_valid(DynamicJointsValidFieldType::DURATION)){
    LOG_WARN("DURATION FIELD VALID -> UNLOADING");
    if (!buffer->unload(this->duration_))
    {
      LOG_ERROR("Failed to unload joint traj. pt. duration");
      return false;
    }
  }

  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload joint traj. pt. sequence number");
    return false;
  }

  LOG_COMM("Joint traj. pt successfully unloaded");
  return true;
}

}
}
}

