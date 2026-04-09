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

#ifndef MIKADO_DYNAMIC_JOINT_TRAJ_PT
#define MIKADO_DYNAMIC_JOINT_TRAJ_PT

#ifndef FLATHEADERS
#include "simple_message/classes/mikado_dynamic_joints.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

namespace industrial
{
namespace simple_message
{
namespace mikado_classes
{

namespace DynamicJointsValidFieldTypes
{
enum DynamicJointsValidFieldType
{
  DURATION = 0x01, VELOCITY = 0x02, POSITION = 0x04
};
}
typedef DynamicJointsValidFieldTypes::DynamicJointsValidFieldType DynamicJointsValidFieldType;

/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt class, but this
 * class provides the full message contents directly to the robot controller,
 * rather than simplifying the velocity duration.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   velocity            (industrial::shared_types::shared_real)   4  bytes
 *   duration            (industrial::shared_types::shared_real)   4  bytes
 *   positions           (industrial::mikado_dynamic_joints) nr_jts * 4 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MikadoDynamicJointsTrajPt : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MikadoDynamicJointsTrajPt(void);
  /**
   * \brief Destructor
   *
   */
  ~MikadoDynamicJointsTrajPt(void);

  /**
   * \brief Initializes a empty dynamic joint trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a dynamic joint trajectory point
   *
   */
  void init(industrial::shared_types::shared_int sequence,
            industrial::shared_types::shared_real velocity,
            industrial::shared_types::shared_real duration,
            industrial::simple_message::mikado_classes::MikadoDynamicJoints & positions);

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }

   /**
   * \brief Sets the duration
   *
   * \param duration
   */
  void setDuration(industrial::shared_types::shared_real duration=0)
  {
    this->duration_ = duration;
    this->valid_fields_ |= DynamicJointsValidFieldType::DURATION;  // set the bit

  }

  /**
   * \brief Returns duration
   *
   * \return duration
   */
  industrial::shared_types::shared_real getDuration()
  {
    if(!is_valid(DynamicJointsValidFieldType::DURATION)){
      LOG_WARN("Optional field duration not set.");
    }
    return this->duration_;
  }

   /**
   * \brief Sets velocity
   *
   * \param velocity
   */
  void setVelocity(industrial::shared_types::shared_real velocity=0)
  {
    this->velocity_ = velocity;
    this->valid_fields_ |= DynamicJointsValidFieldType::VELOCITY;  // set the bit
  }

   /**
   * \brief Sets the field to valid
   *
   * \param field_type
   */
  void setField(DynamicJointsValidFieldType field_type)
  {
    this->valid_fields_ |= field_type;  // set the bit
  }

  /**
   * \brief Sets the field to invalid
   *
   * \param field_type
   */
  void setFieldInvalid(DynamicJointsValidFieldType field_type)
  {
    this->valid_fields_ &= field_type;  // set the bit
  }

  /**
   * \brief Returns velocity
   *
   * \return velocity
   */
  industrial::shared_types::shared_real getVelocity()
  {
    if(!is_valid(DynamicJointsValidFieldType::VELOCITY)){
      LOG_WARN("Optional field velocity not set");
    }
    return this->velocity_;
  }

  /**
   * \brief Sets joint position data
   *
   * \param positions new joint position data
   */
  void setPositions(industrial::simple_message::mikado_classes::MikadoDynamicJoints &positions)
  {
    this->positions_.copyFrom(positions);
    this->valid_fields_ |= DynamicJointsValidFieldTypes::POSITION;  // set the bit
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param dest returned joint position
   * \return true if this field contains valid data
   */
  bool getPositions(industrial::simple_message::mikado_classes::MikadoDynamicJoints &dest)
  {
    dest.copyFrom(this->positions_);
    return true;
  }

  /**
   * \brief Clears the position data
   */
  void clearPositions()
  {
    this->positions_.init();
    this->valid_fields_ &= DynamicJointsValidFieldType::POSITION;  // set the bit
  }

  /**
   * \brief check the validity state for a given field
   * @param field field to check
   * @return true if specified field contains valid data
   */
  bool is_valid(DynamicJointsValidFieldType field)
  {
    return valid_fields_ & field;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MikadoDynamicJointsTrajPt &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MikadoDynamicJointsTrajPt &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    const unsigned int validReals =
        static_cast<unsigned int>(is_valid(DynamicJointsValidFieldType::VELOCITY)) +
        static_cast<unsigned int>(is_valid(DynamicJointsValidFieldType::DURATION));

    return sizeof(industrial::shared_types::shared_int) +
           validReals * sizeof(industrial::shared_types::shared_real) +
           positions_.byteLength();
  }

private:
    /**
   * \brief bit-mask of (optional) fields that have been initialized with valid data -> This information is not part of the actual message and only used to determine what information to put into/extract from the byte buffer
   * \see enum DynamicJointsValidFieldTypes
   */
  industrial::shared_types::shared_int valid_fields_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

  /**
   * \brief duration of movement to this point
   */
  industrial::shared_types::shared_real duration_;

   /**
   * \brief robot velocity when moving to this points
   */
  industrial::shared_types::shared_real velocity_;

  /**
   * \brief dynamic joint trajectory point positional data
   */
  industrial::simple_message::mikado_classes::MikadoDynamicJoints positions_;



};

}
}
}

#endif /* MIKADO_DYNAMIC_JOINT_TRAJ_PT */
