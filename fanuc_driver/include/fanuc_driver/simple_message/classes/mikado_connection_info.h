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

#ifndef MIKADO_CONNECTION_INFO_H
#define MIKADO_CONNECTION_INFO_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif


namespace industrial
{
namespace simple_message
{
namespace mikado_classes
{

/**
 * \brief Class encapsulated connection information data.  The connection information data is
 * used to tell the robot what data it will receive and should send. This allows to reduce the transfered data to the bare minimum.
 *
 *
 * The byte representation of the connection information is as follows (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   num_ax_traj       (industrial::shared_types::shared_int)    4  bytes
 *   num_ax_state      (industrial::shared_types::shared_int)    4  bytes
 *   num_eax_traj      (industrial::shared_types::shared_int)    4  bytes
 *   num_eax_state     (industrial::shared_types::shared_int)    4  bytes
 *   is_traj_radian    (industrial::shared_types::shared_bool)   1  bytes
 *   is_state_radian   (industrial::shared_types::shared_bool)   1  bytes
 *   is_traj_velocity  (industrial::shared_types::shared_bool)   1  bytes
 *   is_traj_duration  (industrial::joint_states::shared_bool)   1  bytes
 *   is_traj_is_moving (industrial::joint_states::shared_bool)   1  bytes
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MikadoConnectionInfo : public industrial::simple_serialize::SimpleSerialize
{
public:
/**
 * \brief Default constructor
 *
 * This method creates empty data.
 *
 */
MikadoConnectionInfo(void);
/**
 * \brief Destructor
 *
 */
~MikadoConnectionInfo(void);

/**
 * \brief Initializes an empty robot connection info
 *
 */
void init();

/**
 * \brief Initializes a full robot connection info
 *
 */
void init(shared_types::shared_int num_ax_traj, shared_types::shared_int num_ax_state, shared_types::shared_int num_eax_traj, shared_types::shared_int num_eax_state, shared_types::shared_bool is_traj_radian, shared_types::shared_bool is_state_radian, shared_types::shared_bool is_traj_velocity, shared_types::shared_bool is_traj_duration, shared_types::shared_bool is_traj_is_moving);

shared_types::shared_int getNumAxTraj() const
{
  return num_ax_traj_;
}

shared_types::shared_int getNumAxState() const
{
  return num_ax_state_;
}

shared_types::shared_int getNumEaxTraj() const
{
  return num_eax_traj_;
}

shared_types::shared_int getNumEaxState() const
{
  return num_eax_state_;
}

shared_types::shared_bool getIsTrajRadian() const
{
  return is_traj_radian_;
}

shared_types::shared_bool getIsStateRadian() const
{
  return is_state_radian_;
}

shared_types::shared_bool getIsTrajVelocity() const
{
  return is_traj_velocity_;
}

shared_types::shared_bool getIsTrajDuration() const
{
  return is_traj_duration_;
}

shared_types::shared_bool getIsTrajIsMoving() const
{
  return is_traj_is_moving_;
}

void setNumAxTraj(shared_types::shared_int value)
{
  num_ax_traj_ = value;
}

void setNumAxState(shared_types::shared_int value)
{
  num_ax_state_ = value;
}

void setNumEaxTraj(shared_types::shared_int value)
{
  num_eax_traj_ = value;
}

void setNumEaxState(shared_types::shared_int value)
{
  num_eax_state_ = value;
}

void setIsTrajRadian(shared_types::shared_bool value)
{
  is_traj_radian_ = value;
}

void setIsStateRadian(shared_types::shared_bool value)
{
  is_state_radian_ = value;
}

void setIsTrajVelocity(shared_types::shared_bool value)
{
  is_traj_velocity_ = value;
}

void setIsTrajDuration(shared_types::shared_bool value)
{
  is_traj_duration_ = value;
}

void setIsTrajIsMoving(shared_types::shared_bool value)
{
  is_traj_is_moving_ = value;
}

/**
 * \brief Copies the passed in value
 *
 * \param src (value to copy)
 */
void copyFrom(MikadoConnectionInfo &src);

/**
 * \brief == operator implementation
 *
 * \return true if equal
 */
bool operator==(MikadoConnectionInfo &rhs);

// Overrides - SimpleSerialize
bool load(industrial::byte_array::ByteArray *buffer);
bool unload(industrial::byte_array::ByteArray *buffer);
unsigned int byteLength()
{
  return 4 * sizeof(industrial::shared_types::shared_int) + 5 * sizeof(industrial::shared_types::shared_bool);
}

private:

/**
 * \brief Defines the number of axis that are sent for traj points
 */
industrial::shared_types::shared_int num_ax_traj_;

/**
 * \brief Defines the number of axis that are sent for states
 */
industrial::shared_types::shared_int num_ax_state_;

/**
 * \brief Defines the number of external axis that are sent for traj points
 */
industrial::shared_types::shared_int num_eax_traj_;

/**
 * \brief Defines the number of external axis that are sent for states
 */
industrial::shared_types::shared_int num_eax_state_;

/**
 * \brief Defines if traj points are sent in radian (otherwise degree)
 */
industrial::shared_types::shared_bool is_traj_radian_;

/**
 * \brief Defines if states are sent in radian (otherwise degree)
 */
industrial::shared_types::shared_bool is_state_radian_;

/**
 * \brief Defines if a velocity is sent with each trajectory point
 */
industrial::shared_types::shared_bool is_traj_velocity_;

/**
 * \brief Defines if a duration is sent with each trajectory point
 */
industrial::shared_types::shared_bool is_traj_duration_;

/**
 * \brief Defines if "is_moving" state is sent witch each state update
 */
industrial::shared_types::shared_bool is_traj_is_moving_;

};

}
}
}

#endif /* MIKADO_CONNECTION_INFO_H */
