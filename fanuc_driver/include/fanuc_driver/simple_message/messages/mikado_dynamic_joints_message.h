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

#ifndef MIKADO_DYNAMIC_JOINTS_MESSAGE
#define MIKADO_DYNAMIC_JOINTS_MESSAGE

#ifndef FLATHEADERS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
#include "simple_message/classes/mikado_dynamic_joints.h"
#include "simple_message/classes/mikado_types.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "mikado_status.h"
#endif

using namespace industrial::simple_message::mikado_classes;

namespace industrial
{
namespace simple_message
{
namespace mikado_messages
{



/**
 * \brief Class encapsulated dynamic joints generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::mikado_status::MikadoDynamicJoints data type.
 * The data portion of this typed message matches MikadoDynamicJoints.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MikadoDynamicJointsMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  MikadoDynamicJointsMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~MikadoDynamicJointsMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

    /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \param bool if joint_state contains is_robot_moving
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg, bool is_sending_is_robot_moving);

  /**
   * \brief Initializes message from a mikado dynamic joints structure
   *
   * \param status structure to initialize from
   *
   */
  void init(MikadoDynamicJoints & joints);

  /**
   * \brief Initializes message from a mikado dynamic joints structure
   *
   * \param status structure to initialize from
   *
   */
  void init(shared_types::shared_int sequence, MikadoDynamicJoints & joints);

  /**
   * \brief Initializes a new mikado status message
   *
   */
  void init();

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence){
    this->sequence_ = sequence;
  }

  /**
   * \brief returns the maximum message sequence number
   *
   * \return message sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return sequence_;
  }

  /**
   * \brief returns reference to underlying joint class
   *
   * \return reference to joint class
   */
  MikadoDynamicJoints & getJoints()
  {
    return this->joints_;
  }


  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return sizeof(shared_types::shared_int) + this->joints_.byteLength();
  }

  shared_types::shared_int sequence_;

  MikadoDynamicJoints joints_;

};

}
}
}

#endif /* MIKADO_DYNAMIC_JOINTS_MESSAGE */
