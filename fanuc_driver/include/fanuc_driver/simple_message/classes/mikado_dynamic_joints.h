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

#ifndef MIKADO_DYNAMIC_JOINTS_H
#define MIKADO_DYNAMIC_JOINTS_H

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
 * \brief Class encapsulated a dynmaic amount of reals used for joint data
 *
 *
 *   member:             type                                      size
 *   arguments           (industrial::shared_types::shared_real)   4 * num_args
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MikadoDynamicJoints : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MikadoDynamicJoints(void);
  /**
   * \brief Destructor
   *
   */
  ~MikadoDynamicJoints(void);

  /**
   * \brief Initializes mpty arguments
   *
   */
  void init(industrial::shared_types::shared_int num_joints=10, shared_types::shared_bool is_sending_is_robot_moving=false);

  /**
   * \brief Sets the number of joints
   *
   * \param argument number of joints
   *
   * \return true if num_joints updated, otherwise false (more than max joints)
   */
  bool setNumJoints(industrial::shared_types::shared_int num_joints);

  /**
   * \brief Gets the number of joints
   *
   * \return number of joints
   */
  shared_types::shared_int getNumJoints(){
    return this->num_joints_;
  }

  /**
   * \brief Sets if the bool for sending "is_robot_moving" should be sent
   *
   * \param argument number of joints
   *
   * \return true if is_send_is_robot_moving updated, otherwise false
   */
  bool setIsSendIsRobotMoving(industrial::shared_types::shared_bool is_send_is_robot_moving);

  /**
   * \brief Gets if the bool that determins if the bool for "is_robot_moving" should be sent
   *
   * \return is_sending_is_robot_moving
   */
  bool getIsSendIsRobotMoving(){
    return this->is_send_is_robot_moving;
  }

  /**
   * \brief Sets "is_robot_moving"
   *
   * \param is_robot_moving
   *
   * \return true if is_robot_moving updated, otherwise false
   */
  bool setIsRobotMoving(industrial::shared_types::shared_bool is_send_is_robot_moving);

  /**
   * \brief Gets if the bool that determins if the bool for "is_robot_moving" should be sent
   *
   * \return is_sending_is_robot_moving
   */
  bool getIsRobotMoving(){
    return this->is_robot_moving;
  }


  /**
   * \brief Sets a joint argument within the buffer
   *
   * \param argument index
   * \param arument value
   *
   * \return true if argument set, otherwise false (index greater than max)
   */
  bool setJoint(industrial::shared_types::shared_int index, industrial::shared_types::shared_real value);

  /**
   * \brief Gets a joint argument within the buffer
   *
   * \param argument index
   * \param argument value (passed by reference)
   *
   * \return true if value valid, otherwise false (index greater than max)
   */
  bool getJoint(industrial::shared_types::shared_int index, industrial::shared_types::shared_real & value) const;

 /**
   * \brief Gets a joint argument within the buffer
   *
   * \param argument index
   *
   * \return Argument (-1 if out of bounds)
   */
  industrial::shared_types::shared_real getJoint(industrial::shared_types::shared_int index) const;

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MikadoDynamicJoints &src);

  /**
   * \brief returns the maximum number of arguments the message holds
   *
   * \return max number of arguments
   */
  int getNumJoints() const
  {
    return this->num_joints_;
  }

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MikadoDynamicJoints &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return this->num_joints_ * sizeof(industrial::shared_types::shared_real) + (is_send_is_robot_moving
        ? sizeof(industrial::shared_types::shared_bool) : 0);
  }

private:

  /**
   * \brief number of joints that can be held in the message.
   */
  industrial::shared_types::shared_int num_joints_;

  /**
   * \brief bool indicating if is_robot_moving will be sent
   */
  industrial::shared_types::shared_bool is_send_is_robot_moving;

  /**
   * \brief bool indicating if robot is moving (will only be sent if is_send_is_robot_moving is true)
   */
  industrial::shared_types::shared_bool is_robot_moving;
  /**
   * \brief internal data buffer
   */
  std::vector<industrial::shared_types::shared_real> joints_;

};

}
}
}

#endif /* MIKADO_DYNAMIC_JOINTS_H */
