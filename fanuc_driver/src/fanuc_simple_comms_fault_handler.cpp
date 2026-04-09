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
#include "fanuc_simple_comms_fault_handler.h"
#include "simple_message/log_wrapper.h"
#include <ros/ros.h>
#else
#include "fanuc_simple_comms_fault_handler.h"
#include "log_wrapper.h"
#endif

namespace industrial
{
namespace fanuc_simple_comms_fault_handler
{

FanucSimpleCommsFaultHandler::FanucSimpleCommsFaultHandler()
{
  this->connection_ = NULL;
}


FanucSimpleCommsFaultHandler::~FanucSimpleCommsFaultHandler()
{
}

bool FanucSimpleCommsFaultHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  bool rtn = false;

  if (NULL != connection)
  {
    this->setConnection(connection);
    ROS_WARN("Default communications fault handler successfully initialized");
    rtn = true;
  }
  else
  {
    ROS_WARN("Failed to initialize default communications fault handler");
    rtn = false;
  }
  return rtn;
}

void FanucSimpleCommsFaultHandler::setConnectionInfoMessage(const industrial::simple_message::SimpleMessage& connection_info_msg){
    this->connection_info_msg_ = std::make_unique<industrial::simple_message::SimpleMessage>(connection_info_msg);
}

void FanucSimpleCommsFaultHandler::connectionFailCB()
{

  if (!(this->getConnection()->isConnected()))
  {
    ROS_WARN("Connection failed, attempting reconnect");
    this->getConnection()->makeConnect();
    if(this->connection_info_msg_){
        // send connection info
        bool res = this->connection_->sendMsg(*this->connection_info_msg_);
        if (res){
            ROS_WARN("Sent connection info after reconnect: Success");
        } else {
        ROS_WARN("Failed to send connection info after reconnect: Failure");
        }
    } else {
        ROS_WARN("Not sending connection info message after reconnect as it has not been specified.");
    }
  }
  else
  {
    ROS_WARN("Connection fail callback called while still connected (Possible bug)");
  }
}



}//namespace default_comms_fault_handler
}//namespace industrial




