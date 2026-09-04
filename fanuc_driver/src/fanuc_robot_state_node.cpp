/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-2015, TU Delft Robotics Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the TU Delft Robotics Institute nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: G.A. vd. Hoorn - TU Delft Robotics Institute
 */

#include "fanuc_driver/fanuc_utils.h"
#include "fanuc_driver/fanuc_simple_comms_fault_handler.h"
#include "fanuc_driver/fanuc_robot_state_interface.h"
#include "fanuc_driver/dynamic_joints_relay_handler.h"
#include "industrial_utils/param_utils.h"
#include "simple_message/classes/mikado_connection_info.h"
#include "simple_message/messages/mikado_connection_info_message.h"
#include "simple_message/classes/mikado_dynamic_joints.h"
#include "simple_message/messages/mikado_dynamic_joints_message.h"
#include "simple_message/classes/mikado_types.h"
#include "simple_message/simple_message.h"
#include "simple_message/messages/joint_message.h"
#include <industrial_msgs/RobotStatus.h>
#include <mutex>
#include <csignal>


using industrial_robot_client::fanuc_robot_state_interface::FanucRobotStateInterface;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial::fanuc_simple_comms_fault_handler::FanucSimpleCommsFaultHandler;

class Fanuc_RobotStateInterface : public FanucRobotStateInterface
{
  private:
    FanucSimpleCommsFaultHandler fanucSimpleCommsFaultHandler_;

  public:

    industrial::simple_message::SimpleMessage prepareConnectionInfoMsg(){
      industrial::simple_message::SimpleMessage msg;
      industrial::simple_message::mikado_classes::MikadoConnectionInfo mikConnectionInfo;
      industrial::simple_message::mikado_messages::MikadoConnectionInfoMessage mikConnectionInfoMsg;
      int number_axis_state = 6;
      int number_external_axis_state = 0;
      bool is_state_radian = false;
      bool is_traj_is_moving = true;

      ros::NodeHandle handle;
      // read ros_params in, if not present, use defaults
      std::string prefix = "/robot_description_manipulators/manipulator/";
      handle.param(prefix + "connection_info_number_axis_state", number_axis_state, number_axis_state);
      handle.param(prefix + "connection_info_number_external_axis_state", number_external_axis_state, number_external_axis_state);
      handle.param(prefix + "connection_info_is_state_radian", is_state_radian, is_state_radian);
      handle.param(prefix + "connection_info_is_traj_is_moving", is_traj_is_moving, is_traj_is_moving);

      mikConnectionInfo.init(0, number_axis_state, 0,
                              number_external_axis_state, false, is_state_radian, false,
                              false, is_traj_is_moving);
      mikConnectionInfoMsg.init(mikConnectionInfo);
      mikConnectionInfoMsg.toRequest(msg);
      number_axis_state_ = number_axis_state;
      number_external_axis_state_ = number_external_axis_state;
      is_state_radian_ = is_state_radian;
      is_traj_is_moving_ = is_traj_is_moving;
      ROS_INFO("Sent connection information. State connection parameters:\n"
                "  number_axis_state: %d\n"
                "  number_external_axis_state: %d\n"
                "  is_state_radian: %s\n"
                "  is_traj_is_moving: %s",
                number_axis_state,
                number_external_axis_state,
                is_state_radian ? "true" : "false",
                is_traj_is_moving ? "true" : "false"
              );
      fanucSimpleCommsFaultHandler_.init(this->get_connection());
      fanucSimpleCommsFaultHandler_.setConnectionInfoMessage(msg);
      this->get_manager()->setCommsFaultHandler(&fanucSimpleCommsFaultHandler_);
      return msg;
   }

  bool negotiateConnection(){
    industrial::simple_message::SimpleMessage msg;
    industrial::simple_message::mikado_classes::MikadoConnectionInfo mikConnectionInfo;
    industrial::simple_message::mikado_messages::MikadoConnectionInfoMessage mikConnectionInfoMsg;

    msg = prepareConnectionInfoMsg();
    if(!this->connection_){
      ROS_ERROR("[RobotStateNode] Cannot send connection information as no connection has been initialized");
      return false;
    }
    if (!this->connection_->isConnected()) {
      ROS_ERROR("[RobotStateNode] Cannot send connection information as no connection has been established");
      return false;
    }
    bool res = this->connection_->sendMsg(msg);
    if (res){
      ROS_WARN("[INIT] Set connection message");
    } else {
      ROS_WARN("Failed to send connection information");
    }
    return res;
  }

  int number_axis_state_;
  int number_external_axis_state_;
  bool is_state_radian_;
  bool is_traj_is_moving_;
};

class Fanuc_JointRelayHandler : public industrial_robot_client::dynamic_joints_relay_handler::DynamicJointsRelayHandler
{
  int J23_factor_;
  ros::Subscriber robot_status_sub_;
  ros::Publisher  robot_status_pub_;
  industrial_msgs::RobotStatus last_robot_status_;
  bool have_robot_status_ = false;
  std::mutex robot_status_mutex_;

public:
  Fanuc_JointRelayHandler() : industrial_robot_client::dynamic_joints_relay_handler::DynamicJointsRelayHandler()
  {
    if (ros::param::has("J23_factor"))
      ros::param::get("J23_factor", this->J23_factor_);
    else
      J23_factor_ = 0;
  }

  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string>& joint_names)
  {
    bool ok = DynamicJointsRelayHandler::init(connection, joint_names);

    ros::NodeHandle nh;
    robot_status_sub_ = nh.subscribe("/robot_status", 1, &Fanuc_JointRelayHandler::robotStatusCB, this);
    robot_status_pub_ = nh.advertise<industrial_msgs::RobotStatus>("/robot_status", 1);
    return ok;
  }

  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(robot_status_mutex_);
    last_robot_status_ = *msg;
    have_robot_status_ = true;
  }

  bool create_messages(industrial::joint_message::JointMessage& msg_in,
                     control_msgs::FollowJointTrajectoryFeedback* control_state,
                     sensor_msgs::JointState* sensor_state) override
  {
    bool ok = DynamicJointsRelayHandler::create_messages(msg_in,
                                                        control_state,
                                                        sensor_state);

    if (!ok) {
      return false;
    }

    if (this->is_traj_is_moving_)
    {
      bool new_in_motion = this->is_moving_;

      std::lock_guard<std::mutex> lock(robot_status_mutex_);

      if (have_robot_status_)
      {
        auto new_val = new_in_motion
          ? industrial_msgs::TriState::TRUE
          : industrial_msgs::TriState::FALSE;

        if (last_robot_status_.in_motion.val != new_val)
        {
          ROS_WARN("[Dynamic Joints handler] Publishing robot status as in_motion changed");
          last_robot_status_.in_motion.val = new_val;
          last_robot_status_.header.stamp = ros::Time::now();
          robot_status_pub_.publish(last_robot_status_);
        }
      } else{
        ROS_WARN("NO OLD ROBOT STATUS");
      }
    }

    return true;
  }

   bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    // correct for parallel linkage effects, if desired
    //   - use NEGATIVE factor for motor->joint correction
    fanuc::utils::linkage_transform(pos_in, pos_out, J23_factor_);
    if(!this->is_state_radian_){
      const double deg_to_rad = M_PI / 180.0;
      std::transform(pos_out->begin(), pos_out->end(), pos_out->begin(),
                    [deg_to_rad](double x) { return x * deg_to_rad; });
    }
    return true;
  }


};

void signal_handler(int signal) {
  if (signal == SIGTERM) {
    std::cerr << "SIGTERM received\n";
    std::exit(EXIT_FAILURE);
  } else {
    std::cerr << "Unexpected signal " << signal << " received\n";
  }
}

int main(int argc, char** argv)
{
  // exit gracefully on rosnode kill
  std::signal(SIGTERM, signal_handler);

  // initialize node
  ros::init(argc, argv, "state_interface");
  ROS_WARN("[Dynamic Joints handler] Will publish robot status on in_motion change");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Fanuc_RobotStateInterface rsi;
  rsi.init();
  rsi.negotiateConnection();

  // replace the JointRelayHandler with Fanuc-version
  Fanuc_JointRelayHandler jointHandler;  // for joint-linkage correction
  jointHandler.is_state_radian_ = rsi.is_state_radian_;
  jointHandler.is_traj_is_moving_ = rsi.is_traj_is_moving_;
  jointHandler.number_axis_state_ = rsi.number_axis_state_;
  jointHandler.number_external_axis_state_ = rsi.number_external_axis_state_;
  jointHandler.number_of_joints = jointHandler.number_axis_state_ + jointHandler.number_external_axis_state_;

  std::vector<std::string> joint_names = rsi.get_joint_names();
  jointHandler.init(rsi.get_connection(), joint_names);
  rsi.add_handler(&jointHandler);
  rsi.run();

  return 0;
}
