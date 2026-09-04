/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

#include "industrial_utils/param_utils.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "simple_message/joint_data.h"
#include <fanuc_driver/fanuc_utils.h>
#include "fanuc_joint_trajectory_downloader.h"
#include "simple_message/classes/mikado_connection_info.h"
#include "simple_message/messages/mikado_connection_info_message.h"
#include "simple_message/classes/mikado_dynamic_joints_traj_pt.h"
#include "simple_message/messages/mikado_dynamic_joints_traj_pt_message.h"
#include "simple_message/classes/mikado_dynamic_joints.h"
#include <csignal>


using industrial_robot_client::joint_trajectory_downloader::FanucJointTrajectoryDownloader;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;


static constexpr char const* name_interpolation_max_joint_difference = "interpolation_max_joint_difference";

class FanucDynamicJointTrajectoryDownloader : public FanucJointTrajectoryDownloader
{
  using FanucJointTrajectoryDownloader::init;  // so base-class init() stays visible

  int J23_factor_;
  bool override_velocity_ = false;
  double fixed_override_ = 0.1;

  ros::Publisher pub_joint_control_state_;

public:

  ~FanucDynamicJointTrajectoryDownloader()
  {
    // running trajectory stop here instead of in base class destructor
    trajectoryStop();
  }

  bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION)
  {
    if (!FanucJointTrajectoryDownloader::init(default_ip, default_port)){  // call base-class init()
      throw std::runtime_error("Initialzing FanucDynamicJointTrajectoryDownloader failed.");
    }

   if (!ros::param::has("J23_factor")){
      ROS_FATAL("Joint 2-3 linkage factor parameter not supplied.");
      throw std::runtime_error("Cannot find required parameter 'J23_factor' on parameter server.");
    }

    ros::param::get("J23_factor", this->J23_factor_);

    if (ros::param::has("velocity_override"))
    {
        ros::param::get("velocity_override", fixed_override_);
        fixed_override_ = std::min(1.0, std::max(0.0, fixed_override_));
        override_velocity_ = true;
        ROS_INFO("Using fixed velocity override, ignoring calculated/set velocities. Using %i%%", int(fixed_override_*100.0));
    }

    // trajectory interpolation is handled by FanucJointTrajectoryInterface

    return true;
  }

  bool negotiateConnection(){
    industrial::simple_message::SimpleMessage msg, reply;
    industrial::simple_message::mikado_classes::MikadoConnectionInfo mikConnectionInfo;
    industrial::simple_message::mikado_messages::MikadoConnectionInfoMessage mikConnectionInfoMsg;

    if(!this->connection_){
      ROS_ERROR("[Joint Downloader] Cannot send connection information as no connection has been initialized");
      return false;
    }

    if (!this->connection_->isConnected()) {
      ROS_ERROR("[Joint Downloader] Cannot send connection information as no connection has been established");
      return false;
    }

    int number_axis_trajectory = 6;
    int number_external_axis_trajectory = 0;
    bool is_traj_radian = false;
    bool is_traj_velocity = false;
    bool is_traj_duration = false;
    ros::NodeHandle handle;
    // read ros_params in, if not present, use defaults
    std::string prefix = "/robot_description_manipulators/manipulator/";
    handle.param(prefix + "connection_info_number_axes_trajectory", number_axis_trajectory, number_axis_trajectory);
    handle.param(prefix + "connection_info_number_external_axes_trajectory", number_external_axis_trajectory, number_external_axis_trajectory);
    handle.param(prefix + "connection_info_is_traj_radian", is_traj_radian, is_traj_radian);
    handle.param(prefix + "connection_info_is_traj_velocity", is_traj_velocity, is_traj_velocity);
    handle.param(prefix + "connection_info_is_traj_duration", is_traj_duration, is_traj_duration);

    mikConnectionInfo.init(number_axis_trajectory, 0, number_external_axis_trajectory,
                            0, is_traj_radian, false, is_traj_velocity,
                            is_traj_duration, false);
    mikConnectionInfoMsg.init(mikConnectionInfo);
    mikConnectionInfoMsg.toRequest(msg);
    bool res = this->connection_->sendMsg(msg);
    if (res){
      this->number_axis_trajectory_ = number_axis_trajectory;
      this->number_external_axis_trajectory_ = number_external_axis_trajectory;
      this->is_traj_radian_ = is_traj_radian;
      this->is_traj_velocity_ = is_traj_velocity;
      this->is_traj_duration_ = is_traj_duration;
      ROS_INFO("Sent connection information. Trajectory connection parameters:\n"
                "  number_axis_trajectory: %d\n"
                "  number_external_axis_trajectory: %d\n"
                "  is_traj_radian: %s\n"
                "  is_traj_velocity: %s\n"
                "  is_traj_duration: %s\n",
                number_axis_trajectory,
                number_external_axis_trajectory,
                is_traj_radian ? "true" : "false",
                is_traj_velocity ? "true" : "false",
                is_traj_duration ? "true" : "false"
              );
    } else {
      ROS_WARN("Failed to send connection information");
    }
    return res;
  }

  bool transform(const trajectory_msgs::JointTrajectoryPoint& pt_in,
      trajectory_msgs::JointTrajectoryPoint* pt_out)
  {
    // sending points back to the Fanuc, so invert factor
    fanuc::utils::linkage_transform(pt_in, pt_out, -J23_factor_);
    if(!this->is_traj_radian_){
        const double deg_to_rad = 180.0 / M_PI;
        std::transform(pt_out->positions.begin(), pt_out->positions.end(), pt_out->positions.begin(),
                      [deg_to_rad](double x) { return x * deg_to_rad; });
    }

    return true;
  }

  bool calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity)
  {
    *rbt_velocity = 0;  // currently not used by fanuc driver
    return true;
  }

  bool send_to_robot(const std::vector<industrial::joint_traj_pt_message::JointTrajPtMessage>& messages)
  {
    bool rslt=true;
    std::vector<industrial::joint_traj_pt_message::JointTrajPtMessage> points(messages);
    std::vector<industrial::simple_message::mikado_messages::MikadoDynamicJointsTrajPtMessage> dynamicPoints;
    industrial::simple_message::mikado_messages::MikadoDynamicJointsTrajPtMessage mikDynamicJointsTrajPtMessage;
    industrial::simple_message::mikado_classes::MikadoDynamicJointsTrajPt mikDynamicJointsTrajPt;
    industrial::simple_message::mikado_classes::MikadoDynamicJoints mikDynamicJoints;
    mikDynamicJoints.init((this->number_axis_trajectory_+ this->number_external_axis_trajectory_));
    mikDynamicJointsTrajPt.init();
    mikDynamicJointsTrajPt.setPositions(mikDynamicJoints);
    if(this->is_traj_duration_){
      mikDynamicJointsTrajPt.setField(industrial::simple_message::mikado_classes::DynamicJointsValidFieldTypes::DURATION);
    } else {
      mikDynamicJointsTrajPt.setFieldInvalid(industrial::simple_message::mikado_classes::DynamicJointsValidFieldTypes::DURATION);
    }
    if(this->is_traj_velocity_){
      mikDynamicJointsTrajPt.setField(industrial::simple_message::mikado_classes::DynamicJointsValidFieldTypes::VELOCITY);
    } else {
      mikDynamicJointsTrajPt.setFieldInvalid(industrial::simple_message::mikado_classes::DynamicJointsValidFieldTypes::VELOCITY);
    }
    industrial::simple_message::SimpleMessage msg;

    // Trajectory download requires at least two points (START/END)
    if (points.size() < 2){
      points.push_back(industrial::joint_traj_pt_message::JointTrajPtMessage(points[0]));
    }

    // The first and last points are assigned special sequence values
    points.begin()->setSequence(industrial::joint_traj_pt::SpecialSeqValues::START_TRAJECTORY_DOWNLOAD);
    points.back().setSequence(industrial::joint_traj_pt::SpecialSeqValues::END_TRAJECTORY);

    if (!this->connection_->isConnected())
    {
      ROS_WARN("Attempting robot reconnection");
      if(this->connection_->makeConnect()){
        this->negotiateConnection();
      };
    }

    ROS_INFO("Sending trajectory points, size: %d", (int)points.size());
    industrial::joint_data::JointData jd;


    for (int i = 0; i < (int)points.size(); ++i)
    {
      points[i].point_.getJointPosition(jd);
      ROS_DEBUG("Sending joints trajectory point[%d]", i);
      for (int j = 0; j < (this->number_axis_trajectory_ + this->number_external_axis_trajectory_); j++){
        mikDynamicJoints.setJoint(j, jd.getJoint(j));
      }
      mikDynamicJointsTrajPtMessage.point_.setPositions(mikDynamicJoints);
      mikDynamicJointsTrajPtMessage.point_.setSequence(points[i].point_.getSequence());
      if(this->is_traj_velocity_){
        mikDynamicJointsTrajPtMessage.point_.setField(DynamicJointsValidFieldTypes::VELOCITY);
        mikDynamicJointsTrajPtMessage.point_.setVelocity(points[i].point_.getVelocity());
      }
      if(this->is_traj_velocity_){
        mikDynamicJointsTrajPtMessage.point_.setField(DynamicJointsValidFieldTypes::DURATION);
        mikDynamicJointsTrajPtMessage.point_.setDuration(points[i].point_.getDuration());
      }
      bool ptRslt = false;
      if(i == 0){
        industrial::simple_message::SimpleMessage reply;
        mikDynamicJointsTrajPtMessage.toRequest(msg);
        ROS_WARN("Sending point0 of type: %d", msg.getCommType());
        ptRslt = this->connection_->sendAndReceiveMsg(msg, reply);
        if(!ptRslt){
          ROS_WARN("Attempting robot reconnection");
          if(this->connection_->makeConnect()){
            ROS_WARN("Reconnect successfull, trying to send connection info message");
            if(this->negotiateConnection()){
              ROS_WARN("Connection info message sent. Trying to resend point");
              ptRslt = this->connection_->sendAndReceiveMsg(msg, reply);
            } else {
              ROS_WARN("Could not send connection info message");
            }
          } else{
            ROS_WARN("Failed to send point. Could not reconnect.");
            return false;
          }
        }
      } else{
        mikDynamicJointsTrajPtMessage.toTopic(msg);
        ptRslt = this->connection_->sendMsg(msg);
      }
      if (ptRslt) {
        ROS_WARN("Point[%d] sent to controller", i);
      } else{
        ROS_WARN("Failed sent joint point, skipping point");
      }

      rslt &= ptRslt;
    }

    return rslt;
  }

  void trajectoryStop()
  {
    industrial::simple_message::mikado_classes::MikadoDynamicJoints mikDynamicJoints;
    industrial::simple_message::mikado_messages::MikadoDynamicJointsTrajPtMessage dyJtsMsg;
    industrial::simple_message::SimpleMessage msg, reply;
    mikDynamicJoints.setNumJoints(this->number_axis_trajectory_ + this->number_external_axis_trajectory_);
    if(this->is_traj_duration_){
      dyJtsMsg.point_.setField(DynamicJointsValidFieldTypes::DURATION);
    }
    if(this->is_traj_velocity_){
      dyJtsMsg.point_.setField(DynamicJointsValidFieldTypes::VELOCITY);
    }
    dyJtsMsg.point_.setPositions(mikDynamicJoints);

    ROS_INFO("Dynamic Joints trajectory handler: entering stopping state");
    dyJtsMsg.setSequence(industrial::joint_traj_pt::SpecialSeqValues::STOP_TRAJECTORY);
    dyJtsMsg.toRequest(msg);
    ROS_DEBUG("Sending stop command");
    this->connection_->sendAndReceiveMsg(msg, reply);
  }

  private:
    int number_axis_trajectory_;
    int number_external_axis_trajectory_;
    bool is_traj_radian_;
    bool is_traj_velocity_;
    bool is_traj_duration_;
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
  ros::init(argc, argv, "motion_interface");

  FanucDynamicJointTrajectoryDownloader motionInterface;
  bool connection_established = false;
  while (!connection_established){
    if(motionInterface.init()){
      connection_established = motionInterface.negotiateConnection();
      if(!connection_established){
        ROS_WARN("[Fanuc Joint Downloader] Failed to establish connection. Trying again.");
        sleep(5);
      }
    }
  }
  ROS_WARN("[Fanuc Joint Downloader] Established connection");
  motionInterface.run();

  return 0;
}
