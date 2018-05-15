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

#include <fanuc_driver/fanuc_utils.h>

#include <industrial_robot_client/joint_trajectory_streamer.h>

#include <simple_message/joint_traj_pt.h>

#include <stdexcept>


using industrial_robot_client::joint_trajectory_streamer::JointTrajectoryStreamer;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
typedef industrial::joint_traj_pt::JointTrajPt rbt_JointTrajPt;
typedef trajectory_msgs::JointTrajectoryPoint  ros_JointTrajPt;


class Fanuc_JointTrajectoryStreamer : public JointTrajectoryStreamer
{
  int J23_factor_;
  bool override_velocity_ = false;
  double fixed_override_ = 0.1;


public:
  Fanuc_JointTrajectoryStreamer() : JointTrajectoryStreamer(), J23_factor_(0)
  {
    if (!ros::param::has("J23_factor"))
    {
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
  }


  virtual ~Fanuc_JointTrajectoryStreamer() {}

  bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtMessage>* msgs) override
  {
    msgs->clear();

    // check for valid trajectory
    if (!is_valid(*traj))
      return false;

    // guestimate velocity if only two points are given
    double velocity_guess = 0;
    if (traj->points.size() >= 2) {
        size_t n = traj->points.size() - 1;
        double duration = (traj->points[n].time_from_start - traj->points[n-1].time_from_start).toSec();
        const std::vector<double>& x0 = traj->points[n-1].positions;
        const std::vector<double>& x1 = traj->points[n].positions;
        for (int i = 0; i<x1.size(); ++i) {
            double dx = std::abs(x1[i]-x0[i]);
            double v = dx/duration;
            double v_rel = v / joint_vel_limits_[traj->joint_names[i]];
            if (v_rel > velocity_guess)
                velocity_guess = std::min(v_rel, 1.0);
        }

        ROS_ERROR("Velocity estimate is %f", velocity_guess*100);

    }

    for (size_t i=0; i<traj->points.size(); ++i)
    {
      ros_JointTrajPt rbt_pt, xform_pt;
      double vel, duration;

      // select / reorder joints for sending to robot
      if (!select(traj->joint_names, traj->points[i], this->all_joint_names_, &rbt_pt))
        return false;

      // transform point data (e.g. for joint-coupling)
      if (!transform(rbt_pt, &xform_pt))
        return false;

      if (override_velocity_) {
          vel = fixed_override_;
      } else {
          if (i == traj->points.size()-1) {
              calc_duration(xform_pt, &duration);

              vel = velocity_guess;
          } else {
              // reduce velocity to a single scalar, for robot command
              if (!calc_speed(xform_pt, &vel, &duration))
                return false;
          }
      }

      JointTrajPtMessage msg = create_message(i, xform_pt.positions, vel, duration);
      msgs->push_back(msg);
    }

    for (auto& msg: *msgs) {
        std::cout << " " << msg.point_.getVelocity();
    }
    std::cout << std::endl;

    return true;
  }

  bool transform(const trajectory_msgs::JointTrajectoryPoint& pt_in,
      trajectory_msgs::JointTrajectoryPoint* pt_out)
  {
    // sending points back to the Fanuc, so invert factor
    fanuc::utils::linkage_transform(pt_in, pt_out, -J23_factor_);

    return true;
  }

  static JointTrajPtMessage create_message(int seq, std::vector<double> joint_pos, double velocity, double duration)
  {
    industrial::joint_data::JointData pos;
    ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

    for (size_t i=0; i<joint_pos.size(); ++i)
      pos.setJoint(i, joint_pos[i]);

    rbt_JointTrajPt pt;
    pt.init(seq, pos, velocity, duration);

    JointTrajPtMessage msg;
    msg.init(pt);

    return msg;
  }

};


int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "motion_interface");

  // launch the default JointTrajectoryStreamer connection/handlers
  Fanuc_JointTrajectoryStreamer motionInterface;

  motionInterface.init();
  motionInterface.run();

  return 0;
}
