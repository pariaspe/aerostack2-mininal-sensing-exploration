/*!*******************************************************************************************
 *  \file       takeoff_behavior.hpp
 *  \brief      Takeoff behavior class header file
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef TAKE_OFF_BEHAVIOR_HPP
#define TAKE_OFF_BEHAVIOR_HPP

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/take_off.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "takeoff_base.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TakeOffBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::TakeOff> {
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;
  using PSME              = as2_msgs::msg::PlatformStateMachineEvent;

  TakeOffBehavior(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~TakeOffBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

  bool sendEventFSME(const int8_t _event);

  bool process_goal(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal,
                    as2_msgs::action::TakeOff::Goal &new_goal);

  bool on_activate(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> &message) override;
  bool on_pause(const std::shared_ptr<std::string> &message) override;
  bool on_resume(const std::shared_ptr<std::string> &message) override;
  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> &goal,
      std::shared_ptr<as2_msgs::action::TakeOff::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::TakeOff::Result> &result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus &state) override;

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
  std::shared_ptr<takeoff_base::TakeOffBase> takeoff_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::chrono::nanoseconds tf_timeout;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
      platform_cli_;
};

#endif  // TAKE_OFF_BEHAVIOR_HPP
