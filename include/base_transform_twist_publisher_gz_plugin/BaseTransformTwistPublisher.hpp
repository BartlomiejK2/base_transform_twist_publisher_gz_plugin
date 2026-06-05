// Copyright 2025, Bartłomiej Krajewski

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

// http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __BASE_TRANSFORM_TWIST_PUBLISHER_GZ_PLUGIN__
#define __BASE_TRANSFORM_TWIST_PUBLISHER_GZ_PLUGIN__

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace gazebo_plugins
{
  class BaseTransformTwistPublisher:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    public:
      void Configure(const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

      void PostUpdate(const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

    private:
      rclcpp::Node::SharedPtr node_;

      rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transformPublisher_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twistPublisher_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster_;

      gz::sim::Entity baseEntity_{gz::sim::kNullEntity};
      gz::sim::Entity robotEntity_{gz::sim::kNullEntity};

      std::string baseFrameName_;
      std::string referenceFrameName_{"world"};
  };
} // namespace gazebo_plugins
#endif