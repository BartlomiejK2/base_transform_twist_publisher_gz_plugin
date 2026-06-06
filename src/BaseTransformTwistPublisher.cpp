#include <base_transform_twist_publisher_gz_plugin/BaseTransformTwistPublisher.hpp>

#include <gz/plugin/Register.hh>

#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <gz/sim/Link.hh>


namespace gazebo_plugins
{
  void BaseTransformTwistPublisher::Configure(const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager&)
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("base_pose_twist_publisher");

    transformPublisher_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>(
      "base_transfrom", rclcpp::QoS(1).best_effort().keep_last(1));

    twistPublisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "base_twist", rclcpp::QoS(1).best_effort().keep_last(1));

    transformBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

    const std::string maybeBaseName = _sdf->Get<std::string>("base_frame_name");
    
    if(!maybeBaseName.empty())
    {
      baseFrameName_ = _sdf->Get<std::string>("base_frame_name");
    }
    else
    {
      std::string errorMessage = "No <base_frame_name> defined!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage.c_str());
      throw std::runtime_error(errorMessage );
    }

    const std::string maybeReferenceName = _sdf->Get<std::string>("reference_frame_name");
    if(!maybeReferenceName.empty())
    {
      referenceFrameName_ = _sdf->Get<std::string>("reference_frame_name");
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "No <reference_frame_name> defined, using \"%s\"!", referenceFrameName_.c_str());
    }

    _ecm.Each<gz::sim::components::Link, gz::sim::components::Name>([&](
      const gz::sim::Entity &_entity,
      const gz::sim::components::Link *,
      const gz::sim::components::Name *_name)
      {
        if (_name->Data() == baseFrameName_)
        {
          baseEntity_ = _entity;
          return false;
        }
        return true;
      });

    if(baseEntity_ == gz::sim::kNullEntity)
    {
      std::string errorMessage = "Could not find " + baseFrameName_ + " frame!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage);
      throw std::runtime_error(errorMessage);
    }


    RCLCPP_INFO(node_->get_logger(), "Tracking: %s", baseFrameName_.c_str());

    robotEntity_ = _ecm.ParentEntity(baseEntity_);

    if(robotEntity_ == gz::sim::kNullEntity)
    {
      std::string errorMessage = "Could not find robot entity!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage);
      throw std::runtime_error(errorMessage);
    }

    auto world = _ecm.ParentEntity(robotEntity_);

    if(world != gz::sim::kNullEntity)
    {
      std::string errorMessage = "Given base frame is not true base frame!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage);
      throw std::runtime_error(errorMessage);
    }

    // Turn on velocity checker for this link
    gz::sim::Link gzLink_ = gz::sim::Link(baseEntity_);
    gzLink_.EnableVelocityChecks(_ecm);
  }

  void BaseTransformTwistPublisher::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
  {
    const auto poseComp = _ecm.Component<gz::sim::components::Pose>(robotEntity_);

    const auto linVelComp = _ecm.Component<gz::sim::components::LinearVelocity>(
      baseEntity_);

    const auto angVelComp =_ecm.Component<gz::sim::components::AngularVelocity>(
      baseEntity_);

    if(!poseComp || !linVelComp || !angVelComp)
    {
      std::string errorMessage = "Cannot get pose, linear or angular velocity of base!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage);
      return;
    }

    const auto stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count());

    geometry_msgs::msg::TransformStamped baseTransform;
    baseTransform.header.stamp = stamp;
    baseTransform.header.frame_id = referenceFrameName_;
    baseTransform.child_frame_id = baseFrameName_;

    const auto& pose = poseComp->Data();

    baseTransform.transform.translation.x = pose.Pos().X();
    baseTransform.transform.translation.y = pose.Pos().Y();
    baseTransform.transform.translation.z = pose.Pos().Z();

    baseTransform.transform.rotation.x = pose.Rot().X();
    baseTransform.transform.rotation.y = pose.Rot().Y();
    baseTransform.transform.rotation.z = pose.Rot().Z();
    baseTransform.transform.rotation.w = pose.Rot().W();

    geometry_msgs::msg::TwistStamped baseTwist;
    baseTwist.header.stamp = stamp;
    baseTwist.header.frame_id = baseFrameName_;

    const auto& lin = linVelComp->Data();
    const auto& ang = angVelComp->Data();

    baseTwist.twist.linear.x = lin.X();
    baseTwist.twist.linear.y = lin.Y();
    baseTwist.twist.linear.z = lin.Z();

    baseTwist.twist.angular.x = ang.X();
    baseTwist.twist.angular.y = ang.Y();
    baseTwist.twist.angular.z = ang.Z();

    transformPublisher_->publish(baseTransform);
    twistPublisher_->publish(baseTwist);
    transformBroadcaster_->sendTransform(baseTransform);
  }
} // namespace gazebo_plugins

IGNITION_ADD_PLUGIN(
    gazebo_plugins::BaseTransformTwistPublisher,
    ignition::gazebo::System,
    gazebo_plugins::BaseTransformTwistPublisher::ISystemConfigure,
    gazebo_plugins::BaseTransformTwistPublisher::ISystemPostUpdate)