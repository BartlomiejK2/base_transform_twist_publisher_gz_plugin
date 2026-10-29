#include <base_odometry_publisher_gz_plugin/BaseOdometryPublisher.hpp>

#include <gz/plugin/Register.hh>

#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <gz/sim/Link.hh>


namespace gazebo_plugins
{
  void BaseOdometryPublisher::Configure(const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager&)
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("base_odometry_publisher");

    // Set use_sim_time
    node_->set_parameter(rclcpp::Parameter("use_sim_time", true));

    odometryPublisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(1).best_effort().keep_last(1));

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
      throw std::runtime_error(errorMessage);
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

    const auto publishFrequencyString = _sdf->Get<std::string>("max_publish_frequency");
    if(!publishFrequencyString.empty())
    {
      const double publishFrequency = std::stod(publishFrequencyString);
      publishDuration_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / publishFrequency));
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "No <max_publish_frequency> defined, using 100Hz");
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
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage.c_str());
      throw std::runtime_error(errorMessage);
    }


    RCLCPP_INFO(node_->get_logger(), "Tracking: %s", baseFrameName_.c_str());

    robotEntity_ = _ecm.ParentEntity(baseEntity_);

    if(robotEntity_ == gz::sim::kNullEntity)
    {
      std::string errorMessage = "Could not find robot entity!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage.c_str());
      throw std::runtime_error(errorMessage);
    }

    auto world = _ecm.ParentEntity(robotEntity_);

    if(world != gz::sim::kNullEntity)
    {
      std::string errorMessage = "Given base frame is not true base frame!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage.c_str());
      throw std::runtime_error(errorMessage);
    }

    // Turn on velocity checker for this link
    gz::sim::Link gzLink_ = gz::sim::Link(baseEntity_);
    gzLink_.EnableVelocityChecks(_ecm);
    
  }

  void BaseOdometryPublisher::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
  {
    if((_info.simTime - lastPublishTime_) < publishDuration_)
    {
      // Simulation to fast
      return;
    }

    lastPublishTime_ = _info.simTime;

    const auto poseComp = _ecm.Component<gz::sim::components::Pose>(robotEntity_);

    const auto linVelComp = _ecm.Component<gz::sim::components::LinearVelocity>(
      baseEntity_);

    const auto angVelComp =_ecm.Component<gz::sim::components::AngularVelocity>(
      baseEntity_);

    if(!poseComp || !linVelComp || !angVelComp)
    {
      std::string errorMessage = "Cannot get pose, linear or angular velocity of base!";
      RCLCPP_ERROR(node_->get_logger(), "%s", errorMessage.c_str());
      return;
    }

    const auto stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count());

    // Pose is expressed in reference frame, twist in base frame (nav_msgs/Odometry convention)
    nav_msgs::msg::Odometry baseOdometry;
    baseOdometry.header.stamp = stamp;
    baseOdometry.header.frame_id = referenceFrameName_;
    baseOdometry.child_frame_id = baseFrameName_;

    const auto& pose = poseComp->Data();

    baseOdometry.pose.pose.position.x = pose.Pos().X();
    baseOdometry.pose.pose.position.y = pose.Pos().Y();
    baseOdometry.pose.pose.position.z = pose.Pos().Z();

    baseOdometry.pose.pose.orientation.x = pose.Rot().X();
    baseOdometry.pose.pose.orientation.y = pose.Rot().Y();
    baseOdometry.pose.pose.orientation.z = pose.Rot().Z();
    baseOdometry.pose.pose.orientation.w = pose.Rot().W();

    const auto& lin = linVelComp->Data();
    const auto& ang = angVelComp->Data();

    baseOdometry.twist.twist.linear.x = lin.X();
    baseOdometry.twist.twist.linear.y = lin.Y();
    baseOdometry.twist.twist.linear.z = lin.Z();

    baseOdometry.twist.twist.angular.x = ang.X();
    baseOdometry.twist.twist.angular.y = ang.Y();
    baseOdometry.twist.twist.angular.z = ang.Z();

    geometry_msgs::msg::TransformStamped baseTransform;
    baseTransform.header = baseOdometry.header;
    baseTransform.child_frame_id = baseOdometry.child_frame_id;

    baseTransform.transform.translation.x = baseOdometry.pose.pose.position.x;
    baseTransform.transform.translation.y = baseOdometry.pose.pose.position.y;
    baseTransform.transform.translation.z = baseOdometry.pose.pose.position.z;
    baseTransform.transform.rotation = baseOdometry.pose.pose.orientation;

    odometryPublisher_->publish(baseOdometry);
    transformBroadcaster_->sendTransform(baseTransform);
  }
} // namespace gazebo_plugins

IGNITION_ADD_PLUGIN(
    gazebo_plugins::BaseOdometryPublisher,
    ignition::gazebo::System,
    gazebo_plugins::BaseOdometryPublisher::ISystemConfigure,
    gazebo_plugins::BaseOdometryPublisher::ISystemPostUpdate)