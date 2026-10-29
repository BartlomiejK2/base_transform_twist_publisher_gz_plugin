# base_odometry_publisher_gz_plugin
Gazebo/ROS2 plugin that publishes ground truth odometry (`nav_msgs/msg/Odometry` on `/base_odometry`) of URDF base: pose (translation + rotation) in the reference frame and twist (linear and angular velocity) in the base frame. The base pose is also broadcast on TF.

```xml
<plugin filename="BaseOdometryPublisher" name="gazebo_plugins::BaseOdometryPublisher">
  <base_frame_name>base_link</base_frame_name>
  <reference_frame_name>world</reference_frame_name>
  <max_publish_frequency>100</max_publish_frequency>
</plugin>
```
