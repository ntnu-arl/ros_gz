// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "factories.hpp"
#include "ros_ign_bridge/convert.hpp"

namespace ros_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_impl(
  const std::string & ros_type_name,
  const std::string & ign_type_name)
{
  // mapping from string to specialized template
  if (
    (ros_type_name == "std_msgs/Bool" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Boolean")
  {
    return std::make_shared<
      Factory<
        std_msgs::Bool,
        gz::msgs::Boolean
      >
    >("std_msgs/Bool", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/ColorRGBA" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Color")
  {
    return std::make_shared<
      Factory<
        std_msgs::ColorRGBA,
        gz::msgs::Color
      >
    >("std_msgs/ColorRGBA", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Empty" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Empty")
  {
    return std::make_shared<
      Factory<
        std_msgs::Empty,
        gz::msgs::Empty
      >
    >("std_msgs/Empty", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Int32" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Int32")
  {
    return std::make_shared<
      Factory<
        std_msgs::Int32,
        gz::msgs::Int32
      >
    >("std_msgs/Int32", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Float32" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Float")
  {
    return std::make_shared<
      Factory<
        std_msgs::Float32,
        gz::msgs::Float
      >
    >("std_msgs/Float32", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Float64" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Double")
  {
    return std::make_shared<
      Factory<
        std_msgs::Float64,
        gz::msgs::Double
      >
    >("std_msgs/Float64", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/Header" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Header")
  {
    return std::make_shared<
      Factory<
        std_msgs::Header,
        gz::msgs::Header
      >
    >("std_msgs/Header", ign_type_name);
  }
  if (
    (ros_type_name == "std_msgs/String" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.StringMsg")
  {
    return std::make_shared<
      Factory<
        std_msgs::String,
        gz::msgs::StringMsg
      >
    >("std_msgs/String", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/Quaternion" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Quaternion")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Quaternion,
        gz::msgs::Quaternion
      >
    >("geometry_msgs/Quaternion", ign_type_name);
  }
  if (
    (ros_type_name == "rosgraph_msgs/Clock" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Clock")
  {
    return std::make_shared<
      Factory<
        rosgraph_msgs::Clock,
        gz::msgs::Clock
      >
    >("rosgraph_msgs/Clock", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/Vector3" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Vector3d")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Vector3,
        gz::msgs::Vector3d
      >
    >("geometry_msgs/Vector3", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/Point" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Vector3d")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Point,
        gz::msgs::Vector3d
      >
    >("geometry_msgs/Point", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/Pose" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Pose,
        gz::msgs::Pose
      >
    >("geometry_msgs/Pose", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/PoseArray" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Pose_V")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::PoseArray,
        gz::msgs::Pose_V
      >
    >("geometry_msgs/PoseArray", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/PoseStamped" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::PoseStamped,
        gz::msgs::Pose
      >
    >("geometry_msgs/PoseStamped", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/Transform" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Transform,
        gz::msgs::Pose
      >
    >("geometry_msgs/Transform", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/TransformStamped" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Pose")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::TransformStamped,
        gz::msgs::Pose
      >
    >("geometry_msgs/TransformStamped", ign_type_name);
  }
  if (
    (ros_type_name == "tf2_msgs/TFMessage" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Pose_V")
  {
    return std::make_shared<
      Factory<
        tf2_msgs::TFMessage,
        gz::msgs::Pose_V
      >
    >("tf2_msgs/TFMessage", ign_type_name);
  }
  if (
    (ros_type_name == "geometry_msgs/Twist" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Twist")
  {
    return std::make_shared<
      Factory<
        geometry_msgs::Twist,
        gz::msgs::Twist
      >
    >("geometry_msgs/Twist", ign_type_name);
  }
//  if (
//    (ros_type_name == "mav_msgs/Actuators" || ros_type_name == "") &&
//     ign_type_name == "gz.msgs.Actuators")
//  {
//    return std::make_shared<
//      Factory<
//        mav_msgs::Actuators,
//        gz::msgs::Actuators
//      >
//    >("mav_msgs/Actuators", ign_type_name);
//  }
  if (
    (ros_type_name == "nav_msgs/OccupancyGrid" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.OccupancyGrid")
  {
    return std::make_shared<
      Factory<
        nav_msgs::OccupancyGrid,
        gz::msgs::OccupancyGrid
      >
    >("nav_msgs/OccupancyGrid", ign_type_name);
  }
  if (
    (ros_type_name == "nav_msgs/Odometry" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Odometry")
  {
    return std::make_shared<
      Factory<
        nav_msgs::Odometry,
        gz::msgs::Odometry
      >
    >("nav_msgs/Odometry", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/FluidPressure" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.FluidPressure")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::FluidPressure,
        gz::msgs::FluidPressure
      >
    >("sensor_msgs/FluidPressure", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/Image" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Image")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::Image,
        gz::msgs::Image
      >
    >("sensor_msgs/Image", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/CameraInfo" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.CameraInfo")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::CameraInfo,
        gz::msgs::CameraInfo
      >
    >("sensor_msgs/CameraInfo", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/Imu" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.IMU")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::Imu,
        gz::msgs::IMU
      >
    >("sensor_msgs/Imu", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/JointState" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Model")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::JointState,
        gz::msgs::Model
      >
    >("sensor_msgs/JointState", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/LaserScan" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.LaserScan")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::LaserScan,
        gz::msgs::LaserScan
      >
    >("sensor_msgs/LaserScan", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/MagneticField" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Magnetometer")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::MagneticField,
        gz::msgs::Magnetometer
      >
    >("sensor_msgs/Magnetometer", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/NavSatFix" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.NavSat")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::NavSatFix,
        gz::msgs::NavSat
      >
    >("sensor_msgs/NavSatFix", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/PointCloud2" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.PointCloudPacked")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::PointCloud2,
        gz::msgs::PointCloudPacked
      >
    >("sensor_msgs/PointCloud2", ign_type_name);
  }
  if (
    (ros_type_name == "sensor_msgs/BatteryState" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.BatteryState")
  {
    return std::make_shared<
      Factory<
        sensor_msgs::BatteryState,
        gz::msgs::BatteryState
      >
    >("sensor_msgs/BatteryState", ign_type_name);
  }
  if (
    (ros_type_name == "visualization_msgs/Marker" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Marker")
  {
    return std::make_shared<
      Factory<
        visualization_msgs::Marker,
        gz::msgs::Marker
      >
    >("visualization_msgs/Marker", ign_type_name);
  }
  if (
    (ros_type_name == "visualization_msgs/MarkerArray" || ros_type_name == "") &&
     ign_type_name == "gz.msgs.Marker_V")
  {
    return std::make_shared<
      Factory<
        visualization_msgs::MarkerArray,
        gz::msgs::Marker_V
      >
    >("visualization_msgs/MarkerArray", ign_type_name);
  }
  return std::shared_ptr<FactoryInterface>();
}

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros_type_name,
            const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> factory;
  factory = get_factory_impl(ros_type_name, ign_type_name);
  if (factory)
    return factory;

  throw std::runtime_error("No template specialization for the pair");
}

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::Bool,
  gz::msgs::Boolean
>::convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  gz::msgs::Boolean & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Bool,
  gz::msgs::Boolean
>::convert_ign_to_ros(
  const gz::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::ColorRGBA,
  gz::msgs::Color
>::convert_ros_to_ign(
  const std_msgs::ColorRGBA & ros_msg,
  gz::msgs::Color & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::ColorRGBA,
  gz::msgs::Color
>::convert_ign_to_ros(
  const gz::msgs::Color & ign_msg,
  std_msgs::ColorRGBA & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Empty,
  gz::msgs::Empty
>::convert_ros_to_ign(
  const std_msgs::Empty & ros_msg,
  gz::msgs::Empty & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Empty,
  gz::msgs::Empty
>::convert_ign_to_ros(
  const gz::msgs::Empty & ign_msg,
  std_msgs::Empty & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Int32,
  gz::msgs::Int32
>::convert_ros_to_ign(
  const std_msgs::Int32 & ros_msg,
  gz::msgs::Int32 & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Int32,
  gz::msgs::Int32
>::convert_ign_to_ros(
  const gz::msgs::Int32 & ign_msg,
  std_msgs::Int32 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Float32,
  gz::msgs::Float
>::convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  gz::msgs::Float & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Float32,
  gz::msgs::Float
>::convert_ign_to_ros(
  const gz::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Float64,
  gz::msgs::Double
>::convert_ros_to_ign(
  const std_msgs::Float64 & ros_msg,
  gz::msgs::Double & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Float64,
  gz::msgs::Double
>::convert_ign_to_ros(
  const gz::msgs::Double & ign_msg,
  std_msgs::Float64 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::Header,
  gz::msgs::Header
>::convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  gz::msgs::Header & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::Header,
  gz::msgs::Header
>::convert_ign_to_ros(
  const gz::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  std_msgs::String,
  gz::msgs::StringMsg
>::convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  gz::msgs::StringMsg & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::String,
  gz::msgs::StringMsg
>::convert_ign_to_ros(
  const gz::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

// rosgraph_msgs
template<>
void
Factory<
  rosgraph_msgs::Clock,
  gz::msgs::Clock
>::convert_ros_to_ign(
  const rosgraph_msgs::Clock & ros_msg,
  gz::msgs::Clock & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  rosgraph_msgs::Clock,
  gz::msgs::Clock
>::convert_ign_to_ros(
  const gz::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

// geometry_msgs
template<>
void
Factory<
  geometry_msgs::Quaternion,
  gz::msgs::Quaternion
>::convert_ros_to_ign(
  const geometry_msgs::Quaternion & ros_msg,
  gz::msgs::Quaternion & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Quaternion,
  gz::msgs::Quaternion
>::convert_ign_to_ros(
  const gz::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::Vector3,
  gz::msgs::Vector3d
>::convert_ros_to_ign(
  const geometry_msgs::Vector3 & ros_msg,
  gz::msgs::Vector3d & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Vector3,
  gz::msgs::Vector3d
>::convert_ign_to_ros(
  const gz::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::Point,
  gz::msgs::Vector3d
>::convert_ros_to_ign(
  const geometry_msgs::Point & ros_msg,
  gz::msgs::Vector3d & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Point,
  gz::msgs::Vector3d
>::convert_ign_to_ros(
  const gz::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::Pose,
  gz::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::Pose & ros_msg,
  gz::msgs::Pose & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Pose,
  gz::msgs::Pose
>::convert_ign_to_ros(
  const gz::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::PoseArray,
  gz::msgs::Pose_V
>::convert_ros_to_ign(
  const geometry_msgs::PoseArray & ros_msg,
  gz::msgs::Pose_V & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::PoseArray,
  gz::msgs::Pose_V
>::convert_ign_to_ros(
  const gz::msgs::Pose_V & ign_msg,
  geometry_msgs::PoseArray & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  gz::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::PoseStamped & ros_msg,
  gz::msgs::Pose & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  gz::msgs::Pose
>::convert_ign_to_ros(
  const gz::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::Transform,
  gz::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::Transform & ros_msg,
  gz::msgs::Pose & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Transform,
  gz::msgs::Pose
>::convert_ign_to_ros(
  const gz::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  gz::msgs::Pose
>::convert_ros_to_ign(
  const geometry_msgs::TransformStamped & ros_msg,
  gz::msgs::Pose & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  gz::msgs::Pose
>::convert_ign_to_ros(
  const gz::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  tf2_msgs::TFMessage,
  gz::msgs::Pose_V
>::convert_ros_to_ign(
  const tf2_msgs::TFMessage & ros_msg,
  gz::msgs::Pose_V & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  tf2_msgs::TFMessage,
  gz::msgs::Pose_V
>::convert_ign_to_ros(
  const gz::msgs::Pose_V & ign_msg,
  tf2_msgs::TFMessage & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  geometry_msgs::Twist,
  gz::msgs::Twist
>::convert_ros_to_ign(
  const geometry_msgs::Twist & ros_msg,
  gz::msgs::Twist & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  geometry_msgs::Twist,
  gz::msgs::Twist
>::convert_ign_to_ros(
  const gz::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

// mav_msgs
//template<>
//void
//Factory<
//  mav_msgs::Actuators,
//  gz::msgs::Actuators
//>::convert_ros_to_ign(
//  const mav_msgs::Actuators & ros_msg,
//  gz::msgs::Actuators & ign_msg)
//{
//  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
//}
//
//template<>
//void
//Factory<
//  mav_msgs::Actuators,
//  gz::msgs::Actuators
//>::convert_ign_to_ros(
//  const gz::msgs::Actuators & ign_msg,
//  mav_msgs::Actuators & ros_msg)
//{
//  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
//}

// nav_msgs
template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  gz::msgs::OccupancyGrid
>::convert_ros_to_ign(
  const nav_msgs::OccupancyGrid & ros_msg,
  gz::msgs::OccupancyGrid & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  gz::msgs::OccupancyGrid
>::convert_ign_to_ros(
  const gz::msgs::OccupancyGrid & ign_msg,
  nav_msgs::OccupancyGrid & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  nav_msgs::Odometry,
  gz::msgs::Odometry
>::convert_ros_to_ign(
  const nav_msgs::Odometry & ros_msg,
  gz::msgs::Odometry & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  nav_msgs::Odometry,
  gz::msgs::Odometry
>::convert_ign_to_ros(
  const gz::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

// sensor_msgs
template<>
void
Factory<
  sensor_msgs::FluidPressure,
  gz::msgs::FluidPressure
>::convert_ros_to_ign(
  const sensor_msgs::FluidPressure & ros_msg,
  gz::msgs::FluidPressure & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::FluidPressure,
  gz::msgs::FluidPressure
>::convert_ign_to_ros(
  const gz::msgs::FluidPressure & ign_msg,
  sensor_msgs::FluidPressure & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::Image,
  gz::msgs::Image
>::convert_ros_to_ign(
  const sensor_msgs::Image & ros_msg,
  gz::msgs::Image & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::Image,
  gz::msgs::Image
>::convert_ign_to_ros(
  const gz::msgs::Image & ign_msg,
  sensor_msgs::Image & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::CameraInfo,
  gz::msgs::CameraInfo
>::convert_ros_to_ign(
  const sensor_msgs::CameraInfo & ros_msg,
  gz::msgs::CameraInfo & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::CameraInfo,
  gz::msgs::CameraInfo
>::convert_ign_to_ros(
  const gz::msgs::CameraInfo & ign_msg,
  sensor_msgs::CameraInfo & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::Imu,
  gz::msgs::IMU
>::convert_ros_to_ign(
  const sensor_msgs::Imu & ros_msg,
  gz::msgs::IMU & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::Imu,
  gz::msgs::IMU
>::convert_ign_to_ros(
  const gz::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::JointState,
  gz::msgs::Model
>::convert_ros_to_ign(
  const sensor_msgs::JointState & ros_msg,
  gz::msgs::Model & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::JointState,
  gz::msgs::Model
>::convert_ign_to_ros(
  const gz::msgs::Model & ign_msg,
  sensor_msgs::JointState & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::LaserScan,
  gz::msgs::LaserScan
>::convert_ros_to_ign(
  const sensor_msgs::LaserScan & ros_msg,
  gz::msgs::LaserScan & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::LaserScan,
  gz::msgs::LaserScan
>::convert_ign_to_ros(
  const gz::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::MagneticField,
  gz::msgs::Magnetometer
>::convert_ros_to_ign(
  const sensor_msgs::MagneticField & ros_msg,
  gz::msgs::Magnetometer & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::MagneticField,
  gz::msgs::Magnetometer
>::convert_ign_to_ros(
  const gz::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::NavSatFix,
  gz::msgs::NavSat
>::convert_ros_to_ign(
  const sensor_msgs::NavSatFix & ros_msg,
  gz::msgs::NavSat & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::NavSatFix,
  gz::msgs::NavSat
>::convert_ign_to_ros(
  const gz::msgs::NavSat & ign_msg,
  sensor_msgs::NavSatFix & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  gz::msgs::PointCloudPacked
>::convert_ros_to_ign(
  const sensor_msgs::PointCloud2 & ros_msg,
  gz::msgs::PointCloudPacked & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  gz::msgs::PointCloudPacked
>::convert_ign_to_ros(
  const gz::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::PointCloud2 & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  sensor_msgs::BatteryState,
  gz::msgs::BatteryState
>::convert_ros_to_ign(
  const sensor_msgs::BatteryState & ros_msg,
  gz::msgs::BatteryState & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  sensor_msgs::BatteryState,
  gz::msgs::BatteryState
>::convert_ign_to_ros(
  const gz::msgs::BatteryState & ign_msg,
  sensor_msgs::BatteryState & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  visualization_msgs::Marker,
  gz::msgs::Marker
>::convert_ros_to_ign(
    const visualization_msgs::Marker & ros_msg,
    gz::msgs::Marker & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  visualization_msgs::Marker,
  gz::msgs::Marker
>::convert_ign_to_ros(
    const gz::msgs::Marker & ign_msg,
    visualization_msgs::Marker & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  gz::msgs::Marker_V
>::convert_ros_to_ign(
    const visualization_msgs::MarkerArray & ros_msg,
    gz::msgs::Marker_V & ign_msg)
{
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
}

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  gz::msgs::Marker_V
>::convert_ign_to_ros(
    const gz::msgs::Marker_V & ign_msg,
    visualization_msgs::MarkerArray & ros_msg)
{
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
}

}  // namespace ros_ign_bridge
