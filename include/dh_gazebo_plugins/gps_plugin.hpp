#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3WithCovarianceStamped.h>

namespace gazebo
{
// Default values
static constexpr char defaultGpsTopic[] = "gps";
static constexpr char defaultGroundSpeedTopic[] = "ground_speed";
static constexpr double defaultHorPosStdDev = 3.;
static constexpr double defaultVerPosStdDev = 6.;
static constexpr double defaultHorVelStdDev = 0.1;
static constexpr double defaultVerVelStdDev = 0.1;

class GazeboGpsPlugin : public SensorPlugin
{
public:
  using NormalDistribution = std::normal_distribution<double>;
  using PosTopic = sensor_msgs::NavSatFix;
  using VelTopic = geometry_msgs::Vector3WithCovarianceStamped;

  GazeboGpsPlugin();

protected:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  void onUpdate();

private:
  ros::NodeHandle nh_;

  ros::Publisher gz_gps_pub_;
  ros::Publisher gz_ground_speed_pub_;

  std::string ns_;                  // Namespace, read from SDF file.
  std::string gps_topic_;           // Name of topic for GPS messages, read from SDF file.
  std::string ground_speed_topic_;  // Name of topic for ground speed messages, read from SDF file.

  sensors::GpsSensorPtr parent_sensor_;    // Pointer to the parent sensor
  physics::WorldPtr world_;                // Pointer to the world.
  physics::LinkPtr link_;                  // Pointer to the sensor link.
  event::ConnectionPtr updateConnection_;  // Pointer to the update event connection.

  PosTopic gz_gps_message_;           // GPS message to be published on sensor update.
  VelTopic gz_ground_speed_message_;  // Ground speed message to be published on sensor update.

  NormalDistribution ground_speed_n_[3];  // Normal distributions for ground speed noise.
  std::random_device random_device_;      // Random device.
  std::mt19937 random_generator_;         // Random number generator.
};
}  // namespace gazebo
