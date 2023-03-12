#include <sensor_msgs/NavSatStatus.h>

#include "../../include/dh_gazebo_plugins/gps_plugin.hpp"
#include "../../include/dh_gazebo_plugins/utils.hpp"

using namespace std;

namespace gazebo
{
GazeboGpsPlugin::GazeboGpsPlugin() : SensorPlugin(), random_generator_(random_device_())
{
}

void GazeboGpsPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  // Store the pointer to the parent sensor.
  parent_sensor_ = dynamic_pointer_cast<sensors::GpsSensor>(sensor);
  world_ = physics::get_world(parent_sensor_->WorldName());

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  string link_name;

  if (sdf->HasElement("robotNamespace"))
  {
    ns_ = sdf->GetElement("robotNamespace")->Get<string>();
  }
  else
  {
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("linkName"))
  {
    link_name = sdf->GetElement("linkName")->Get<string>();
  }
  else
  {
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";
  }

  // Get the pointer to the link that holds the sensor.
  link_ = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name));
  if (link_ == NULL)
  {
    gzerr << "[gazebo_gps_plugin] Couldn't find specified link \"" << link_name << "\"\n";
  }

  getSdfParam<string>(sdf, "gpsTopic", gps_topic_, defaultGpsTopic);
  getSdfParam<string>(sdf, "groundSpeedTopic", ground_speed_topic_, defaultGroundSpeedTopic);

  double hor_pos_std_dev;
  double ver_pos_std_dev;
  double hor_vel_std_dev;
  double ver_vel_std_dev;
  getSdfParam<double>(sdf, "horPosStdDev", hor_pos_std_dev, defaultHorPosStdDev);
  getSdfParam<double>(sdf, "verPosStdDev", ver_pos_std_dev, defaultVerPosStdDev);
  getSdfParam<double>(sdf, "horVelStdDev", hor_vel_std_dev, defaultHorVelStdDev);
  getSdfParam<double>(sdf, "verVelStdDev", ver_vel_std_dev, defaultVerVelStdDev);

  // Connect to the sensor update event.
  this->updateConnection_ =
    this->parent_sensor_->ConnectUpdated(boost::bind(&GazeboGpsPlugin::onUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  // Initialize the normal distributions for ground speed.
  ground_speed_n_[0] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[1] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[2] = NormalDistribution(0, ver_vel_std_dev);

  // ============================================ //
  // ======= POPULATE STATIC PARTS OF MSGS ====== //
  // ============================================ //

  // Fill the static parts of the GPS message.
  gz_gps_message_.header.frame_id = link_name;
  gz_gps_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gz_gps_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gz_gps_message_.position_covariance_type = PosTopic::COVARIANCE_TYPE_KNOWN;

  gz_gps_message_.position_covariance[0] = sqr(hor_pos_std_dev);
  gz_gps_message_.position_covariance[1] = 0.;
  gz_gps_message_.position_covariance[2] = 0.;
  gz_gps_message_.position_covariance[3] = 0.;
  gz_gps_message_.position_covariance[4] = sqr(hor_pos_std_dev);
  gz_gps_message_.position_covariance[5] = 0.;
  gz_gps_message_.position_covariance[6] = 0.;
  gz_gps_message_.position_covariance[7] = 0.;
  gz_gps_message_.position_covariance[8] = sqr(ver_pos_std_dev);

  // Fill the static parts of the ground speed message.
  gz_ground_speed_message_.header.frame_id = link_name;

  gz_ground_speed_message_.vel.covariance[0] = sqr(hor_vel_std_dev);
  gz_ground_speed_message_.vel.covariance[1] = 0.;
  gz_ground_speed_message_.vel.covariance[2] = 0.;
  gz_ground_speed_message_.vel.covariance[3] = 0.;
  gz_ground_speed_message_.vel.covariance[4] = sqr(hor_vel_std_dev);
  gz_ground_speed_message_.vel.covariance[5] = 0.;
  gz_ground_speed_message_.vel.covariance[6] = 0.;
  gz_ground_speed_message_.vel.covariance[7] = 0.;
  gz_ground_speed_message_.vel.covariance[8] = sqr(ver_vel_std_dev);

  // Advertise publishers
  gz_gps_pub_ = nh_.advertise<PosTopic>("/" + ns_ + "/" + gps_topic_, 1);
  gz_ground_speed_pub_ = nh_.advertise<VelTopic>("/" + ns_ + "/" + ground_speed_topic_, 1);
}

void GazeboGpsPlugin::onUpdate()
{
  // Get the time of the last measurement.
  common::Time current_time = parent_sensor_->LastMeasurementTime();

  // Get the linear velocity in the world frame.
  ignition::math::Vector3d W_ground_speed_W_L = link_->WorldLinearVel();

  // Apply noise to ground speed.
  W_ground_speed_W_L += ignition::math::Vector3d(
    ground_speed_n_[0](random_generator_), ground_speed_n_[1](random_generator_),
    ground_speed_n_[2](random_generator_));

  // Fill the GPS message.
  // TODO: ノイズを加える
  gz_gps_message_.latitude = parent_sensor_->Latitude().Degree();
  gz_gps_message_.longitude = parent_sensor_->Longitude().Degree();
  gz_gps_message_.altitude = parent_sensor_->Altitude();
  gz_gps_message_.header.stamp.sec = current_time.sec;
  gz_gps_message_.header.stamp.nsec = current_time.nsec;

  // Fill the ground speed message.
  gz_ground_speed_message_.vel.vel.vx = W_ground_speed_W_L.X();
  gz_ground_speed_message_.vel.vel.vy = W_ground_speed_W_L.Y();
  gz_ground_speed_message_.vel.vel.vz = W_ground_speed_W_L.Z();
  gz_ground_speed_message_.header.stamp.sec = current_time.sec;
  gz_ground_speed_message_.header.stamp.nsec = current_time.nsec;

  // Publish the GPS message.
  gz_gps_pub_.publish(gz_gps_message_);

  // Publish the ground speed message.
  gz_ground_speed_pub_.publish(gz_ground_speed_message_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}  // namespace gazebo
