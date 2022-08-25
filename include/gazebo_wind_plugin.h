/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Wind.pb.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

static constexpr double kDefaultWindVelocityMean = 0.0;
static constexpr double kDefaultWindVelocityMax = 100.0;
static constexpr double kDefaultWindVelocityVariance = 0.0;
static constexpr double kDefaultWindGustVelocityMean = 0.0;
static constexpr double kDefaultWindGustVelocityMax = 10.0;
static constexpr double kDefaultWindGustVelocityVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
static constexpr double kDefaultWindDirectionVariance = 0.0;
static constexpr double kDefaultWindGustDirectionVariance = 0.0;
static constexpr bool kDefaultUseCustomStaticWindField = false;

/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public WorldPlugin {
 public:
  GazeboWindPlugin()
      : WorldPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_("world_wind"),
        wind_velocity_mean_(kDefaultWindVelocityMean),
        wind_velocity_max_(kDefaultWindVelocityMax),
        wind_velocity_variance_(kDefaultWindVelocityVariance),
        wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
        wind_gust_velocity_max_(kDefaultWindGustVelocityMax),
        wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
        wind_direction_mean_(kDefaultWindDirectionMean),
        wind_direction_variance_(kDefaultWindDirectionVariance),
        wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
        wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
        use_custom_static_wind_field_(kDefaultUseCustomStaticWindField),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        pub_interval_(0.5),
        node_handle_(NULL) {}

  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /// \brief Pointer to the update event connection.


  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_pub_topic_;

  double wind_velocity_mean_;
  double wind_velocity_max_;
  double wind_velocity_variance_;
  double wind_gust_velocity_mean_;
  double wind_gust_velocity_max_;
  double wind_gust_velocity_variance_;
  double pub_interval_;
  std::default_random_engine wind_velocity_generator_;
  std::normal_distribution<double> wind_velocity_distribution_;
  std::default_random_engine wind_gust_velocity_generator_;
  std::normal_distribution<double> wind_gust_velocity_distribution_;

  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d wind_direction_mean_;
  ignition::math::Vector3d wind_gust_direction_mean_;
  double wind_direction_variance_;
  double wind_gust_direction_variance_;
  bool use_custom_static_wind_field_;
  float min_x_;
  float min_y_;
  int n_x_;
  int n_y_;
  float res_x_;
  float res_y_;
  std::vector<float> vertical_spacing_factors_;
  std::vector<float> bottom_z_;
  std::vector<float> top_z_;
  std::vector<float> u_;
  std::vector<float> v_;
  std::vector<float> w_;
  void ReadCustomWindField(std::string& custom_wind_field_path);

  //void Test(ignition::math::Vector3d value);

  ignition::math::Vector3d LinearInterpolation(double position, ignition::math::Vector3d value0, ignition::math::Vector3d value1, double point0, double point1) const;
  ignition::math::Vector3d BilinearInterpolation(double* position, ignition::math::Vector3d value0, ignition::math::Vector3d value1, ignition::math::Vector3d value2, ignition::math::Vector3d value3, double* points) const;
  ignition::math::Vector3d TrilinearInterpolation(ignition::math::Vector3d link_position, ignition::math::Vector3d value0, ignition::math::Vector3d value1, ignition::math::Vector3d value2, ignition::math::Vector3d value3, ignition::math::Vector3d value4, ignition::math::Vector3d value5, ignition::math::Vector3d value6, ignition::math::Vector3d value7, double* points) const;

  std::default_random_engine wind_direction_generator_;
  std::normal_distribution<double> wind_direction_distribution_X_;
  std::normal_distribution<double> wind_direction_distribution_Y_;
  std::normal_distribution<double> wind_direction_distribution_Z_;
  std::default_random_engine wind_gust_direction_generator_;
  std::normal_distribution<double> wind_gust_direction_distribution_X_;
  std::normal_distribution<double> wind_gust_direction_distribution_Y_;
  std::normal_distribution<double> wind_gust_direction_distribution_Z_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;
  common::Time last_time_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;

  physics_msgs::msgs::Wind wind_msg;
  ignition::math::Vector3d wind_at_vertices[8];
  ignition::math::Vector3d * wind_at_vertices_ptr[8];


  ignition::math::Vector3d value;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
