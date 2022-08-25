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

#include "gazebo_wind_plugin.h"
#include "common.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  update_connection_->~Connection();
}

void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
  world_ = world;

  std::cout << "[gazebo_wind_plugin] Initialized wind plugin" << std::endl;

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (sdf->HasElement("xyzOffset"))
    xyz_offset_ = sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  double pub_rate = 2.0;
  getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); //Wind topic publishing rates
  pub_interval_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;
  getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
  getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
  getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(sdf, "windGustVeloctiyMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
  getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
  getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);
  // Check if a custom static wind field should be used.
  getSdfParam<bool>(sdf, "useCustomStaticWindField", use_custom_static_wind_field_,
                      use_custom_static_wind_field_);

  if(use_custom_static_wind_field_){
    std::cout<<"[gazebo_wind_plugin] custom wind field selected \n";
    std::string custom_wind_field_path;
    getSdfParam<std::string>(sdf, "customWindFieldPath", custom_wind_field_path,
                        custom_wind_field_path);
    ReadCustomWindField(custom_wind_field_path);
  }


  wind_direction_mean_.Normalize();
  wind_gust_direction_mean_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  // Set random wind velocity mean and standard deviation
  wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
  // Set random wind direction mean and standard deviation
  wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
  // Set random wind gust velocity mean and standard deviation
  wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
  // Set random wind gust direction mean and standard deviation
  wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
  #else
    last_time_ = world_->GetSimTime();
  #endif

  if(use_custom_static_wind_field_){
    // Get model and base link for position determination
    model_ = world_->ModelByName("nxp_drone");
    link_ = model_->GetLink("base_link");
    if (link_ == NULL)
      gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                << "\".");
  }
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {

// Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif
  if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
    return;
  }
  last_time_ = now;

  ignition::math::Vector3d wind(0.0, 0.0, 0.0);

  double wind_strength = std::abs(wind_velocity_distribution_(wind_velocity_generator_));
  wind_strength = (wind_strength > wind_velocity_max_) ? wind_velocity_max_ : wind_strength;
  // Get normal distribution wind direction
  ignition::math::Vector3d wind_direction;
  wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
  wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
  wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);

  double test_;

  ignition::math::Vector3d test;
  ignition::math::Vector3d wind_at_vertices_0;
  // Calculate the wind force.
  // Get normal distribution wind strength
  if(!use_custom_static_wind_field_){
    // Calculate total wind velocity
    wind = wind_strength * wind_direction;
  }
  else{
    //Get the current position of the aircraft in world coordinates.
    ignition::math::Vector3d link_position = link_->WorldPose().Pos();



    // Calculate the x, y index of the grid points with x, y-coordinate
    // just smaller than or equal to aircraft x, y position.
    std::size_t x_inf = floor((link_position.X() - min_x_) / res_x_);
    std::size_t y_inf = floor((link_position.Y() - min_y_) / res_y_);

    // In case aircraft is on one of the boundary surfaces at max_x or max_y,
    // decrease x_inf, y_inf by one to have x_sup, y_sup on max_x, max_y.
    if (x_inf == n_x_ - 1u) {
      x_inf = n_x_ - 2u;
    }
    if (y_inf == n_y_ - 1u) {
      y_inf = n_y_ - 2u;
    }

    // Calculate the x, y index of the grid points with x, y-coordinate just
    // greater than the aircraft x, y position.
    std::size_t x_sup = x_inf + 1u;
    std::size_t y_sup = y_inf + 1u;

    // Save in an array the x, y index of each of the eight grid points
    // enclosing the aircraft.
    constexpr unsigned int n_vertices = 8;
    std::size_t idx_x[n_vertices] = {x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup};
    std::size_t idx_y[n_vertices] = {y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup};

    // Find the vertical factor of the aircraft in each of the four surrounding
    // grid columns, and their minimal/maximal value.
    constexpr unsigned int n_columns = 4;
    float vertical_factors_columns[n_columns];
    for (std::size_t i = 0u; i < n_columns; ++i) {
      vertical_factors_columns[i] = (
        link_position.Z() - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]) /
        (top_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_] - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]);
    }

    // Find maximal and minimal value amongst vertical factors.
    float vertical_factors_min = std::min(std::min(std::min(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3]);
    float vertical_factors_max = std::max(std::max(std::max(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3]);

    std::size_t idx_z[n_vertices] = {0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                                        0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                                        0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                                        0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u};

    // Check if aircraft is out of wind field or not, and act accordingly.
    if (x_inf >= 0u && y_inf >= 0u && vertical_factors_max >= 0u &&
        x_sup <= (n_x_ - 1u) && y_sup <= (n_y_ - 1u) && vertical_factors_min <= 1u) {
      // Find indices in z-direction for each of the vertices. If link is not
      // within the range of one of the columns, set to lowest or highest two.

      for (std::size_t i = 0u; i < n_columns; ++i) {
        if (vertical_factors_columns[i] < 0u) {
          // Link z-position below lowest grid point of that column.
          idx_z[2u * i + 1u] = 1u;
        } else if (vertical_factors_columns[i] >= 1u) {
          // Link z-position above highest grid point of that column.
          idx_z[2u * i] = vertical_spacing_factors_.size() - 2u;
        } else {
          // Link z-position between two grid points in that column.
          for (std::size_t j = 0u; j < vertical_spacing_factors_.size() - 1u; ++j) {
            if (vertical_spacing_factors_[j] <= vertical_factors_columns[i] &&
                vertical_spacing_factors_[j + 1u] > vertical_factors_columns[i]) {
              idx_z[2u * i] = j;
              idx_z[2u * i + 1u] = j + 1u;
              break;
            }
          }
        }
      }
    }

    double test_value = u_.size();

    test.X() = test_value;
    test.Y() = 1;
    test.Z() = 1;


    ignition::math::Vector3d wind_at_vertices_1;
    ignition::math::Vector3d wind_at_vertices_2;
    ignition::math::Vector3d wind_at_vertices_3;
    ignition::math::Vector3d wind_at_vertices_4;
    ignition::math::Vector3d wind_at_vertices_5;
    ignition::math::Vector3d wind_at_vertices_6;
    ignition::math::Vector3d wind_at_vertices_7;

    wind_at_vertices_0.X() = u_[idx_x[0] + idx_y[0] * n_x_ + idx_z[0] * n_x_ * n_y_];
    wind_at_vertices_0.Y() = v_[idx_x[0] + idx_y[0] * n_x_ + idx_z[0] * n_x_ * n_y_];
    wind_at_vertices_0.Z() = w_[idx_x[0] + idx_y[0] * n_x_ + idx_z[0] * n_x_ * n_y_];
    wind_at_vertices_1.X() = u_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
    wind_at_vertices_1.Y() = v_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
    wind_at_vertices_1.Z() = w_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
    wind_at_vertices_2.X() = u_[idx_x[2] + idx_y[2] * n_x_ + idx_z[2] * n_x_ * n_y_];
    wind_at_vertices_2.Y() = v_[idx_x[2] + idx_y[2] * n_x_ + idx_z[2] * n_x_ * n_y_];
    wind_at_vertices_2.Z() = w_[idx_x[2] + idx_y[2] * n_x_ + idx_z[2] * n_x_ * n_y_];
    wind_at_vertices_3.X() = u_[idx_x[3] + idx_y[3] * n_x_ + idx_z[3] * n_x_ * n_y_];
    wind_at_vertices_3.Y() = v_[idx_x[3] + idx_y[3] * n_x_ + idx_z[3] * n_x_ * n_y_];
    wind_at_vertices_3.Z() = w_[idx_x[3] + idx_y[3] * n_x_ + idx_z[3] * n_x_ * n_y_];
    wind_at_vertices_4.X() = u_[idx_x[4] + idx_y[4] * n_x_ + idx_z[4] * n_x_ * n_y_];
    wind_at_vertices_4.Y() = v_[idx_x[4] + idx_y[4] * n_x_ + idx_z[4] * n_x_ * n_y_];
    wind_at_vertices_4.Z() = w_[idx_x[4] + idx_y[4] * n_x_ + idx_z[4] * n_x_ * n_y_];
    wind_at_vertices_5.X() = u_[idx_x[5] + idx_y[5] * n_x_ + idx_z[5] * n_x_ * n_y_];
    wind_at_vertices_5.Y() = v_[idx_x[5] + idx_y[5] * n_x_ + idx_z[5] * n_x_ * n_y_];
    wind_at_vertices_5.Z() = w_[idx_x[5] + idx_y[5] * n_x_ + idx_z[5] * n_x_ * n_y_];
    wind_at_vertices_6.X() = u_[idx_x[6] + idx_y[6] * n_x_ + idx_z[6] * n_x_ * n_y_];
    wind_at_vertices_6.Y() = v_[idx_x[6] + idx_y[6] * n_x_ + idx_z[6] * n_x_ * n_y_];
    wind_at_vertices_6.Z() = w_[idx_x[6] + idx_y[6] * n_x_ + idx_z[6] * n_x_ * n_y_];
    wind_at_vertices_7.X() = u_[idx_x[7] + idx_y[7] * n_x_ + idx_z[7] * n_x_ * n_y_];
    wind_at_vertices_7.Y() = v_[idx_x[7] + idx_y[7] * n_x_ + idx_z[7] * n_x_ * n_y_];
    wind_at_vertices_7.Z() = w_[idx_x[7] + idx_y[7] * n_x_ + idx_z[7] * n_x_ * n_y_];


    // wind_at_vertices_ptr = &wind_at_vertices;
    // std::size_t i = 0u;
    // ignition::math::Vector3d *wind_at_vertices_8 = wind_at_vertices_ptr[i];
    // wind_at_vertices_0.X() = 1; //u_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
      // wind_at_vertices[i].Y() = v_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
      // wind_at_vertices[i].Z() = w_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
    //}

    // Extract the relevant coordinate of every point needed for trilinear
    // interpolation (first z-direction, then x-direction, then y-direction).
    constexpr unsigned int n_points_interp_z = 8;
    constexpr unsigned int n_points_interp_x = 4;
    constexpr unsigned int n_points_interp_y = 2;
    double interpolation_points[n_points_interp_x + n_points_interp_y + n_points_interp_z];
    for (std::size_t i = 0u; i < n_points_interp_x + n_points_interp_y + n_points_interp_z; ++i) {
      if (i < n_points_interp_z) {
        interpolation_points[i] = (
          top_z_[idx_x[i] + idx_y[i] * n_x_] - bottom_z_[idx_x[i] + idx_y[i] * n_x_])
          * vertical_spacing_factors_[idx_z[i]] + bottom_z_[idx_x[i] + idx_y[i] * n_x_];
      } else if (i >= n_points_interp_z && i < n_points_interp_x + n_points_interp_z) {
        interpolation_points[i] = min_x_ + res_x_ * idx_x[2u * (i - n_points_interp_z)];
      } else {
        interpolation_points[i] = min_y_ + res_y_ * idx_y[4u * (i - n_points_interp_z - n_points_interp_x)];
      }

    }

    //Brute force the linear interpolation
    ignition::math::Vector3d intermediate_value_0 = wind_at_vertices_0 + (wind_at_vertices_1 - wind_at_vertices_0) /
                                                      (interpolation_points[1] - interpolation_points[0]) * (link_position[2] - interpolation_points[0]);

    ignition::math::Vector3d intermediate_value_1 = wind_at_vertices_2 + (wind_at_vertices_3 - wind_at_vertices_2) /
                                                      (interpolation_points[3] - interpolation_points[2]) * (link_position[2] - interpolation_points[2]);

    ignition::math::Vector3d intermediate_value_2 = wind_at_vertices_4 + (wind_at_vertices_5 - wind_at_vertices_4) /
                                                      (interpolation_points[5] - interpolation_points[4]) * (link_position[2] - interpolation_points[4]);

    ignition::math::Vector3d intermediate_value_3 = wind_at_vertices_6 + (wind_at_vertices_7 - wind_at_vertices_6) /
                                                      (interpolation_points[7] - interpolation_points[6]) * (link_position[2] - interpolation_points[6]);

    ignition::math::Vector3d intermediate_value_4 = intermediate_value_0 + (intermediate_value_1 - intermediate_value_0) /
                                                      (interpolation_points[9] - interpolation_points[8]) * (link_position[0] - interpolation_points[8]);

    ignition::math::Vector3d intermediate_value_5 = intermediate_value_2 + (intermediate_value_3 - intermediate_value_2) /
                                                      (interpolation_points[11] - interpolation_points[10]) * (link_position[0] - interpolation_points[10]);

    ignition::math::Vector3d intermediate_value_6 = intermediate_value_4 + (intermediate_value_5 - intermediate_value_4) /
                                                      (interpolation_points[13] - interpolation_points[12]) * (link_position[1] - interpolation_points[12]);

      // wind_direction = intermediate_value_6;

    // else{
    //   wind = wind_strength * wind_direction;
    // }
       // (0, 0, 0); //intermediate_value_6;
      //Test(wind_at_vertices_2);
      //wind_velocity = Test(wind_at_vertices_2); //LinearInterpolation(
      //link_position[0], wind_at_vertices_0, wind_at_vertices_1, interpolation_points[0], interpolation_points[1]);
    //} else {
      // Set the wind velocity to the default constant value specified by user.
    //  wind = wind_strength * wind_direction;
    //}
    wind = test;
  }

  ignition::math::Vector3d wind_gust(0, 0, 0);
  // Calculate the wind gust velocity.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    // Get normal distribution wind gust strength
    double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
    wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_ : wind_gust_strength;
    // Get normal distribution wind gust direction
    ignition::math::Vector3d wind_gust_direction;
    wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
    wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
    wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
    wind_gust = wind_gust_strength * wind_gust_direction;
  }

  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(wind.X() + wind_gust.Z());
  wind_v->set_y(wind.Y() + wind_gust.Y());
  wind_v->set_z(wind.Z() + wind_gust.Z());

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  // std::cout << wind.X() << std::endl;

  wind_pub_->Publish(wind_msg);

  // if(use_custom_static_wind_field_){
  //   // Get the current position of the aircraft in world coordinates.
  //   ignition::math::Vector3d link_position = link_->WorldPose().Pos();

  //   // Calculate the x, y index of the grid points with x, y-coordinate
  //   // just smaller than or equal to aircraft x, y position.
  //   std::size_t x_inf = floor((link_position.X() - min_x_) / res_x_);
  //   std::size_t y_inf = floor((link_position.Y() - min_y_) / res_y_);

  //   // In case aircraft is on one of the boundary surfaces at max_x or max_y,
  //   // decrease x_inf, y_inf by one to have x_sup, y_sup on max_x, max_y.
  //   if (x_inf == n_x_ - 1u) {
  //     x_inf = n_x_ - 2u;
  //   }
  //   if (y_inf == n_y_ - 1u) {
  //     y_inf = n_y_ - 2u;
  //   }

  //   // Calculate the x, y index of the grid points with x, y-coordinate just
  //   // greater than the aircraft x, y position.
  //   std::size_t x_sup = x_inf + 1u;
  //   std::size_t y_sup = y_inf + 1u;

  //   // Save in an array the x, y index of each of the eight grid points
  //   // enclosing the aircraft.
  //   constexpr unsigned int n_vertices = 8;
  //   std::size_t idx_x[n_vertices] = {x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup};
  //   std::size_t idx_y[n_vertices] = {y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup};

  //   // Find the vertical factor of the aircraft in each of the four surrounding
  //   // grid columns, and their minimal/maximal value.
  //   constexpr unsigned int n_columns = 4;
  //   float vertical_factors_columns[n_columns];
  //   for (std::size_t i = 0u; i < n_columns; ++i) {
  //     vertical_factors_columns[i] = (
  //       link_position.Z() - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]) /
  //       (top_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_] - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]);
  //   }

  //   // Find maximal and minimal value amongst vertical factors.
  //   float vertical_factors_min = std::min(std::min(std::min(
  //     vertical_factors_columns[0], vertical_factors_columns[1]),
  //     vertical_factors_columns[2]), vertical_factors_columns[3]);
  //   float vertical_factors_max = std::max(std::max(std::max(
  //     vertical_factors_columns[0], vertical_factors_columns[1]),
  //     vertical_factors_columns[2]), vertical_factors_columns[3]);

  //   // Check if aircraft is out of wind field or not, and act accordingly.
  //   if (x_inf >= 0u && y_inf >= 0u && vertical_factors_max >= 0u &&
  //       x_sup <= (n_x_ - 1u) && y_sup <= (n_y_ - 1u) && vertical_factors_min <= 1u) {
  //     // Find indices in z-direction for each of the vertices. If link is not
  //     // within the range of one of the columns, set to lowest or highest two.
  //     std::size_t idx_z[n_vertices] = {0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
  //                             0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
  //                             0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
  //                             0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u};
  //     for (std::size_t i = 0u; i < n_columns; ++i) {
  //       if (vertical_factors_columns[i] < 0u) {
  //         // Link z-position below lowest grid point of that column.
  //         idx_z[2u * i + 1u] = 1u;
  //       } else if (vertical_factors_columns[i] >= 1u) {
  //         // Link z-position above highest grid point of that column.
  //         idx_z[2u * i] = vertical_spacing_factors_.size() - 2u;
  //       } else {
  //         // Link z-position between two grid points in that column.
  //         for (std::size_t j = 0u; j < vertical_spacing_factors_.size() - 1u; ++j) {
  //           if (vertical_spacing_factors_[j] <= vertical_factors_columns[i] &&
  //               vertical_spacing_factors_[j + 1u] > vertical_factors_columns[i]) {
  //             idx_z[2u * i] = j;
  //             idx_z[2u * i + 1u] = j + 1u;
  //             break;
  //           }
  //         }
  //       }
  //     }

  //     ignition::math::Vector3d wind_at_vertices_0;
  //     ignition::math::Vector3d wind_at_vertices_1;
  //     ignition::math::Vector3d wind_at_vertices_2;
  //     ignition::math::Vector3d wind_at_vertices_3;
  //     ignition::math::Vector3d wind_at_vertices_4;
  //     ignition::math::Vector3d wind_at_vertices_5;
  //     ignition::math::Vector3d wind_at_vertices_6;
  //     ignition::math::Vector3d wind_at_vertices_7;

  //     wind_at_vertices_0.X() = u_[idx_x[0] + idx_y[0] * n_x_ + idx_z[0] * n_x_ * n_y_];
  //     wind_at_vertices_0.Y() = v_[idx_x[0] + idx_y[0] * n_x_ + idx_z[0] * n_x_ * n_y_];
  //     wind_at_vertices_0.Z() = w_[idx_x[0] + idx_y[0] * n_x_ + idx_z[0] * n_x_ * n_y_];
  //     wind_at_vertices_1.X() = u_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
  //     wind_at_vertices_1.Y() = v_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
  //     wind_at_vertices_1.Z() = w_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
  //     wind_at_vertices_2.X() = u_[idx_x[2] + idx_y[2] * n_x_ + idx_z[2] * n_x_ * n_y_];
  //     wind_at_vertices_2.Y() = v_[idx_x[2] + idx_y[2] * n_x_ + idx_z[2] * n_x_ * n_y_];
  //     wind_at_vertices_2.Z() = w_[idx_x[2] + idx_y[2] * n_x_ + idx_z[2] * n_x_ * n_y_];
  //     wind_at_vertices_3.X() = u_[idx_x[3] + idx_y[3] * n_x_ + idx_z[3] * n_x_ * n_y_];
  //     wind_at_vertices_3.Y() = v_[idx_x[3] + idx_y[3] * n_x_ + idx_z[3] * n_x_ * n_y_];
  //     wind_at_vertices_3.Z() = w_[idx_x[3] + idx_y[3] * n_x_ + idx_z[3] * n_x_ * n_y_];
  //     wind_at_vertices_4.X() = u_[idx_x[4] + idx_y[4] * n_x_ + idx_z[4] * n_x_ * n_y_];
  //     wind_at_vertices_4.Y() = v_[idx_x[4] + idx_y[4] * n_x_ + idx_z[4] * n_x_ * n_y_];
  //     wind_at_vertices_4.Z() = w_[idx_x[4] + idx_y[4] * n_x_ + idx_z[4] * n_x_ * n_y_];
  //     wind_at_vertices_5.X() = u_[idx_x[5] + idx_y[5] * n_x_ + idx_z[5] * n_x_ * n_y_];
  //     wind_at_vertices_5.Y() = v_[idx_x[5] + idx_y[5] * n_x_ + idx_z[5] * n_x_ * n_y_];
  //     wind_at_vertices_5.Z() = w_[idx_x[5] + idx_y[5] * n_x_ + idx_z[5] * n_x_ * n_y_];
  //     wind_at_vertices_6.X() = u_[idx_x[6] + idx_y[6] * n_x_ + idx_z[6] * n_x_ * n_y_];
  //     wind_at_vertices_6.Y() = v_[idx_x[6] + idx_y[6] * n_x_ + idx_z[6] * n_x_ * n_y_];
  //     wind_at_vertices_6.Z() = w_[idx_x[6] + idx_y[6] * n_x_ + idx_z[6] * n_x_ * n_y_];
  //     wind_at_vertices_7.X() = u_[idx_x[7] + idx_y[7] * n_x_ + idx_z[7] * n_x_ * n_y_];
  //     wind_at_vertices_7.Y() = v_[idx_x[7] + idx_y[7] * n_x_ + idx_z[7] * n_x_ * n_y_];
  //     wind_at_vertices_7.Z() = w_[idx_x[7] + idx_y[7] * n_x_ + idx_z[7] * n_x_ * n_y_];

  //     // wind_at_vertices_ptr = &wind_at_vertices;
  //     // std::size_t i = 0u;
  //     // ignition::math::Vector3d *wind_at_vertices_8 = wind_at_vertices_ptr[i];
  //     // wind_at_vertices_0.X() = 1; //u_[idx_x[1] + idx_y[1] * n_x_ + idx_z[1] * n_x_ * n_y_];
  //       // wind_at_vertices[i].Y() = v_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
  //       // wind_at_vertices[i].Z() = w_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
  //     //}

  //     // Extract the relevant coordinate of every point needed for trilinear
  //     // interpolation (first z-direction, then x-direction, then y-direction).
  //     constexpr unsigned int n_points_interp_z = 8;
  //     constexpr unsigned int n_points_interp_x = 4;
  //     constexpr unsigned int n_points_interp_y = 2;
  //     double interpolation_points[n_points_interp_x + n_points_interp_y + n_points_interp_z];
  //     for (std::size_t i = 0u; i < n_points_interp_x + n_points_interp_y + n_points_interp_z; ++i) {
  //       if (i < n_points_interp_z) {
  //         interpolation_points[i] = (
  //           top_z_[idx_x[i] + idx_y[i] * n_x_] - bottom_z_[idx_x[i] + idx_y[i] * n_x_])
  //           * vertical_spacing_factors_[idx_z[i]] + bottom_z_[idx_x[i] + idx_y[i] * n_x_];
  //       } else if (i >= n_points_interp_z && i < n_points_interp_x + n_points_interp_z) {
  //         interpolation_points[i] = min_x_ + res_x_ * idx_x[2u * (i - n_points_interp_z)];
  //       } else {
  //         interpolation_points[i] = min_y_ + res_y_ * idx_y[4u * (i - n_points_interp_z - n_points_interp_x)];
  //       }
  //     }

  //     //Brute force the linear interpolation
  //     ignition::math::Vector3d intermediate_value_0 = wind_at_vertices_0 + (wind_at_vertices_1 - wind_at_vertices_0) /
  //                                                       (interpolation_points[1] - interpolation_points[0]) * (link_position[2] - interpolation_points[0]);

  //     ignition::math::Vector3d intermediate_value_1 = wind_at_vertices_2 + (wind_at_vertices_3 - wind_at_vertices_2) /
  //                                                       (interpolation_points[3] - interpolation_points[2]) * (link_position[2] - interpolation_points[2]);

  //     ignition::math::Vector3d intermediate_value_2 = wind_at_vertices_4 + (wind_at_vertices_5 - wind_at_vertices_4) /
  //                                                       (interpolation_points[5] - interpolation_points[4]) * (link_position[2] - interpolation_points[4]);

  //     ignition::math::Vector3d intermediate_value_3 = wind_at_vertices_6 + (wind_at_vertices_7 - wind_at_vertices_6) /
  //                                                       (interpolation_points[7] - interpolation_points[6]) * (link_position[2] - interpolation_points[6]);

  //     ignition::math::Vector3d intermediate_value_4 = intermediate_value_0 + (intermediate_value_1 - intermediate_value_0) /
  //                                                       (interpolation_points[9] - interpolation_points[8]) * (link_position[0] - interpolation_points[8]);

  //     ignition::math::Vector3d intermediate_value_5 = intermediate_value_2 + (intermediate_value_3 - intermediate_value_2) /
  //                                                       (interpolation_points[11] - interpolation_points[10]) * (link_position[0] - interpolation_points[10]);

  //     ignition::math::Vector3d intermediate_value_6 = intermediate_value_4 + (intermediate_value_5 - intermediate_value_4) /
  //                                                       (interpolation_points[13] - interpolation_points[12]) * (link_position[1] - interpolation_points[12]);

  //     wind_velocity = intermediate_value_6;
  //     //Test(wind_at_vertices_2);
  //     //wind_velocity = Test(wind_at_vertices_2); //LinearInterpolation(
  //     //link_position[0], wind_at_vertices_0, wind_at_vertices_1, interpolation_points[0], interpolation_points[1]);
  //   } else {
  //     // Set the wind velocity to the default constant value specified by user.
  //     wind_velocity = wind_strength * wind_direction;
  //   }
  // }
}

void GazeboWindPlugin::ReadCustomWindField(std::string& custom_wind_field_path){
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
     // Read the line with the variable name.
    while (fin >> data_name) {
      // Save data on following line into the correct variable.
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      } else if (data_name == "vertical_spacing_factors:") {
        while (fin >> data) {
          vertical_spacing_factors_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "top_z:") {
        while (fin >> data) {
          top_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "u:") {
        while (fin >> data) {
          u_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "v:") {
        while (fin >> data) {
          v_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "w:") {
        while (fin >> data) {
          w_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else {
        // If invalid data name, read the rest of the invalid line,
        // publish a message and ignore data on next line. Then resume reading.
        std::string restOfLine;
        getline(fin, restOfLine);
        gzerr << " [gazebo_wind_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    gzdbg << "[gazebo_wind_plugin] Successfully read custom wind field from text file.\n";
  } else {
    gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
  }
}

// ignition::math::Vector3d GazeboWindPlugin::LinearInterpolation(
//   double position, ignition::math::Vector3d value0, ignition::math::Vector3d value1, double point0, double point1) const {
//   //ignition::math::Vector3d value;
//   // value = value0 + (value1 - value0) /
//   //                       (point0 - point1) * (position - point0);
//   //return value;
//   return value0;
// }

// void GazeboWindPlugin::Test(
//   ignition::math::Vector3d value0){
//   value = value0;
// }

// ignition::math::Vector3d GazeboWindPlugin::BilinearInterpolation(
//   double* position, ignition::math::Vector3d value0, ignition::math::Vector3d value1,
//   ignition::math::Vector3d value2, ignition::math::Vector3d value3, double* points) const {
//   ignition::math::Vector3d intermediate_value0 = LinearInterpolation(
//                                              position[0], value0, value1, &(points[0]));
//   ignition::math::Vector3d intermediate_value1 = LinearInterpolation(
//                                              position[0], value2, value3, &(points[2]));
//   ignition::math::Vector3d value = LinearInterpolation(
//                           position[1], intermediate_value0, intermediate_value1, &(points[4]));
//   return value;
// }

// ignition::math::Vector3d GazeboWindPlugin::TrilinearInterpolation(
//   ignition::math::Vector3d link_position, ignition::math::Vector3d value0, ignition::math::Vector3d value1,
//   ignition::math::Vector3d value2, ignition::math::Vector3d value3, ignition::math::Vector3d value4,
//   ignition::math::Vector3d value5, ignition::math::Vector3d value6, ignition::math::Vector3d value7, double* points) const {
//   double position[3] = {link_position.X(),link_position.Y(),link_position.Z()};
//   ignition::math::Vector3d intermediate_value0 = LinearInterpolation(
//                                              position[2], value0, value1, &(points[0]));
//   ignition::math::Vector3d intermediate_value1 = LinearInterpolation(
//                                              position[2], value2, value3, &(points[2]));
//   ignition::math::Vector3d intermediate_value2 = LinearInterpolation(
//                                              position[2], value4, value5, &(points[4]));
//   ignition::math::Vector3d intermediate_value3 = LinearInterpolation(
//                                              position[2], value6, value7, &(points[6]));
//   ignition::math::Vector3d value = BilinearInterpolation(
//     &(position[0]), intermediate_value0, intermediate_value1, intermediate_value2, intermediate_value3, &(points[8]));
//   return value;
// }

GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);
}
