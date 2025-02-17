-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "robot_map",
  tracking_frame = "robot_imu_link",
  published_frame = "robot_odom",
  odom_frame = "robot_odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- MAP_BUILDER.use_trajectory_builder_3d = true
-- MAP_BUILDER.num_background_threads = 7
-- POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 100
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 5 --10
-- POSE_GRAPH.constraint_builder.min_score = 0.4 -- 0.5
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45 --0.55
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 4e2 --5 for sim because gazebo translation does not reflect correctly
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2 --5 for sim because gazebo rotation does not reflect correctly

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 700
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1000
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.25
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.15
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = true
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
return options
