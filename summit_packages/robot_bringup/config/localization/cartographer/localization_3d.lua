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

include "slam_3d.lua"


-- Numero de mini transformadas que se crean
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 20

-- POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.35 * POSE_GRAPH.constraint_builder.sampling_ratio
POSE_GRAPH.max_num_final_iterations = 1

POSE_GRAPH.global_constraint_search_after_n_seconds = 10

--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 700
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 1000
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 700
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1000

--TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.62
--TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5

--TRAJECTORY_BUILDER_3D.min_range = 1
--TRAJECTORY_BUILDER_3D.max_range = 30
--TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1
--TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1


-- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80
--TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.67

--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = true
--POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false

return options

