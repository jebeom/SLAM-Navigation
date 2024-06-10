/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  lower_resolution_filter.h
 *
 *  author: Francesc Folch Company <frafolcm@gmail.com>
 */

#ifndef LOWER_RESOLUTION_FILTER_H
#define LOWER_RESOLUTION_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters
{
/** \brief A class to provide median filtering of laser scans in time*/
class LowerResolutionFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  int reduce_points_factor_;
  int reduce_frequency_factor_;

  ~LowerResolutionFilter()
  {
  }

  /** \brief Configure filter loading parameter to reduce_points_factor_
   */
  bool configure()
  {
    if (!getParam("reduce_points_factor", reduce_points_factor_))
    {
      ROS_ERROR("Cannot configure LowerResolutionFilter: Didn't find 'reduce_points_factor' paramerer.");
      return false;
    }
    ROS_INFO("Reduce number of points by a factor of %d", reduce_points_factor_);

    return true;
  }

  /** \brief Update the filter and get the response
   * \param scan_in The new scan to filter
   * \param scan_out The filtered scan
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
  {
    if (!this->configured_)
    {
      ROS_ERROR("LaserArrayFilter not configured");
      return false;
    }

    scan_out = scan_in;

    int original_length = scan_in.ranges.size();
    int reduced_length = std::floor(original_length / reduce_points_factor_);
    int index_last_element = original_length - reduce_points_factor_;

    scan_out.ranges.resize(reduced_length);
    scan_out.intensities.resize(reduced_length);

    int j = 0;
    for (int i = 0; i <= index_last_element; i += reduce_points_factor_)
    {
      scan_out.ranges[j] = scan_in.ranges[i];
      scan_out.intensities[j] = scan_in.intensities[i];
      ++j;
    }

    scan_out.angle_increment = scan_in.angle_increment * reduce_points_factor_;
    scan_out.angle_max = scan_in.angle_max - scan_out.angle_increment;

    return true;
  }
};
};  // namespace laser_filters
#endif