/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Álvaro Villena, Robotnik Automation
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef FILL_GAPS_FILTER_H
#define FILL_GAPS_FILTER_H
/**
\author Álvaro Villena
@b FillGapsFilter takes input scans and set inf or nan values to upper_replacement_value_,
 only if there are valid measurements before and after that index.
**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class LaserScanFillGapsFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  float upper_replacement_value_ ;

  bool configure()
  {
    // work around the not implemented getParam(std::string name, float& value) method
    double temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    getParam("upper_replacement_value", temp_replacement_value);
    upper_replacement_value_ = static_cast<float>(temp_replacement_value);

    return true;
  }

  virtual ~LaserScanFillGapsFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan;

    int input_scan_size = input_scan.ranges.size();
    int lower_valid_index = input_scan_size;
    int upper_valid_index = 0;


    if(input_scan_size > 0)
    {
      for (unsigned int i=0; i < input_scan_size / 2; i++)
      {
        if(lower_valid_index == input_scan_size)
        {
          if(filtered_scan.ranges[i] != std::numeric_limits<float>::infinity() && !isnan(filtered_scan.ranges[i]))
          {
            lower_valid_index = i;
          }
        }
        if(upper_valid_index == 0)
        {
          if(filtered_scan.ranges[input_scan_size - i- 1] != std::numeric_limits<float>::infinity() && !isnan(filtered_scan.ranges[i]))
          {
            upper_valid_index = input_scan_size - i- 1;
          }
        }

        if(lower_valid_index < upper_valid_index)
        {
          if(filtered_scan.ranges[i] == std::numeric_limits<float>::infinity() || isnan(filtered_scan.ranges[i]))
          {
            filtered_scan.ranges[i] = upper_replacement_value_;
          }
          if(filtered_scan.ranges[input_scan_size - i- 1] == std::numeric_limits<float>::infinity() || isnan(filtered_scan.ranges[input_scan_size - i- 1]))
          {
            filtered_scan.ranges[input_scan_size - i- 1] = upper_replacement_value_;
          }
        }
      }
    }

    return true;
  }
} ;

}

#endif // FILL_GAPS_FILTER_H
