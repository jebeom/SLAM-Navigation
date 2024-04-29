#pragma once

#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <robotnik_laser_filters/LinearInterpolationIntensityFilterConfig.h>
#include <sensor_msgs/LaserScan.h>

namespace robotnik_laser_filters
{
class LinearInterpolationLaserScanIntensityFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  LinearInterpolationLaserScanIntensityFilter();
  bool configure();
  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan);

private:
  double interpolate(double value, double old_min, double old_max, double new_min, double new_max);

  std::shared_ptr<dynamic_reconfigure::Server<LinearInterpolationIntensityFilterConfig>> dyn_server_;
  void reconfigureCB(LinearInterpolationIntensityFilterConfig& config, uint32_t level);
  boost::recursive_mutex own_mutex_;

  LinearInterpolationIntensityFilterConfig config_ = LinearInterpolationIntensityFilterConfig::__getDefault__();
};
}
