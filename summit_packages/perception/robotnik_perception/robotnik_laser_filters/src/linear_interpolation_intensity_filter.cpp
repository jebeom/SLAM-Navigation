#include <robotnik_laser_filters/linear_interpolation_intensity_filter.h>
#include <ros/node_handle.h>

namespace robotnik_laser_filters
{
LinearInterpolationLaserScanIntensityFilter::LinearInterpolationLaserScanIntensityFilter()
{
}

bool LinearInterpolationLaserScanIntensityFilter::configure()
{
  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<LinearInterpolationIntensityFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<LinearInterpolationIntensityFilterConfig>::CallbackType f;
  f = boost::bind(&LinearInterpolationLaserScanIntensityFilter::reconfigureCB, this, _1, _2);
  dyn_server_->setCallback(f);

  getParam("min_interpolated_intensity", config_.min_interpolated_intensity);
  
  dyn_server_->updateConfig(config_);
  return true;
}

bool LinearInterpolationLaserScanIntensityFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
{
  int min_intensity = *std::min_element(input_scan.intensities.begin(), input_scan.intensities.end());
  int max_intensity = *std::max_element(input_scan.intensities.begin(), input_scan.intensities.end());

  filtered_scan = input_scan;

  // Need to check ever reading in the current scan
  for (unsigned int i=0; i < input_scan.ranges.size() && i < input_scan.intensities.size(); i++)
  {
    float& range = filtered_scan.ranges[i];
    float& intensity = filtered_scan.intensities[i];

    double new_intesity = interpolate(intensity, min_intensity, max_intensity, 0, 100);
    if ( new_intesity < config_.min_interpolated_intensity)
    {
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = 0;
    }
  }

  return true;
}

double LinearInterpolationLaserScanIntensityFilter::interpolate(double value, double old_min, double old_max, double new_min, double new_max) {
  double old_range = old_max - old_min;
  double new_range = new_max - new_min;
  
  if (old_range == 0) {
    return 0;
  }
  
  double new_value = (value - old_min)*(new_max-new_min)/(old_max-old_min) + new_min;
  
  return new_value;
}

void LinearInterpolationLaserScanIntensityFilter::reconfigureCB(LinearInterpolationIntensityFilterConfig& config, uint32_t level)
{
  config_ = config;
}
}
