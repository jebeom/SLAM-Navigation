
#include <robotnik_laser_filters/linear_interpolation_intensity_filter.h>

#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(robotnik_laser_filters::LinearInterpolationLaserScanIntensityFilter, filters::FilterBase<sensor_msgs::LaserScan>)