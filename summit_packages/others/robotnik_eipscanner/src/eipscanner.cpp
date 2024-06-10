#include <fstream> 
#include <bitset>
#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>

#include <EIPScanner/DiscoveryManager.h>
#include <EIPScanner/ParameterObject.h>
#include <EIPScanner/FileObject.h>
#include <utils/Logger.h>
#include <EIPScanner/ConnectionManager.h>
#include <cip/connectionManager/NetworkConnectionParams.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_named_digital_output.h> 
#include <robotnik_msgs/SafetyModuleStatus.h>
#include <robotnik_msgs/SetLaserMode.h> 
#include <robotnik_msgs/LaserMode.h>
#include <robotnik_eipscanner/eipscanner.h>
#include <robotnik_eipscanner/eip_configuration.h>

using namespace std::chrono_literals;
using eipScanner::DiscoveryManager;

using eipScanner::IdentityObject;
using eipScanner::SessionInfo;
using std::bitset;

using eipScanner::MessageRouter;
using eipScanner::cip::ServiceCodes;
using eipScanner::cip::EPath;
using eipScanner::cip::CipUint;
using eipScanner::cip::CipInt;
using eipScanner::cip::CipByte;
using eipScanner::cip::CipDword;
using eipScanner::cip::GeneralStatusCodes;
using eipScanner::ParameterObject;
using eipScanner::utils::Buffer;
using eipScanner::FileObject;
using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;
using eipScanner::ConnectionManager;
using eipScanner::cip::connectionManager::ConnectionParameters;
using eipScanner::cip::connectionManager::NetworkConnectionParams;

namespace robotnik_eipscanner {

EIPScanner::EIPScanner(ros::NodeHandle h) : RComponent(h), nh_(h), pnh_("~"), ip_address_("0.0.0.0")
{
  Logger::setLogLevel(LogLevel::DEBUG);
  rosReadParams();

  output_bits_.assign(eip_configuration_input_.OutputSize(), false);
  RCOMPONENT_INFO_STREAM("output_bits_.size()=" << output_bits_.size());
  component_name.assign(pnh_.getNamespace());

  readParam(pnh_, "ip_address", ip_address_, ip_address_);

  // Topics
  inputs_pub_topic_name_ = "inputs";
  inputs_msg_.digital_inputs.assign(eip_configuration_input_.InputSize(), 0);

  state_pub_topic_name_ = "state";

  emergency_stop_pub_topic_name_ = "emergency_stop";
  emergency_stop_msg_.data = false;

  safety_stop_pub_topic_name_ = "safety_stop";
  safety_stop_msg_.data = false;

  named_io_pub_topic_name_ = "named_io";

  robotnik_msgs::named_input_output rover_power_enabled_msg;
  rover_power_enabled_msg.name = "wheels_power_enabled";

  robotnik_msgs::named_input_output rover_motion_enabled_msg;
  rover_motion_enabled_msg.name = "motion_enabled";

  robotnik_msgs::named_input_output lasers_ok_msg;
  lasers_ok_msg.name = "laser_ok";

  robotnik_msgs::named_input_output hw_ok_edm_msg;
  hw_ok_edm_msg.name = "edm_ok";

  robotnik_msgs::named_input_output hw_ok_selector_msg;
  hw_ok_selector_msg.name = "selector_ok";

  robotnik_msgs::named_input_output hw_ok_msg;
  hw_ok_msg.name = "hw_ok";

  robotnik_msgs::named_input_output vel_ok_msg;
  vel_ok_msg.name = "vel_ok";
  robotnik_msgs::named_input_output platform_moving_msg;
  vel_ok_msg.name = "platform_moving";

  robotnik_msgs::named_input_output mode_changed_msg;
  mode_changed_msg.name = "mode_changed"; // TODO: add constants to robotnik_msgs

  robotnik_msgs::named_input_output reset_required_msg;
  reset_required_msg.name = "reset_required";

  safety_module_status_pub_topic_name_ = "status";

  named_io_msg_.digital_inputs.push_back(rover_power_enabled_msg);
  named_io_msg_.digital_inputs.push_back(rover_motion_enabled_msg);
  named_io_msg_.digital_inputs.push_back(lasers_ok_msg);
  named_io_msg_.digital_inputs.push_back(hw_ok_edm_msg);
  named_io_msg_.digital_inputs.push_back(hw_ok_selector_msg);
  named_io_msg_.digital_inputs.push_back(hw_ok_msg);
  named_io_msg_.digital_inputs.push_back(vel_ok_msg);
  named_io_msg_.digital_inputs.push_back(mode_changed_msg);
  named_io_msg_.digital_inputs.push_back(reset_required_msg);
  named_io_msg_.digital_inputs.push_back(platform_moving_msg);

  for (auto laser = 0; laser < eip_configuration_input_.laser_count_; ++laser) {

    robotnik_msgs::LaserStatus laser_status_msg;
    std::ostringstream laser_status_name_stream;
    laser_status_name_stream << "lsr" << (laser == 0 ? "1" : "2") << "_safezone";
    laser_status_msg.name = laser_status_name_stream.str();
    safety_module_status_msg_.lasers_status.push_back(laser_status_msg);
    for (auto zone = 0; zone < eip_configuration_input_.warning_zones_per_laser_; ++zone) {
      bool zone_bool;
      RCOMPONENT_INFO_STREAM("laser_status_msg.warning_zones.push_back(zone_bool);");
      laser_status_msg.warning_zones.push_back(zone_bool);
    }
    RCOMPONENT_INFO_STREAM("laser_status_msg.warning_zones.size()=" << laser_status_msg.warning_zones.size());
  }

  // Services
  set_digital_output_service_name_ = "digital_output";
  inputs_msg_.digital_outputs.assign(eip_configuration_input_.OutputSize(), 0);

  set_named_digital_output_service_name_ = "set_named_digital_output";

  set_laser_mode_service_name_ = "set_laser_mode";

  Discover();
  GetIdentityObject();
}

EIPScanner::~EIPScanner()
{
}

void EIPScanner::rosReadParams()
{
  bool required = true;
  bool not_required = false;

  readParam(pnh_, "desired_freq", desired_freq_, 1.0, required);
  ROS_INFO_STREAM("desired_freq_=" << desired_freq_);

  readParam(pnh_, eip_configuration_input_.kLaserCountKey, eip_configuration_input_.laser_count_, 1, required);
  ROS_INFO_STREAM("eip_configuration_input_.laser_count_=" << eip_configuration_input_.laser_count_);
  if (eip_configuration_input_.laser_count_ < robotnik_eipscanner::kMinLaserCount ||
      eip_configuration_input_.laser_count_ > robotnik_eipscanner::kMaxLaserCount) {
    ROS_INFO_STREAM("Invalid laser count: " << eip_configuration_input_.laser_count_ <<
      ", so setting to " << robotnik_eipscanner::kMinLaserCount);
     eip_configuration_input_.laser_count_ = robotnik_eipscanner::kMinLaserCount;
  }
  readParam(pnh_, eip_configuration_input_.kWarningZonesPerLaserKey, eip_configuration_input_.warning_zones_per_laser_, 1, required);
  ROS_INFO_STREAM("eip_configuration_input_.warning_zones_per_laser_=" << eip_configuration_input_.warning_zones_per_laser_);
  if (eip_configuration_input_.warning_zones_per_laser_ < robotnik_eipscanner::kMinLaserCount ||
      eip_configuration_input_.warning_zones_per_laser_ > robotnik_eipscanner::kMaxLaserCount) {
    ROS_INFO_STREAM("Invalid laser count: " << eip_configuration_input_.warning_zones_per_laser_ <<
      ", so setting to " << robotnik_eipscanner::kMinLaserCount);
     eip_configuration_input_.warning_zones_per_laser_ = robotnik_eipscanner::kMinLaserCount;
  }

  readParam(pnh_, eip_configuration_input_.kRoverPowerEnabledKey, eip_configuration_input_.rover_power_enabled_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.rover_power_enabled_=" << eip_configuration_input_.rover_power_enabled_);
  readParam(pnh_, eip_configuration_input_.kRoverMotionEnabledKey, eip_configuration_input_.rover_motion_enabled_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.rover_motion_enabled_=" << eip_configuration_input_.rover_motion_enabled_);

  readParam(pnh_, eip_configuration_input_.kHwOkKey, eip_configuration_input_.hw_ok_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.hw_ok_=" << eip_configuration_input_.hw_ok_);

  readParam(pnh_, eip_configuration_input_.kLasersOkKey, eip_configuration_input_.lasers_ok_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lasers_ok_=" << eip_configuration_input_.lasers_ok_);

  readParam(pnh_, eip_configuration_input_.kHwOkEdmKey, eip_configuration_input_.hw_ok_edm_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.hw_ok_edm_=" << eip_configuration_input_.hw_ok_edm_);

  readParam(pnh_, eip_configuration_input_.kHwOkSelectorKey, eip_configuration_input_.hw_ok_selector_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.hw_ok_selector_=" << eip_configuration_input_.hw_ok_selector_);

  readParam(pnh_, eip_configuration_input_.kHwOkEStopKey, eip_configuration_input_.hw_ok_estop_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.hw_ok_estop_=" << eip_configuration_input_.hw_ok_estop_);

  readParam(pnh_, eip_configuration_input_.kVelOkKey, eip_configuration_input_.vel_ok_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.vel_ok_=" << eip_configuration_input_.vel_ok_);

  readParam(pnh_, eip_configuration_input_.kOpModeAutoKey, eip_configuration_input_.op_mode_auto_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.op_mode_auto_=" << eip_configuration_input_.op_mode_auto_);

  readParam(pnh_, eip_configuration_input_.kOpModeManualKey, eip_configuration_input_.op_mode_manual_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.op_mode_manual_=" << eip_configuration_input_.op_mode_manual_);

  readParam(pnh_, eip_configuration_input_.kOpModeMaintenanceKey, eip_configuration_input_.op_mode_maintenance_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.op_mode_maintenance_=" << eip_configuration_input_.op_mode_maintenance_);

  // readParam(pnh_, eip_configuration_input_.kOpModeEmergencyKey, eip_configuration_input_.op_mode_emergency_, 0, required);
  // ROS_INFO_STREAM("eip_configuration_input_.op_mode_emergency_=" << eip_configuration_input_.op_mode_emergency_);

  readParam(pnh_, eip_configuration_input_.kSwitchLaserMuteOnKey, eip_configuration_input_.switch_laser_mute_on_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.switch_laser_mute_on_=" << eip_configuration_input_.switch_laser_mute_on_);

  readParam(pnh_, eip_configuration_input_.kSwitchLaserMuteOffKey, eip_configuration_input_.switch_laser_mute_off_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.switch_laser_mute_off_=" << eip_configuration_input_.switch_laser_mute_off_);

  readParam(pnh_, eip_configuration_input_.kLsr1SafezoneFreeKey, eip_configuration_input_.lsr1_safezone_free_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr1_safezone_free_=" << eip_configuration_input_.lsr1_safezone_free_);

  readParam(pnh_, eip_configuration_input_.kLsr2SafezoneFreeKey, eip_configuration_input_.lsr2_safezone_free_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr2_safezone_free_=" << eip_configuration_input_.lsr2_safezone_free_);

  readParam(pnh_, eip_configuration_input_.kLsrModeStandardInputKey, eip_configuration_input_.lsr_mode_standard_input_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_standard_input_=" << eip_configuration_input_.lsr_mode_standard_input_);
  readParam(pnh_, eip_configuration_input_.kLsrModeDockingInputKey, eip_configuration_input_.lsr_mode_docking_input_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_docking_input_=" << eip_configuration_input_.lsr_mode_docking_input_);
  readParam(pnh_, eip_configuration_input_.kLsrModeStandbyInputKey, eip_configuration_input_.lsr_mode_standby_input_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_standby_input_=" << eip_configuration_input_.lsr_mode_standby_input_);
  readParam(pnh_, eip_configuration_input_.kLsrModeCartInputKey, eip_configuration_input_.lsr_mode_cart_input_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_cart_input_=" << eip_configuration_input_.lsr_mode_cart_input_);
  readParam(pnh_, eip_configuration_input_.kLsrModeErrorKey, eip_configuration_input_.lsr_mode_error_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_error_=" << eip_configuration_input_.lsr_mode_error_);

  readParam(pnh_, eip_configuration_input_.kLsr1WarningZone1FreeKey, eip_configuration_input_.lsr1_warning_zone_1_free_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr1_warning_zone_1_free_=" << eip_configuration_input_.lsr1_warning_zone_1_free_);
  readParam(pnh_, eip_configuration_input_.kLsr2WarningZone1FreeKey, eip_configuration_input_.lsr2_warning_zone_1_free_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr2_warning_zone_1_free_=" << eip_configuration_input_.lsr2_warning_zone_1_free_);
  readParam(pnh_, eip_configuration_input_.kLsr1WarningZone2FreeKey, eip_configuration_input_.lsr1_warning_zone_2_free_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr1_warning_zone_2_free_=" << eip_configuration_input_.lsr1_warning_zone_2_free_);
  readParam(pnh_, eip_configuration_input_.kLsr2WarningZone2FreeKey, eip_configuration_input_.lsr2_warning_zone_2_free_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr2_warning_zone_2_free_=" << eip_configuration_input_.lsr2_warning_zone_2_free_);

  readParam(pnh_, eip_configuration_input_.kPlatformMovingKey, eip_configuration_input_.platform_moving_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.platform_moving_=" << eip_configuration_input_.platform_moving_);

  readParam(pnh_, eip_configuration_input_.kModeChangedKey, eip_configuration_input_.mode_changed_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.mode_changed_=" << eip_configuration_input_.mode_changed_);
  readParam(pnh_, eip_configuration_input_.kResetRequiredKey, eip_configuration_input_.reset_required_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.reset_required_=" << eip_configuration_input_.reset_required_);

  // Outputs
  readParam(pnh_, eip_configuration_input_.kEmergencyStopSwKey, eip_configuration_input_.emergency_stop_sw_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.emergency_stop_sw_=" << eip_configuration_input_.emergency_stop_sw_);

  readParam(pnh_, eip_configuration_input_.kBuzzerOnPulseKey, eip_configuration_input_.buzzer_on_pulse_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.buzzer_on_pulse=" << eip_configuration_input_.buzzer_on_pulse_);
  readParam(pnh_, eip_configuration_input_.kBuzzerOnIntermittentKey, eip_configuration_input_.buzzer_on_intermittent_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.buzzer_on_intermittent_=" << eip_configuration_input_.buzzer_on_intermittent_);

  readParam(pnh_, eip_configuration_input_.kBuzzerMode2Key, eip_configuration_input_.buzzer_mode_2_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.buzzer_mode_2_=" << eip_configuration_input_.buzzer_mode_2_);
  readParam(pnh_, eip_configuration_input_.kBuzzerMode3Key, eip_configuration_input_.buzzer_mode_3_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.buzzer_mode_3_=" << eip_configuration_input_.buzzer_mode_3_);

  readParam(pnh_, eip_configuration_input_.kLsrModeStandardOutputKey, eip_configuration_input_.lsr_mode_standard_output_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_standard_output_=" << eip_configuration_input_.lsr_mode_standard_output_);

  readParam(pnh_, eip_configuration_input_.kLsrModeDockingOutputKey, eip_configuration_input_.lsr_mode_docking_output_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_docking_output_=" << eip_configuration_input_.lsr_mode_docking_output_);

  readParam(pnh_, eip_configuration_input_.kLsrModeStandbyOutputKey, eip_configuration_input_.lsr_mode_standby_output_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_standby_output_=" << eip_configuration_input_.lsr_mode_standby_output_);

  readParam(pnh_, eip_configuration_input_.kLsrModeCartOutputKey, eip_configuration_input_.lsr_mode_cart_output_, 0, required);
  ROS_INFO_STREAM("eip_configuration_input_.lsr_mode_cart_output_=" << eip_configuration_input_.lsr_mode_cart_output_);
}

int EIPScanner::rosSetup()
{
  RComponent::rosSetup();


  bool required = true;
  bool not_required = false;

  // Publishers
  // See https://github.com/RobotnikAutomation/safety_module/blob/master/src/safety_module/safety_module_modbus_node.py
  // didn't implement if self.watchdog_signals_enabled:
  //   self.watchdog_signales_pub = rospy.Publisher('~watchdog_signals', BoolArray, queue_size=1)
  int queue_size = 10;
  inputs_pub_ = pnh_.advertise<robotnik_msgs::inputs_outputs>(inputs_pub_topic_name_, queue_size);
  state_pub_ = pnh_.advertise<robotnik_msgs::State>(state_pub_topic_name_, queue_size);
  emergency_stop_pub_ = pnh_.advertise<std_msgs::Bool>(emergency_stop_pub_topic_name_, 1);
  safety_stop_pub_ = pnh_.advertise<std_msgs::Bool>(safety_stop_pub_topic_name_, 1);
  named_io_pub_ = pnh_.advertise<robotnik_msgs::named_inputs_outputs>(named_io_pub_topic_name_, queue_size);
  safety_module_status_pub_ = pnh_.advertise<robotnik_msgs::SafetyModuleStatus>(safety_module_status_pub_topic_name_, queue_size);

  // Services
  set_digital_output_service_ =
        pnh_.advertiseService(set_digital_output_service_name_, &EIPScanner::SetDigitalOutputServiceCallback, this);
  set_named_digital_output_service_ =
        pnh_.advertiseService(set_named_digital_output_service_name_, &EIPScanner::SetNamedDigitalOutputServiceCallback, this);
  set_laser_mode_service_ =
        pnh_.advertiseService(set_laser_mode_service_name_, &EIPScanner::SetLaserModeServiceCallback, this);

  return 0;
}

int EIPScanner::rosShutdown()
{
  RComponent::rosShutdown();

  return 0;
}

void EIPScanner::rosPublish()
{
  RComponent::rosPublish();

  if (getState() == robotnik_msgs::State::READY_STATE)
  {
    std::lock_guard<std::mutex> inputs_lock_guard(message_mutex_);
    std::this_thread::sleep_for(100ms);
    PollState();
    inputs_pub_.publish(inputs_msg_);
  }

  emergency_stop_pub_.publish(emergency_stop_msg_);
  safety_stop_pub_.publish(safety_stop_msg_);

  // adapted: https://github.com/RobotnikAutomation/safety_module/blob/master/src/safety_module/safety_module_modbus_node.py
  state_msg_.state = getState();
  state_msg_.state_description = GetStateString(getState());
  state_msg_.desired_freq = desired_freq_;
  state_msg_.real_freq = real_freq;
  state_pub_.publish(state_msg_);

  named_io_pub_.publish(named_io_msg_);

  safety_module_status_pub_.publish(safety_module_status_msg_);
}

// apapted: https://github.com/RobotnikAutomation/safety_module/blob/master/src/safety_module/safety_module_modbus_node.py
string EIPScanner::GetStateString(int state)
  {
    switch (state)
    {
      case robotnik_msgs::State::INIT_STATE:
        return "INIT";
        break;
      case robotnik_msgs::State::STANDBY_STATE:
        return "STANDBY";
        break;
      case robotnik_msgs::State::READY_STATE:
        return "READY";
        break;
      case robotnik_msgs::State::EMERGENCY_STATE:
        return "EMERGENCY";
        break;
      case robotnik_msgs::State::FAILURE_STATE:
        return "FAILURE";
        break;
      case robotnik_msgs::State::SHUTDOWN_STATE:
        return "SHUTDOWN";
        break;
      default:
        return "UNKNOWN";
        break;
    }
  }

void EIPScanner::initState()
{
  RComponent::initState();
  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void  EIPScanner::standbyState()
{
    RCOMPONENT_INFO_STREAM("standbyState()");
    switchToState(robotnik_msgs::State::READY_STATE);
}

void  EIPScanner::readyState()
{

}

void EIPScanner::failureState()
{

}

void EIPScanner::emergencyState()
{

}

void EIPScanner::PrintOutputBits()
{
  RCOMPONENT_ERROR_STREAM("printing output bits");
  std::cout << '\n';
  for (bool bit : output_bits_) {
    std::cout << bit;
  }
  std::cout << '\n';
}

bool EIPScanner::SetDigitalOutputServiceCallback(robotnik_msgs::set_digital_output::Request & req,
                                robotnik_msgs::set_digital_output::Response & res)
{
  RCOMPONENT_INFO_STREAM("Received srv trigger petition.");
  if (state == robotnik_msgs::State::READY_STATE)
  {
    RCOMPONENT_INFO_STREAM("DigitalOutputService is ready");
    std::lock_guard<std::mutex> guard(message_mutex_);
    std::this_thread::sleep_for(100ms);

    output_bits_.assign(output_bits_.size(), 0);
    // output_bits_.reset();
    RCOMPONENT_INFO_STREAM("output_bits_.size()=" << output_bits_.size());
    RCOMPONENT_INFO_STREAM("req.output=" << req.output);
    RCOMPONENT_INFO_STREAM("req.value=" << req.value);
    output_bits_[req.output] = req.value;
    RCOMPONENT_INFO("Preparing to write state");
    WriteState();
    RCOMPONENT_INFO("Finished writing state");
    //output_bits_.reset();
    output_bits_.assign(output_bits_.size(), 0);
    res.ret = true;
    return true;
  }
  else
  {
    RCOMPONENT_INFO_STREAM("DigitalOutputService is not ready");
    res.ret = false;
    return true;
  }
  // No way to get here
  return false;
}

bool EIPScanner::SetNamedDigitalOutputServiceCallback(robotnik_msgs::set_named_digital_output::Request & req,
                                robotnik_msgs::set_named_digital_output::Response & res)
{
  RCOMPONENT_INFO_STREAM("Call SetLaserModeService");
  if (state == robotnik_msgs::State::READY_STATE)
  {
    RCOMPONENT_INFO_STREAM("SetLaserModeService is ready");
    std::lock_guard<std::mutex> guard(high_level_mutex_);

    robotnik_msgs::set_digital_output::Request set_digital_output_request;
    
    unsigned bit_index;

    if (req.name == eip_configuration_input_.kBuzzerOnPulseKey) {
      bit_index = eip_configuration_input_.buzzer_on_pulse_;
    } else if (req.name == eip_configuration_input_.kBuzzerOnIntermittentKey) {
      bit_index = eip_configuration_input_.buzzer_on_intermittent_;
    } else if (req.name == eip_configuration_input_.kBuzzerMode2Key) {
      bit_index = eip_configuration_input_.buzzer_mode_2_;
    } else if (req.name == eip_configuration_input_.kBuzzerMode3Key) {
      bit_index = eip_configuration_input_.buzzer_mode_3_;
    } else {
      // TODO: better error message
      RCOMPONENT_ERROR_STREAM("Unknown named digital output: " << req.name);
      return false;
    }
    set_digital_output_request.output = bit_index;
    set_digital_output_request.value = req.value;
    robotnik_msgs::set_digital_output::Response low_level_response;
    bool success = SetDigitalOutputServiceCallback(set_digital_output_request, low_level_response);
    if (!success) {
      return false;
    }
    res.ret = low_level_response.ret;
    return true;
  }
  else
  {
    RCOMPONENT_INFO_STREAM("SetLaserModeService is not ready");
    res.ret = false;
    return true;
  }
}

bool EIPScanner::SetLaserModeServiceCallback(robotnik_msgs::SetLaserMode::Request & req,
                                robotnik_msgs::SetLaserMode::Response & res)
{
  RCOMPONENT_INFO_STREAM("Call SetLaserModeService");
  if (state == robotnik_msgs::State::READY_STATE)
  {
    RCOMPONENT_INFO_STREAM("SetLaserModeService is ready");
    std::lock_guard<std::mutex> guard(high_level_mutex_);

    robotnik_msgs::set_digital_output::Request set_digital_output_request;
    // TODO: make bit locations configurable
    unsigned bit_index;

    if (req.mode == robotnik_msgs::LaserMode::STANDARD) {
      bit_index = eip_configuration_input_.lsr_mode_standard_output_;
    } else if (req.mode == robotnik_msgs::LaserMode::DOCKING_STATION) {
      bit_index = eip_configuration_input_.lsr_mode_docking_output_;
    } else if (req.mode == "standby") {  // TODO: add constant to LaserMode
      bit_index = eip_configuration_input_.lsr_mode_standby_output_;
    } else if (req.mode == robotnik_msgs::LaserMode::CART) {
      bit_index = eip_configuration_input_.lsr_mode_cart_output_;
    } else {
      // TODO: better error message
      RCOMPONENT_ERROR_STREAM("Unknown laser mode");
      return false;
    }
    set_digital_output_request.output = bit_index;
    set_digital_output_request.value = true;
    robotnik_msgs::set_digital_output::Response low_level_response;
    bool success = SetDigitalOutputServiceCallback(set_digital_output_request, low_level_response);
    if (!success) {
      return false;
    }
    res.ret = low_level_response.ret;
    return true;
  }
  else
  {
    RCOMPONENT_INFO_STREAM("SetLaserModeService is not ready");
    res.ret = false;
    return true;
  }
}

void EIPScanner::Discover()
{
  RCOMPONENT_INFO("Attempting to discover devices");

  auto receive_timeout = std::chrono::seconds(2);
  DiscoveryManager discoveryManager(ip_address_, kExplicitMessagingDefaultPort, receive_timeout);
  auto devices = discoveryManager.discover();

  if (devices.size() == 0) {
    RCOMPONENT_ERROR_STREAM("No devices found!");
    exit(0);
  }

  for (auto & device : devices) {
    RCOMPONENT_INFO_STREAM("Discovered device: " << device.identityObject.getProductName()
                          << " with address " << device.socketAddress.toString());
  }
}

void EIPScanner::GetIdentityObject()
{
  RCOMPONENT_INFO_STREAM("Attempting to create an identity object");
    try
    {
      RCOMPONENT_INFO_STREAM("Creating a session info object with ip address " << ip_address_ <<
        " and port " << std::hex << kExplicitMessagingDefaultPort);
      auto si = std::make_shared<SessionInfo>(ip_address_, kExplicitMessagingDefaultPort);
      IdentityObject identityObject(1, si);

      RCOMPONENT_INFO_STREAM("Vendor ID: " << identityObject.getVendorId() << " Device Type: "
              << identityObject.getDeviceType() << " Product Code: " << identityObject.getProductCode() 
              << " Revision: "  << identityObject.getRevision().toString() << " Status: "
              << identityObject.getStatus() << " Serial Number: " << identityObject.getSerialNumber()
              << " Product Name: " << identityObject.getProductName());
    }
    catch(const std::system_error & e)
    {
      RCOMPONENT_ERROR_STREAM(e.what());
      exit(0);
    }
}

void EIPScanner::WriteState()
{
  RCOMPONENT_INFO_STREAM("Attempting to write data to the PLC");
  SendExplicitMessage(Instance::kWrite, ServiceCodes::SET_ATTRIBUTE_SINGLE);
}

void EIPScanner::PollState()
{
  RCOMPONENT_INFO_STREAM("Attempting to read data from the PLC");
  SendExplicitMessage(Instance::kRead, ServiceCodes::GET_ATTRIBUTE_SINGLE);
}

void EIPScanner::SendExplicitMessage(Instance instance, ServiceCodes service_code)
{
  if ((service_code != ServiceCodes::GET_ATTRIBUTE_SINGLE) && (service_code != ServiceCodes::SET_ATTRIBUTE_SINGLE)) {
    RCOMPONENT_ERROR_STREAM("Unexpected EtherNet/IP service code: " << std::hex << service_code);
    return;
  }
  try
  {
    RCOMPONENT_INFO_STREAM("Creating a session info object with ip address " << ip_address_ <<
      " and port " << std::hex << kExplicitMessagingDefaultPort);
    auto si = std::make_shared<SessionInfo>(ip_address_, kExplicitMessagingDefaultPort);

    // Read the number of the parameters
    MessageRouter messageRouter;
    // According to DOC_MAN_FIE_installationshandbuch-psc1-feldbusse_SEN_AIN_V7.pdf, p. 93:
    // There are two different Assembly Objects (class ID: 04h) available:
    // Instance 64h(100d) / (PLC -> PSC1)
    // 4 Byte Functional Inputs and 64 Byte SD Bus request
    // Instance 65h(101d) / (PLC <- PSC1)
    // 128 Byte Functional Outputs and 64 Byte SD Bus response

    // the data (attribute 03h ) and the data length (attribute 04h ) can be read or written.
    // From https://eipscanner.readthedocs.io/en/latest/explicit_messaging.html
    // The most typical operations in the explicit communication are reading and writing CIP attributes.
    // The example that we used above is suitable, but we should keep in mind 2 things:
    // 1. Use cip::Epath with Attribute ID which you’re going to read or write an attribute.
    // For an example EPath(1,2,3), where ClassId=1, InstanceId=2, AttributeId=3
    // Use cip::ServiceCodes enum with the common service codes

    const EPath epath = EPath(kPsc1C100_ClassId, (CipUint)instance, (CipUint)Attribute::kData);
    RCOMPONENT_INFO_STREAM("Creating an epath with ClassID=" <<  kPsc1C100_ClassId << " InstanceID=" << (CipUint)instance << " AttributeId=" << (CipUint)Attribute::kData);

    Buffer send_buffer;
    // From a screenshot that appears on
    // DOC_MAN_FIE_installationshandbuch-psc1-feldbusse_SEN_AIN_V7.pdf, p. 93
  

    
    
    // Add back using vector<bool>
    // RCOMPONENT_INFO_STREAM("Binary: " << output_bits_ << '\n');
    PrintOutputBits();
    for (int byte = 0; byte < (output_bits_.size() / 8); ++byte) {
      RCOMPONENT_INFO_STREAM("Pushing byte " << byte);
      std::bitset<8> byte_bit_set;
      for (int bit = 0; bit < 8; ++bit) {
        int bit_offset = (byte * 8) + bit;
        if (output_bits_[bit_offset]) {
          RCOMPONENT_ERROR_STREAM("Pushing bit");
          byte_bit_set[bit] = true;
        }
      }
      RCOMPONENT_ERROR_STREAM("Pushing byte data " << byte_bit_set);
      send_buffer << (int8_t)(byte_bit_set.to_ulong());
    }

    // CipDword dword = output_bits_.to_ulong();
    // send_buffer << dword;
    
    // also from p 93:
    // "Reading/Writing of fieldbus data is also possible via Explicit Messaging objects."
    // "With the services Get_Attribute_Single (0Eh/14d) Set_Attribute_Single (10h/16d)""
    RCOMPONENT_INFO_STREAM("Sending a request with service_code=" << service_code);
    auto response = messageRouter.sendRequest(
      si,
      service_code,
      epath,
      send_buffer.data());
    
    if (response.getGeneralStatusCode() != GeneralStatusCodes::SUCCESS) {
      RCOMPONENT_ERROR_STREAM("Failed to read the data");
      logGeneralAndAdditionalStatus(response);
      return;
    }
    RCOMPONENT_INFO_STREAM("The reply service is " << response.getServiceCode());
    logGeneralAndAdditionalStatus(response);

    RCOMPONENT_INFO_STREAM("The size of the data is " << response.getData().size() << " bytes");
    Buffer receive_buffer(response.getData());
    RCOMPONENT_INFO_STREAM("The size of the buffer is " << receive_buffer.size() << " bytes");

    // Ignore read if setting
    if (service_code == ServiceCodes::SET_ATTRIBUTE_SINGLE) {
      if (!receive_buffer.empty()) {
        RCOMPONENT_ERROR_STREAM("Error: unexpected non-empty data buffer after a write operation");
      }
      return;
    }

    RCOMPONENT_INFO_STREAM("Begin to read bytes");

    int byte_number = 0;
    while (!receive_buffer.empty()) {
      RCOMPONENT_DEBUG_STREAM("Byte # " << byte_number);
      CipByte byte;
      receive_buffer >> byte;
      RCOMPONENT_DEBUG_STREAM("Decimal: " << (int)byte);
      bitset<kBitsPerByte> data_bits(byte);
      RCOMPONENT_DEBUG_STREAM("Binary: " << data_bits << '\n');

      int bit_number = 0;
      for (auto i = 0; i < kBitsPerByte; ++i) {
        inputs_msg_.digital_inputs[byte_number*kBitsPerByte + bit_number] = data_bits[i];
        bit_number += 1;
      }

      if (byte != 0b0000'0000) {
        RCOMPONENT_DEBUG_STREAM("NOT ZERO");
      }
      byte_number += 1;
    }
  }
  catch(const std::system_error & e)
  {
    RCOMPONENT_ERROR_STREAM(e.what());
    exit(0);
  }
  ReadInputs();
}

void EIPScanner::ReadInputs()
{
  RCOMPONENT_INFO_STREAM("Reading input variables");

  // Detect emergency stop
  emergency_stop_msg_.data = !inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_estop_];
  // TODO: are these really equivalent
  safety_module_status_msg_.emergency_stop = emergency_stop_msg_.data;

  bool sr1_safezone_free = inputs_msg_.digital_inputs[eip_configuration_input_.lsr1_safezone_free_];
  RCOMPONENT_DEBUG_STREAM("eip_configuration_input_.lsr1_safezone_free_=" << eip_configuration_input_.lsr1_safezone_free_);
  RCOMPONENT_DEBUG_STREAM("sr1_safezone_free=" << sr1_safezone_free);
  bool sr2_safezone_free = inputs_msg_.digital_inputs[eip_configuration_input_.lsr2_safezone_free_];
  RCOMPONENT_DEBUG_STREAM("eip_configuration_input_.lsr2_safezone_free_=" << eip_configuration_input_.lsr2_safezone_free_);
  RCOMPONENT_DEBUG_STREAM("sr2_safezone_free=" << sr2_safezone_free);

  safety_stop_msg_.data = sr1_safezone_free && sr2_safezone_free;
  // TODO: are these really equivalent?
  safety_module_status_msg_.safety_stop = safety_stop_msg_.data;

  RCOMPONENT_INFO_STREAM("Reading named io variables");

  // TODO; selecting by index is dangerous and brittle
  // Could fix this by dynamically creating a hash map from name to index
  named_io_msg_.digital_inputs[0].value = inputs_msg_.digital_inputs[eip_configuration_input_.rover_power_enabled_];
  named_io_msg_.digital_inputs[1].value = inputs_msg_.digital_inputs[eip_configuration_input_.rover_motion_enabled_];
  named_io_msg_.digital_inputs[2].value = inputs_msg_.digital_inputs[eip_configuration_input_.lasers_ok_];
  named_io_msg_.digital_inputs[3].value = inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_edm_];
  named_io_msg_.digital_inputs[4].value = inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_selector_];
  named_io_msg_.digital_inputs[5].value = inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_];
  named_io_msg_.digital_inputs[6].value = inputs_msg_.digital_inputs[eip_configuration_input_.vel_ok_];
  named_io_msg_.digital_inputs[7].value = inputs_msg_.digital_inputs[eip_configuration_input_.mode_changed_];
  named_io_msg_.digital_inputs[8].value = inputs_msg_.digital_inputs[eip_configuration_input_.reset_required_];
  named_io_msg_.digital_inputs[9].value = inputs_msg_.digital_inputs[eip_configuration_input_.platform_moving_];

  RCOMPONENT_INFO_STREAM("eip_configuration_input_.mode_changed_" << eip_configuration_input_.mode_changed_);
  RCOMPONENT_INFO_STREAM("inputs_msg_.digital_inputs[eip_configuration_input_.mode_changed_" << inputs_msg_.digital_inputs[eip_configuration_input_.mode_changed_]);
  RCOMPONENT_INFO_STREAM("eip_configuration_input_.reset_required_" << eip_configuration_input_.reset_required_);
  RCOMPONENT_INFO_STREAM("inputs_msg_.digital_inputs[eip_configuration_input_.reset_required_" << inputs_msg_.digital_inputs[eip_configuration_input_.reset_required_]);

  RCOMPONENT_INFO_STREAM("Reading op mode");

  bool op_mode_auto_bool = inputs_msg_.digital_inputs[eip_configuration_input_.op_mode_auto_];
  bool op_mode_manual_bool = inputs_msg_.digital_inputs[eip_configuration_input_.op_mode_manual_];
  bool op_mode_maintenance_bool = inputs_msg_.digital_inputs[eip_configuration_input_.op_mode_maintenance_];
  // bool op_mode_emergency_bool = inputs_msg_.digital_inputs[eip_configuration_input_.op_mode_emergency_];
  int op_mode_count = 0;

  if (op_mode_auto_bool) {
    safety_module_status_msg_.operation_mode = robotnik_msgs::SafetyModuleStatus::OM_AUTO;
    op_mode_count += 1;
  }
  if (op_mode_manual_bool) {
    safety_module_status_msg_.operation_mode = robotnik_msgs::SafetyModuleStatus::OM_MANUAL;
    op_mode_count += 1;
  }
  if (op_mode_maintenance_bool) {
    safety_module_status_msg_.operation_mode = robotnik_msgs::SafetyModuleStatus::OM_MAINTENANCE;
    op_mode_count += 1;
  }
  if (op_mode_count != 1) {
    // TODO: more thorough error message
    RCOMPONENT_ERROR_STREAM("More than one mutually operation exclusive mode is active");
  }

  RCOMPONENT_INFO_STREAM("Reading safety mode");

  // TODO Check bit 12 against 24 and 25
  safety_module_status_msg_.emergency_stop = !inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_estop_];
  // TODO Is this really redundant?
  //safety_module_status_msg_.safety_stop = safety_stop_msg_.data;
  // TODO: review
  safety_module_status_msg_.safety_stop = !(inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_estop_] && inputs_msg_.digital_inputs[eip_configuration_input_.lasers_ok_] && inputs_msg_.digital_inputs[eip_configuration_input_.hw_ok_]);

  ROS_INFO_STREAM("eip_configuration_input_.switch_laser_mute_on_=" << eip_configuration_input_.switch_laser_mute_on_);
  ROS_INFO_STREAM("eip_configuration_input_.switch_laser_mute_off_=" << eip_configuration_input_.switch_laser_mute_off_);
  bool switch_laser_mute_on_bool = inputs_msg_.digital_inputs[eip_configuration_input_.switch_laser_mute_on_];

  bool switch_laser_mute_off_bool = inputs_msg_.digital_inputs[eip_configuration_input_.switch_laser_mute_off_];
  
  int safety_mode_count = 0;
  if (switch_laser_mute_on_bool) {
    safety_module_status_msg_.safety_mode = robotnik_msgs::SafetyModuleStatus::LASER_MUTE;
    safety_mode_count += 1;
  }
  if (switch_laser_mute_off_bool) {
    safety_module_status_msg_.safety_mode = robotnik_msgs::SafetyModuleStatus::SAFE;
    safety_mode_count += 1;
  }

  if (safety_mode_count != 1) {
    // TODO: more thorough error message
    RCOMPONENT_ERROR_STREAM(safety_mode_count << " safety modes are active, but exactly one should be.");
  }

  RCOMPONENT_INFO_STREAM("Reading safe zones and warning zones");

  bool lsr1_safezone_free_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr1_safezone_free_];
  bool lsr2_safezone_free_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr2_safezone_free_];

  bool lsr1_warning_zone_1_free_ = inputs_msg_.digital_inputs[eip_configuration_input_.lsr1_warning_zone_1_free_];
  bool lsr2_warning_zone_1_free_ = inputs_msg_.digital_inputs[eip_configuration_input_.lsr2_warning_zone_1_free_];
  bool lsr1_warning_zone_2_free_ = inputs_msg_.digital_inputs[eip_configuration_input_.lsr1_warning_zone_2_free_];
  bool lsr2_warning_zone_2_free_ = inputs_msg_.digital_inputs[eip_configuration_input_.lsr2_warning_zone_2_free_];

  RCOMPONENT_INFO_STREAM("Reading laser 1");
  RCOMPONENT_INFO_STREAM("safety_module_status_msg_.lasers_status.size()=" << safety_module_status_msg_.lasers_status.size());
  safety_module_status_msg_.lasers_status[0].detecting_obstacles = !lsr1_safezone_free_bool;
  
  // TODO: I don't know why the following line doesn't work, why the warning_zones vector seems to be cleared (only on first call)
  // while the lasers_status is not
  // safety_module_status_msg_.lasers_status[0].warning_zones[0] = lsr1_warning_zone_1_free_;
  safety_module_status_msg_.lasers_status[0].warning_zones.clear();
  safety_module_status_msg_.lasers_status[0].warning_zones.push_back(!lsr1_warning_zone_1_free_);
  RCOMPONENT_INFO_STREAM(
    "safety_module_status_msg_.lasers_status[0].warning_zones.size()=" <<
    safety_module_status_msg_.lasers_status[0].warning_zones.size());
  RCOMPONENT_INFO_STREAM("Reading warning zones of laser 1");
  if (eip_configuration_input_.warning_zones_per_laser_ == 1) {
    safety_module_status_msg_.lasers_status[0].free_warning = lsr1_warning_zone_1_free_;
  } else if (eip_configuration_input_.warning_zones_per_laser_ == 2) {
    safety_module_status_msg_.lasers_status[0].free_warning = lsr1_warning_zone_1_free_ && lsr1_warning_zone_2_free_;
    // safety_module_status_msg_.lasers_status[0].warning_zones[1] = lsr1_warning_zone_2_free_;
    safety_module_status_msg_.lasers_status[0].warning_zones.push_back(!lsr1_warning_zone_2_free_);
  } else {
    RCOMPONENT_ERROR_STREAM("Invalid number of warning areas: " << eip_configuration_input_.warning_zones_per_laser_);
  }
  if (eip_configuration_input_.laser_count_ > 1) {
    RCOMPONENT_INFO_STREAM("Reading laser 2");
    safety_module_status_msg_.lasers_status[1].warning_zones.clear();
    safety_module_status_msg_.lasers_status[1].detecting_obstacles = !lsr2_safezone_free_bool;
    safety_module_status_msg_.lasers_status[1].warning_zones.push_back(!lsr2_warning_zone_1_free_);
    if (eip_configuration_input_.warning_zones_per_laser_ == 1) {
      safety_module_status_msg_.lasers_status[1].free_warning = lsr2_warning_zone_1_free_;
    } else if (eip_configuration_input_.warning_zones_per_laser_ == 2) {
      safety_module_status_msg_.lasers_status[1].free_warning = lsr2_warning_zone_1_free_ && lsr2_warning_zone_2_free_;
      safety_module_status_msg_.lasers_status[1].warning_zones.push_back(!lsr2_warning_zone_2_free_);
    } else {
      RCOMPONENT_ERROR_STREAM("Invalid number of warning areas: " << eip_configuration_input_.warning_zones_per_laser_);
    }
  }


  RCOMPONENT_INFO_STREAM("Reading laser mode");
  
  bool lsr_mode_standard_input_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr_mode_standard_input_];
  bool lsr_mode_docking_input_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr_mode_docking_input_];
  bool lsr_mode_standby_input_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr_mode_standby_input_];
  bool lsr_mode_cart_input_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr_mode_cart_input_];
  bool lsr_mode_error_bool = inputs_msg_.digital_inputs[eip_configuration_input_.lsr_mode_error_];
  int lsr_mode_count = 0;
  if (lsr_mode_standard_input_bool) {
    safety_module_status_msg_.lasers_mode.name = robotnik_msgs::LaserMode::STANDARD;
    lsr_mode_count += 1;
  }
  if (lsr_mode_docking_input_bool) {
    safety_module_status_msg_.lasers_mode.name = robotnik_msgs::LaserMode::DOCKING_STATION;
    lsr_mode_count += 1;
  }
  if (lsr_mode_standby_input_bool) {
    safety_module_status_msg_.lasers_mode.name = "standby"; // TODO: not defined in LaserMode
    lsr_mode_count += 1;
  }
  if (lsr_mode_cart_input_bool) {
    safety_module_status_msg_.lasers_mode.name = robotnik_msgs::LaserMode::CART;
    lsr_mode_count += 1;
  }
  if (lsr_mode_error_bool) {
    safety_module_status_msg_.lasers_mode.name = robotnik_msgs::LaserMode::INVALID;
    lsr_mode_count += 1;
  }

  if (lsr_mode_count != 1) {
    // TODO: more thorough error message
    RCOMPONENT_ERROR_STREAM(lsr_mode_count << " laser modes are active, but exactly one should be.");
  }

  // safety_module_status_msg_.emergency_stop = inputs_msg_.digital_inputs[eip_configuration_input_.op_mode_emergency_];

}


} // namespace robotnik_eipscanner

