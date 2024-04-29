#include <string>
#include <ros/ros.h>
#include <robotnik_msgs/LaserMode.h>
#include "robotnik_eipscanner/eip_configuration.h"

namespace robotnik_eipscanner {

// Currently hard-coded for Schmersal C-10, which should be the same as the C-100
EIPConfigurationInput::EIPConfigurationInput() :
    // Inputs
    input_size_(192 * kBitsPerByte),
    output_size_(4 * kBitsPerByte),

    kLaserCountKey("laser_count"),
    kWarningZonesPerLaserKey("warning_zones_per_laser"),

    kRoverPowerEnabledKey("wheels_power_enabled"),
    kRoverMotionEnabledKey("motion_enabled"),
    kHwOkKey("hw_ok"),
    kLasersOkKey("laser_ok"),
    kHwOkEdmKey("edm_ok"),
    kHwOkSelectorKey("selector_ok"),

    kHwOkEStopKey("hw_ok_estop"),
    kVelOkKey("vel_ok"),

    kOpModeAutoKey("op_mode_auto"),
    kOpModeManualKey("op_mode_manual"),
    kOpModeMaintenanceKey("op_mode_maintenance"),
    // kOpModeEmergencyKey("op_mode_emergency"),

    kSwitchLaserMuteOnKey("switch_laser_mute_on"),
    kSwitchLaserMuteOffKey("switch_laser_mute_off"),

    kLsr1SafezoneFreeKey("lsr1_safezone_free"),
    kLsr2SafezoneFreeKey("lsr2_safezone_free"),

    kLsr1WarningZone1FreeKey("lsr1_warningzone1_free"),
    kLsr2WarningZone1FreeKey("lsr2_warningzone1_free"),
    kLsr1WarningZone2FreeKey("lsr1_warningzone2_free"),
    kLsr2WarningZone2FreeKey("lsr2_warningzone2_free"),

    kLsrModeStandardInputKey("lsr_mode_standard_input"),
    kLsrModeDockingInputKey("lsr_mode_docking_input"),
    kLsrModeStandbyInputKey("lsr_mode_standby_input"),
    kLsrModeCartInputKey("lsr_mode_cart_input"),
    kLsrModeErrorKey("lsr_mode_error"),

    kPlatformMovingKey("platform_moving"),

    kModeChangedKey("mode_changed"),
    kResetRequiredKey("reset_required"),

    // Outputs
    kEmergencyStopSwKey("emergency_stop_sw"),

    kBuzzerOnPulseKey("buzzer_on_pulse"),
    kBuzzerOnIntermittentKey("buzzer_on_intermittent"),

    kBuzzerMode2Key("buzzer_mode_2"),
    kBuzzerMode3Key("buzzer_mode_3"),

    kLsrModeStandardOutputKey("lsr_mode_standard_output"),
    kLsrModeDockingOutputKey("lsr_mode_docking_output"),
    kLsrModeStandbyOutputKey("lsr_mode_standby_output"),
    kLsrModeCartOutputKey("lsr_mode_cart_output")
{
    ROS_INFO_STREAM("output_size_=" << output_size_);
}

const int EIPConfigurationInput::InputSize() const {return input_size_; };
const int EIPConfigurationInput::OutputSize() const {return output_size_; };

RunState EIPConfigurationInput::GetRunState()
{
    return run_state_;
}
} // namespace robotnik_eipscanner