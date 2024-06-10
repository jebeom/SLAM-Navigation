#ifndef ROBOTNIK_EIPSCANNER_EIP_CONFIGURATION_INPUT_H_
#define ROBOTNIK_EIPSCANNER_EIP_CONFIGURATION_INPUT_H_

#include <bitset>
#include <string>
#include <robotnik_msgs/LaserMode.h>
#include <EIPScanner/ParameterObject.h>

using eipScanner::cip::CipUint;

namespace robotnik_eipscanner {

constexpr CipUint kPsc1C100_ClassId{0x04};
const eipScanner::cip::CipUint kExplicitMessagingDefaultPort{0xAF12};
constexpr int kBitsPerByte{8};

constexpr int kMinWarningZonesPerLaser{1};
constexpr int kMaxWarningZonesPerLaser{2};
constexpr int kMinLaserCount{1};
constexpr int kMaxLaserCount{2};

enum class RunState
{
    /* Byte 0, first 4 bits (decimal values):
    (1-3 will be invisible, since the bus isn't initialized)
    1 init 2 self check 3 initializing bus
    4 running
    5 stopped (probably can't get data either)
    6 fatal error
    7 alarm
    */
    kInit = 1,
    kSelfCheck = 2,
    kInitializingBus = 3,
    kRunning = 4,
    kStopped = 5,
    kFatalError = 6,
    kAlarm = 7
};

enum class Instance : CipUint
{
    kWrite = 0x64,
    kRead = 0x65
};

enum class Attribute : CipUint
{
    kData = 0x3,
    kDataLength = 0x4
};

class EIPConfigurationInput
{
public:
    EIPConfigurationInput();
    RunState GetRunState();

    const int InputSize() const;
    const int OutputSize() const;

    int laser_count_;
    const std::string kLaserCountKey;
    int warning_zones_per_laser_;
    const std::string kWarningZonesPerLaserKey;

    // Inputs
    int rover_power_enabled_;
    const std::string kRoverPowerEnabledKey;
    int rover_motion_enabled_;
    const std::string kRoverMotionEnabledKey;
    int lasers_ok_;
    const std::string kLasersOkKey;
    int hw_ok_;
    const std::string kHwOkKey;
    int hw_ok_edm_;
    const std::string kHwOkEdmKey;
    int hw_ok_selector_;
    const std::string kHwOkSelectorKey;
    int hw_ok_estop_;
    const std::string kHwOkEStopKey;
    int vel_ok_;
    const std::string kVelOkKey;

    int op_mode_auto_;
    const std::string kOpModeAutoKey;
    int op_mode_manual_;
    const std::string kOpModeManualKey;
    int op_mode_maintenance_;
    const std::string kOpModeMaintenanceKey;
    // int op_mode_emergency_;
    // const std::string kOpModeEmergencyKey;

    int switch_laser_mute_on_;
    const std::string kSwitchLaserMuteOnKey;
    int switch_laser_mute_off_;
    const std::string kSwitchLaserMuteOffKey;

    int lsr1_safezone_free_;
    const std::string kLsr1SafezoneFreeKey;
    int lsr2_safezone_free_;
    const std::string kLsr2SafezoneFreeKey;

    int lsr1_warning_zone_1_free_;
    const std::string kLsr1WarningZone1FreeKey;
    int lsr2_warning_zone_1_free_;
    const std::string kLsr2WarningZone1FreeKey;
    int lsr1_warning_zone_2_free_;
    const std::string kLsr1WarningZone2FreeKey;
    int lsr2_warning_zone_2_free_;
    const std::string kLsr2WarningZone2FreeKey;

    int lsr_mode_standard_input_;
    const std::string kLsrModeStandardInputKey;
    int lsr_mode_docking_input_;
    const std::string kLsrModeDockingInputKey;
    int lsr_mode_standby_input_;
    const std::string kLsrModeStandbyInputKey;
    int lsr_mode_cart_input_;
    const std::string kLsrModeCartInputKey;
    int lsr_mode_error_;
    const std::string kLsrModeErrorKey;

    int platform_moving_;
    const std::string kPlatformMovingKey;

    int mode_changed_;
    const std::string kModeChangedKey;
    int reset_required_;
    const std::string kResetRequiredKey;

    // Outputs
    int emergency_stop_sw_;
    const std::string kEmergencyStopSwKey;

    int buzzer_on_pulse_;
    const std::string kBuzzerOnPulseKey;
    int buzzer_on_intermittent_;
    const std::string kBuzzerOnIntermittentKey;

    int buzzer_mode_2_;
    const std::string kBuzzerMode2Key;
    int buzzer_mode_3_;
    const std::string kBuzzerMode3Key;

    int lsr_mode_standard_output_;
    const std::string kLsrModeStandardOutputKey;
    int lsr_mode_docking_output_;
    const std::string kLsrModeDockingOutputKey;
    int lsr_mode_standby_output_;
    const std::string kLsrModeStandbyOutputKey;
    int lsr_mode_cart_output_;
    const std::string kLsrModeCartOutputKey;
private:
    RunState run_state_;
    // std::bitset<kPs_C1_C100_InputSize> data_;

    const unsigned input_size_;
    const unsigned output_size_;
};
} // namespace robotnik_eipscanner
#endif // ROBOTNIK_EIPSCANNER_EIP_CONFIGURATION_INPUT_H_