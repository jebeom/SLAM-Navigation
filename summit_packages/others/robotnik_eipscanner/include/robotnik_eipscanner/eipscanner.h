#ifndef ROBOTNIK_EIPSCANNER_EIP_SCANNER_H_
#define ROBOTNIK_EIPSCANNER_EIP_SCANNER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>
#include <bitset>
#include <string>

#include <EIPScanner/ParameterObject.h>

#include <std_msgs/Bool.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/named_inputs_outputs.h>
#include <robotnik_msgs/Register.h>
#include <robotnik_msgs/Registers.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/SafetyModuleStatus.h>
#include <robotnik_msgs/SetLaserMode.h>
#include <robotnik_msgs/set_named_digital_output.h> 
#include <rcomponent/rcomponent.h>

#include "robotnik_eipscanner/eip_configuration.h"

namespace robotnik_eipscanner {
class EIPScanner : public rcomponent::RComponent
{
private:
  /* EIPScanner stuff */
  string ip_address_;

public:
  EIPScanner(ros::NodeHandle h);
  virtual ~EIPScanner();

  static string GetStateString(int state);
protected:
  /* RComponent methods */

  //! Setups all the ROS' stuff
  virtual int rosSetup();

  /* RComponent methods !*/

  /* RComponent stuff */

  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node handle, to read params and publish data
  ros::NodeHandle pnh_;

  /* RComponent stuff */

  //! Actions performed on init state
  virtual void initState();
  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  //! Actions performed on the emergency state
  virtual void emergencyState();
  //! Actions performed on Failure state
  virtual void failureState();
  //! Reads data a publish several info into different topics
  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();
  virtual int rosShutdown();

private:

  /* RComponent stuff !*/

  /* ROS Stuff */

  // Publishers

  //! Publish raw input bits to a topic
  ros::Publisher inputs_pub_;
  std::string inputs_pub_topic_name_;
  robotnik_msgs::inputs_outputs inputs_msg_;


  ros::Publisher state_pub_;
  std::string state_pub_topic_name_;
  robotnik_msgs::State state_msg_;

  ros::Publisher emergency_stop_pub_;
  std::string emergency_stop_pub_topic_name_;
  std_msgs::Bool emergency_stop_msg_;

  ros::Publisher safety_stop_pub_;
  std::string safety_stop_pub_topic_name_;
  std_msgs::Bool safety_stop_msg_;

  ros::Publisher named_io_pub_;
  std::string named_io_pub_topic_name_;
  robotnik_msgs::named_inputs_outputs named_io_msg_;

  ros::Publisher safety_module_status_pub_;
  std::string safety_module_status_pub_topic_name_;
  robotnik_msgs::SafetyModuleStatus safety_module_status_msg_;

  // Lock access to the input data while writing
  // and publishing a single time
  std::mutex message_mutex_;
  // Used by higher level services before locking lower level services
  std::mutex high_level_mutex_;

  // Set a single digital bit
  ros::ServiceServer set_digital_output_service_;
  string set_digital_output_service_name_;
  std::vector<bool> output_bits_;
  bool SetDigitalOutputServiceCallback(robotnik_msgs::set_digital_output::Request & req,
                                robotnik_msgs::set_digital_output::Response & res);

  void PrintOutputBits();
  // Set the laser mode
  ros::ServiceServer set_laser_mode_service_;
  string set_laser_mode_service_name_;
  robotnik_msgs::SetLaserMode set_laser_mode_request_;
  bool SetLaserModeServiceCallback(robotnik_msgs::SetLaserMode::Request & req,
                                robotnik_msgs::SetLaserMode::Response & res);

  // Set named digital output
  ros::ServiceServer set_named_digital_output_service_;
  string set_named_digital_output_service_name_;
  robotnik_msgs::SetLaserMode set_named_digital_output_request_;
  bool SetNamedDigitalOutputServiceCallback(robotnik_msgs::set_named_digital_output::Request & req,
                                robotnik_msgs::set_named_digital_output::Response & res);

  EIPConfigurationInput eip_configuration_input_;

  robotnik_msgs::Registers registers_;

  /* EIPScanner stuff */
  EIPConfigurationInput eip_configuration_input;
  void Discover();
  void GetIdentityObject();
  void PollState();
  void WriteState();
  void ReadInputs();
  void SendExplicitMessage(Instance instance, eipScanner::cip::ServiceCodes service_code);
};
} // namespace robotnik_eipscanner

#endif  // ROBOTNIK_EIPSCANNER_EIP_SCANNER_H_
