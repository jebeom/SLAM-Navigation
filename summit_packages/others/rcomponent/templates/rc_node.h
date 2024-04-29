#ifndef _?RC_NODE_
#define _?RC_NODE_

#include <rcomponent/rcomponent.h>

// Insert here general includes:
#include <math.h>

// Insert here msg and srv includes:
#include <std_msgs/String.h>
#include <robotnik_msgs/StringStamped.h>

#include <std_srvs/Trigger.h>

class ?RCNode : public rcomponent::RComponent
{
public:
  ?RCNode(ros::NodeHandle h);
  ~?RCNode() override;

protected:
  /*** RComponent stuff ***/

  //! Setups all the ROS' stuff
  int rosSetup() override;
  //! Shutdowns all the ROS' stuff
  int rosShutdown() override;
  //! Reads data a publish several info into different topics
  void rosPublish() override;
  //! Reads params from params server
  void rosReadParams() override;
  //! Actions performed on init state
  void initState() override;
  //! Actions performed on standby state
  void standbyState() override;
  //! Actions performed on ready state
  void readyState() override;
  //! Actions performed on the emergency state
  void emergencyState() override;
  //! Actions performed on Failure state
  void failureState() override;

  /* RComponent stuff !*/

  /* ROS Stuff */

  // Publishers

  //! To publish the basic information
  ros::Publisher status_pub_;
  ros::Publisher status_stamped_pub_;

  //! Subscribers
  ros::Subscriber example_sub_;
  string example_subscriber_name_; // Name of the example_sub_ topic

  //! Services
  ros::ServiceServer example_server_;

  //! Callbacks
  void exampleSubCb(const std_msgs::String::ConstPtr& msg);

  bool exampleServerCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  /* ROS stuff !*/

  /* ?RCNode stuff */

  std_msgs::String status_;

  /* ?RCNode stuff !*/


};

#endif  // _?RC_NODE_
