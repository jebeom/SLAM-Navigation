#ifndef _JOINT_STATE_PUBLISHER_
#define _JOINT_STATE_PUBLISHER_

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>


#include <rcomponent/rcomponent.h>
#include <XmlRpcException.h>
#include <boost/algorithm/string.hpp>


class JointStatePublisher : public rcomponent::RComponent
{
public:
  JointStatePublisher(ros::NodeHandle h);
  virtual ~JointStatePublisher();

protected:
  /* RComponent methods */

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Reads params from params server
  virtual void rosReadParams();

  /* RComponent methods !*/

  /* RComponent stuff */

  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node hanlde, to read params and publish data
  ros::NodeHandle pnh_;

  /* RComponent stuff !*/
  
  sensor_msgs::JointState joint_state_;
  ros::Publisher joint_state_pub_;
  std::vector<std::string> joint_state_topics_;
  std::vector<ros::Duration> joint_state_timeouts_;
  std::vector<ros::Subscriber> joint_state_subs_;
  std::vector<sensor_msgs::JointState> joint_state_msgs_;

protected:
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

  virtual void rosPublish();
  /* RComponent stuff !*/

  /* JointStatePublisher stuff */

  virtual	void jointStateCb(const sensor_msgs::JointStateConstPtr& input, int index);

  void updateJointState();
};

#endif  // _JOINT_STATE_PUBLISHER_
