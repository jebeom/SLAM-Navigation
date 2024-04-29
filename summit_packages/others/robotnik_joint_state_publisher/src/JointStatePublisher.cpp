#include <robotnik_joint_state_publisher/JointStatePublisher.h>

JointStatePublisher::JointStatePublisher(ros::NodeHandle h) : RComponent(h), nh_(h), pnh_("~")
{
  component_name.assign(pnh_.getNamespace());

  rosReadParams();
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::rosReadParams()
{
  XmlRpc::XmlRpcValue joint_state_sources;
  bool has_joint_topics = readParam(pnh_, "source_list", joint_state_sources, joint_state_sources);
  
  if (has_joint_topics == true)
  {
    try
    {
      for(int i = 0; i < joint_state_sources.size(); i++)
      {
        std::string topic_name = static_cast<std::string>(joint_state_sources[i]["topic"]);
        double timeout = (double)joint_state_sources[i]["timeout"];
        joint_state_topics_.push_back(topic_name);
        joint_state_timeouts_.push_back(ros::Duration(timeout));
      }
    }
    catch(XmlRpc::XmlRpcException e)
    {
      RCOMPONENT_ERROR_STREAM("Error in topics source list configuration format: " << e.getMessage());
      exit(-1);
    }

  }
  else
  {
    RCOMPONENT_ERROR("No joint state topics specified by 'source_list' parameter");
    RComponent::switchToState(robotnik_msgs::State::SHUTDOWN_STATE);
    // TODO: weeeeell, idk if this is the best way to do this
    exit(-1);
  }

}

int JointStatePublisher::rosSetup()
{
  RComponent::rosSetup();

  // Subscribers
  joint_state_subs_.resize(joint_state_topics_.size());
  joint_state_msgs_.resize(joint_state_topics_.size());
  for (size_t i = 0; i < joint_state_topics_.size(); i++)
  {
    joint_state_subs_[i] = nh_.subscribe<sensor_msgs::JointState>(joint_state_topics_[i], 1, 
                            boost::bind(&JointStatePublisher::jointStateCb, this, _1, i));
  }

  joint_state_pub_ = pnh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  return rcomponent::ReturnValue::OK;
  
}

void JointStatePublisher::initState()
{
  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void  JointStatePublisher::standbyState()
{
  switchToState(robotnik_msgs::State::READY_STATE);
}

void  JointStatePublisher::readyState()
{
  updateJointState();
}

void JointStatePublisher::rosPublish()
{
  RComponent::rosPublish();
  joint_state_pub_.publish(joint_state_);
}

void JointStatePublisher::failureState()
{

}

void JointStatePublisher::emergencyState()
{

}

void JointStatePublisher::jointStateCb(const sensor_msgs::JointStateConstPtr& input, int index)
{

  joint_state_msgs_[index] = *input;
}

void JointStatePublisher::updateJointState()
{
  joint_state_ = sensor_msgs::JointState();
  joint_state_.header.stamp = ros::Time::now();

  for ( int i = 0; i < joint_state_msgs_.size(); i++ )
  {
    ros::Duration elapsed_time = ros::Time::now() - joint_state_msgs_[i].header.stamp;
    bool timed_out = (elapsed_time > joint_state_timeouts_[i]) & 
                      (joint_state_timeouts_[i] != ros::Duration(0.0));
                      
    if (timed_out == true)
    {
      RCOMPONENT_WARN_STREAM_THROTTLE(1, "Joint state message timeout for topic " << joint_state_topics_[i]);
      continue;
    }

    for (int j = 0; j < joint_state_msgs_[i].name.size(); j++)
    {
      joint_state_.name.push_back(joint_state_msgs_[i].name[j]);
      joint_state_.position.push_back(joint_state_msgs_[i].position[j]);
      joint_state_.velocity.push_back(joint_state_msgs_[i].velocity[j]);
      joint_state_.effort.push_back(joint_state_msgs_[i].effort[j]);
    }
  }
}

