#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math

# Insert here msg and srv imports:
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse


class GPSFilter(RComponent):
    """
    
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.gps_fix_input_subscriber_name = rospy.get_param(
            '~gps_fix_input', 'gps/fix')
        self.gps_fix_output_subscriber_name = rospy.get_param(
            '~gps_fix_output', 'gps/fix_output')
        self.limit_fix_covariance_latitude_longitude = rospy.get_param(
            '~gps_fix_covariance_latitude_longitude', 0.1)
        self.limit_fix_covariance_altitude = rospy.get_param(
            '~gps_fix_covariance_altitude', 0.5)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        self.gps_out_pub = rospy.Publisher(
            self.gps_fix_output_subscriber_name, NavSatFix, queue_size=1)

        # Subscriber
        self.gps_sub = rospy.Subscriber(self.gps_fix_input_subscriber_name, NavSatFix, self.gps_sub_cb)
        RComponent.add_topics_health(self, self.gps_sub, topic_id='gps_sub', timeout=1.0, required=False)


        return 0

    def init_state(self):
        self.status = String()

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health
        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return RComponent.ready_state(self)
        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def gps_sub_cb(self, msg):
        fix_output_msgs = NavSatFix()
        fix_output_msgs = msg
        if (fix_output_msgs.position_covariance[0] < self.limit_fix_covariance_latitude_longitude) and (fix_output_msgs.position_covariance[4] < self.limit_fix_covariance_latitude_longitude) and (fix_output_msgs.position_covariance[8] < self.limit_fix_covariance_altitude):
            fix_output_msgs.status.status = fix_output_msgs.status.STATUS_FIX
        else:
            fix_output_msgs.status.status = fix_output_msgs.status.STATUS_NO_FIX
        self.gps_out_pub.publish(fix_output_msgs)
        self.tick_topics_health('gps_sub')
