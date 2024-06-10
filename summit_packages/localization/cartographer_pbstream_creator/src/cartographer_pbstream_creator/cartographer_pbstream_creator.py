#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse

from cartographer_ros_msgs.srv import WriteState, WriteStateResponse


class CartographerPbstreamCreator(RComponent):
    """
    Auxiliar node that offers a service to generate Cartographer pbstream maps. The service returns inmediately without feedback to avoid blocking the caller thread execution
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.cartographer_save_pbstream_service_name = rospy.get_param(
            '~cartographer_save_pbstream_service_name', 'write_state')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        self.status_pub = rospy.Publisher(
            '~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher(
            '~status_stamped', StringStamped, queue_size=10)
        # self.save_pbstream_status_pub = rospy.Publisher(
        #     '~status', SavePbstreamStatus, queue_size=10)

        # Service server
        self.trigger_save_pbstream_srv = rospy.Service(
            '~write_state', WriteState, self.trigger_save_pbstream_cb)

        # Service client
        self.cartographer_save_pbstream = rospy.ServiceProxy(self.cartographer_save_pbstream_service_name, WriteState)

        return 0

    def init_state(self):
        self.status = String()
        self.save_pbstream_petition = False
        self.save_pbstream_filename = ""

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health
        # if(self.check_topics_health() == False):
        #     self.switch_to_state(State.EMERGENCY_STATE)
        #     return RComponent.ready_state(self)

        # Call cartographer_ros service to save pbstream
        if(self.save_pbstream_petition == True):
            self.save_pbstream_petition = False
            ret = self.cartographer_save_pbstream.call(self.save_pbstream_filename, False)
            rospy.logwarn("cartographer_save_pbstream feedback: %s", ret.status.message)
            self.save_pbstream_filename = ""


        # Publish topic with status
        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data
        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        return RComponent.ready_state(self)

    def emergency_state(self):
        pass
        # if(self.check_topics_health() == True):
        #     self.switch_to_state(State.READY_STATE)

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


    def trigger_save_pbstream_cb(self, req):
        rospy.logwarn("Received petition to save map %s", req.filename)
        self.save_pbstream_petition = True
        self.save_pbstream_filename = req.filename

        response = WriteStateResponse()
        response.status.code = 0
        response.status.message = "Received petition to save map " + req.filename
        return response
