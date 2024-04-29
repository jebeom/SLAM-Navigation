#!/usr/bin/env python

from ..command_publisher_interface import *
from std_msgs.msg import Bool

class BoolPublisherInterface(CommandPublisherInterface):
    def __init__(self, name, parameters):
        self.args_description = ['value']
        self.args_types = [bool]
        self.args_void_allowed = [False]
        self.default_values = []
        self.output_types = []
        self.output_description = []


        CommandPublisherInterface.__init__(self, name, parameters)
        self.client = rospy.Publisher(self.namespace, Bool, queue_size=1)


    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        CommandPublisherInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''
        if type(args) == list:
            self.msg = Bool()
            self.msg.data = args[0]
