#!/usr/bin/env python

from ..command_subscriber_interface import *
from robotnik_msgs.msg import inputs_outputs

class WaitDISubscriberInterface(CommandSubscriberInterface):
    def __init__(self, name, parameters):
        self.args_description = ['input_number', 'expected_value']
        self.args_types = [int, bool]
        self.args_void_allowed = [False, False]
        self.default_values = []
        self.output_types = []
        self.output_description = []

        self.target_input = 0

        CommandSubscriberInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        CommandSubscriberInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''
        if type(args) == list:
            self.target_input = args[0]
            self.desired_value = args[1]

    def data_callback(self, msg):
        self.data = msg.digital_inputs[self.target_input]

    def import_messages(self):
        pass

    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''
        parsed_arguments = self.parse_args(args)
        self.build_msg(parsed_arguments)

        self.data = None
        self.client = rospy.Subscriber(self.namespace, inputs_outputs, self.data_callback)

        return True, ""
