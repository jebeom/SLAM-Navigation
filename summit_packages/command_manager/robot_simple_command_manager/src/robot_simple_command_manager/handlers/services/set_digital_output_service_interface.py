#!/usr/bin/env python

from ..command_service_interface import *


class SetDigitalOutputServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = ['output', 'value']
        self.args_types = [int, bool]
        self.args_void_allowed = [False, False]
        self.default_values = []
        self.output_types = []
        self.output_description = []

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        CommandServiceInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''

        if type(args) == list:
            self.request.output = args[0]
            self.request.value = args[1]


    def parse_feedback(self):
        '''
            Parse from feedback object to string
        '''
        return str(self.feedback_msg.ret)
    
    def has_succeeded(self):
        return self.feedback_msg.ret
