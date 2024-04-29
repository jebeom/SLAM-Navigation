#!/usr/bin/env python

from ..command_service_interface import *


class SetLaserModeServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = ['laser_mode']
        self.args_types = [str]
        self.args_void_allowed = [False]
        self.default_values = []
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
            self.request.mode = args[0]


    def parse_feedback(self):
        '''
            Parse from feedback object to string
        '''
        return str(self.feedback_msg.ret)
    
    def has_succeeded(self):
        return self.feedback_msg.ret
