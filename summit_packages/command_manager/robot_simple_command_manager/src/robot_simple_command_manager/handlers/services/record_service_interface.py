#!/usr/bin/env python

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from ..command_service_interface import *


class StartRecordServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = []
        self.args_types = []
        self.args_void_allowed = []
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
            self.request.action = "RECORD"

    def parse_feedback(self):
        '''
            Parse from feedback object to string
        '''
        return ''

class StopRecordServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = []
        self.args_types = []
        self.args_void_allowed = []
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
            self.request.action = "STOP"

    def parse_feedback(self):
        '''
            Parse from feedback object to string
        '''
        return ''

class RecordTimeServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = ['time']
        self.args_types = [int]
        self.args_void_allowed = [False]
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
            self.request.action = "RECORD"
            if args[0] <= 0:
                args[0] = 1
            self.request.max_time = args[0]

    def parse_feedback(self):
        '''
            Parse from feedback object to string
        '''
        return ''
