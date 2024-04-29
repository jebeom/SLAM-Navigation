#!/usr/bin/env python


from ..command_procedure_interface import *


class PlaceProcedureInterface(CommandProcedureInterface):
    def __init__(self, name, parameters):
        self.args_types = []
        self.args_description = []
        self.args_void_allowed = []
        self.default_values = []
        self.output_types = []
        self.output_description = []

        CommandProcedureInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface 
        '''
        CommandProcedureInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''

        if type(args) == list: 
            self.request.procedure.pick_frame_id = ''
