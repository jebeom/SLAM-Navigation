#!/usr/bin/env python
from abc import ABCMeta, abstractmethod
import rospy

from robot_simple_command_manager.parser.parser import TypesParser

class CommandInterface():
    __metaclass__ = ABCMeta
    
    def __init__(self, name, parameters):
        self.client = None
        self.feedback_msg = None
        self.result_msg = None
        self.feedback_str = ''
        self.output = ''
        self.parsed_args = []

        self.name = name
        self.type_parser = TypesParser(self.args_description, self.args_types, \
                                       self.args_void_allowed, self.default_values)

        self.check_definition_parameters()
        self.set_parameters(parameters)

    def check_definition_parameters(self):
        num_of_args = len(self.args_description)

        if num_of_args != len(self.args_types):
            self.raise_exception("Number of types (%d) does not equal with number of descriptions (%d)" \
                % (len(self.args_types), num_of_args))
        
        if num_of_args != len(self.args_void_allowed):
            self.raise_exception("Number of args_void_allowed (%d) does not equal with number of descriptions (%d)" \
                % (len(self.args_void_allowed), num_of_args))
        
        optional_args = sum(self.args_void_allowed)
        if optional_args != len(self.default_values):
            self.raise_exception("Number of default_values (%d) does not equal with number of optional arguments (%d)" \
                % (len(self.default_values), optional_args))        

    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        self.check_parameters(parameters)

        self.parameters = parameters
        self.namespace = parameters['namespace']

    def check_parameters(self, parameters):
        '''
            Checks that the handler has all required parameters set
        '''

        if 'type' not in parameters.keys():
            self.raise_exception('check_parameters: Handler has not "type" parameter defined.')
        else:
            if not self.is_parameter_type_correct(parameters['type'], str):
                self.raise_exception('check_parameters: Handler has "type" parameter defined but is not a string.')

        if 'namespace' not in parameters.keys():
            self.raise_exception('check_parameters: Handler has not "namespace" parameter defined.')
        else:
            if not self.is_parameter_type_correct(parameters['namespace'], str):
                self.raise_exception('check_parameters: Handler has "namespace" parameter defined but is not a str.')

    def get_parameter(self, param_key, default_value):
        '''
            Return the desired paremter or the default_value
        '''
        parameter = None
        if param_key in self.parameters:
            parameter = self.parameters[param_key]
        else:
            parameter = default_value
            rospy.logwarn('Setting the parameter "%s" to the default value "%s" for command "%s"'
                          % (param_key, str(default_value), self.name))

        return parameter

    def raise_exception(self, msg):
        '''
            Raise an exception related to the parameters check
        '''
        raise Exception(msg)

    def is_parameter_type_correct(self, parameter, desired_type):
        '''
            Returns True if the type of the parameter equals to the desired_type
        '''
        return type(parameter) == desired_type

    def parse_args(self, args):
        '''
            Return the list of arguments correctly typed.
            If any argument is wrong, raises an exception.
        '''
        self.parsed_args = self.type_parser.parse_args(args)
        return self.parsed_args

    def get_info(self):
        types = [args_type.__name__ for args_type in self.args_types]
        descriptions = self.args_description
        output_types = [output_type.__name__ for output_type in self.output_types]
        output_description = self.output_description
        return (types, descriptions, output_types, output_description)

    def get_command(self):
        named_args = ""
        for index in range(0, len(self.args_description)):
            current_arg = self.args_description[index] + "="+ str(self.parsed_args[index])
            named_args = named_args + current_arg + " " 
        command = (self.name + " " + named_args).strip()
        return command

    def get_output(self):
        return self.output

    @abstractmethod
    def import_messages(self):
        pass
    
    @abstractmethod
    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''
        pass
    
    @abstractmethod
    def cancel_cmd(self):
        pass

    @abstractmethod
    def feedback_cb(self, feedback):
        pass

    @abstractmethod
    def parse_feedback(self):
        pass

    def parse_output(self):
        return ''

    @abstractmethod
    def get_feedback(self):
        '''
            Return feedback string
        '''
        pass
    
    @abstractmethod
    def is_active(self):
        pass
    
    @abstractmethod
    def has_succeeded(self):
        pass
    
    @abstractmethod
    def get_result(self):
        '''
            Gets the result  as string of the action
        '''
        pass

    @abstractmethod
    def build_msg(self, args):
        '''
            Return True if the command has been sent
        '''
        pass
