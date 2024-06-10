#!/usr/bin/env python
from robot_simple_command_manager.handlers.command_service_interface import *
from dynamic_reconfigure.srv import  ReconfigureRequest, Reconfigure
from dynamic_reconfigure.msg import BoolParameter, IntParameter, StrParameter, DoubleParameter

class SetParameterServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = ['module', 'param_name', 'value']
        self.args_types = [str, str, str]
        self.args_void_allowed = [False, False, False]
        self.default_values = []

        self.output_description = []
        self.output_types = []
        self.client = None

        CommandInterface.__init__(self, name, parameters)
    
    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)
        self.modules = self.get_parameter('modules', {})
    
    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''

        # Checks if the desired module is loaded in parameters
        if args[0] not in self.modules.keys():
            msg = "Module '" + args[0] + "' not included in the parameters of " \
                    + self.name + " handler"
            return False, msg

        # Checks if target parameter exists
        module = args[0]
        module_ns = self.modules[module]
        if module_ns.endswith("/") == True:
            module_ns = module_ns[:-1]
        param_name = args[1]
        has_param = rospy.has_param(module_ns + "/" + param_name)
        if has_param == False:
            msg = "Parameter " + param_name + " does not exists in namespace " \
                    + module_ns
            return False, msg
        
        # Updates the type to fit the real type 
        param_value = rospy.get_param(module_ns + "/" + param_name)
        param_type = type(param_value)
        self.args_types[2] = param_type

        parsed_arguments = self.parse_args(args)
        self.build_msg(parsed_arguments)

        # Re-initialize the service client
        self.client = rospy.ServiceProxy(module_ns + "/set_parameters", Reconfigure)

        self.client.wait_for_service()
        tmp = self.client.call(self.request)
        self.feedback_cb(tmp)

        # Restores the type to str
        self.args_types[2] = str
        return True,""

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            
            self.request = ReconfigureRequest()

            if self.args_types[2] == bool:
                param = BoolParameter()
                param.name = args[1]
                param.value = args[2]
                self.request.config.bools = [param]
            elif self.args_types[2] == str:
                param = StrParameter()
                param.name = args[1]
                param.value = args[2]
                self.request.config.strs = [param]
            elif self.args_types[2] == int:
                param = IntParameter()
                param.name = args[1]
                param.value = args[2]
                self.request.config.ints = [param]
            elif self.args_types[2] == float:
                param = DoubleParameter()
                param.name = args[1]
                param.value = args[2]
                self.request.config.doubles = [param]

    def parse_feedback(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg)

    def parse_output(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''


class DynamicReconfigureBoolServiceInterface(CommandServiceInterface):

    def __init__(self, name, parameters):
        self.args_description = ['value']
        self.args_types = [bool]
        self.args_void_allowed = [False]
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)
        self.param_name = self.get_parameter('param_name', 'teb_autosize')

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            self.request = ReconfigureRequest()
            param = BoolParameter()
            param.name = self.param_name
            param.value = args[0]
            self.request.config.bools = [param]

    def parse_feedback(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg)

    def parse_output(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''

class DynamicReconfigureIntServiceInterface(CommandServiceInterface):

    def __init__(self, name, parameters):
        self.args_description = ['value']
        self.args_types = [int]
        self.args_void_allowed = [False]
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)
        self.param_name = self.get_parameter('param_name', 'min_samples')

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            self.request = ReconfigureRequest()
            param = IntParameter()
            param.name = self.param_name
            param.value = args[0]
            self.request.config.ints = [param]

    def parse_feedback(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg)

    def parse_output(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''

class DynamicReconfigureStrServiceInterface(CommandServiceInterface):

    def __init__(self, name, parameters):
        self.args_description = ['value']
        self.args_types = [str]
        self.args_void_allowed = [False]
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)
        self.param_name = self.get_parameter('param_name', 'odom_topic')

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            self.request = ReconfigureRequest()
            param = StrParameter()
            param.name = self.param_name
            param.value = args[0]
            self.request.config.strs = [param]

    def parse_feedback(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg)

    def parse_output(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''

class DynamicReconfigureDoubleServiceInterface(CommandServiceInterface):

    def __init__(self, name, parameters):
        self.args_description = ['value']
        self.args_types = [float]
        self.args_void_allowed = [False]
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)
        self.param_name = self.get_parameter('param_name', 'max_vel_x')

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            self.request = ReconfigureRequest()
            param = DoubleParameter()
            param.name = self.param_name
            param.value = args[0]
            self.request.config.doubles = [param]

    def parse_feedback(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg)

    def parse_output(self):
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''