#!/usr/bin/env python
import rospy
from robot_simple_command_manager.parser.base_types_parser import BaseTypesParser
from robot_simple_command_manager.parser.lists import *

KS_SEPARATOR_TAGGED_ARG="="

KS_START_LIST="["
KS_END_LIST="]"
KS_SEPARATOR_ELEMENT_LIST=" "

class TypesParser():
    
    def __init__(self, args_description, args_types, args_void_allowed, default_values):
        self.args_description = args_description
        self.args_types = args_types
        self.args_void_allowed = args_void_allowed
        self.default_values = default_values

        return
    
    def parse_args(self, args):
        '''
            Return the list of arguments correctly typed.
            If any argument is wrong, raises an exception.
        '''
        # TODO: check if an argument can be void/null/none
        tmp_args = [None] * len(self.args_void_allowed)
        
        tagged_arg = False
        first_tag_name = ''
        first_tag_index = -1

        num_min_args = len(self.args_void_allowed) - sum(self.args_void_allowed)

        # args is the list of arguments splitted by " ", it is necessary to check
        # if there are lists in the arguments to join them again
        args = self.join_lists(args)
        if len(args) > len(self.args_void_allowed):
            raise Exception("Incorrect number of arguments to parse (%d). Max number is %d" %
                                 (len(args), len(self.args_void_allowed)))
        
        if len(args) < num_min_args:
            raise Exception("Incorrect number of arguments to parse (%d). Min number is %d" %
                                 (len(args), num_min_args))

        for index in range(0, len(args)):
            splitted_arg = args[index].split(KS_SEPARATOR_TAGGED_ARG)
            
            # If tagged argument
            if len(splitted_arg) == 2:
                name = splitted_arg[0]
                value = splitted_arg[1]
                if tagged_arg == False:
                    tagged_arg = True
                    first_tag_name = name
                    first_tag_index = index
                
                description_index = -1
                try:
                    description_index = self.args_description.index(name)
                except ValueError as e:
                    raise Exception("Name '%s' for argument not found in description list." %
                                 (name))
                
                # Check if already filled
                if tmp_args[description_index] is None:
                    parsed_argument = self.parse_arg(value, self.args_types[description_index])
                    tmp_args[description_index] = parsed_argument
                else:
                    raise Exception("Cannot fill argument '%s' with value '%s' cause was filled in the past with value '%s'." %
                                (name, value, str(tmp_args[description_index])))

            # Not tagged argument
            elif len(splitted_arg) == 1:
                if tagged_arg == True:
                    raise Exception("Naming a argument requires the following arguments to be named. First command named was '%s' in position '%d'" %
                                 (first_tag_name, first_tag_index))
                else:
                    # Check if already filled
                    if tmp_args[index] is None:
                        parsed_argument = self.parse_arg(args[index], self.args_types[index])
                        tmp_args[index] = parsed_argument
                    else:
                        raise Exception("Cannot fill argument '%s' with value '%s' cause was filled in the past with value '%s'." %
                                 (self.args_description[index], args[index], str(tmp_args[index])))
            else: 
                raise Exception("Incorrect number of '=' introduced for command (%s)." %
                                 (args[index]))

        end_required = len(self.args_void_allowed) - len(self.default_values) 
        
        # Checks if required arguments has been passed
        for index in range(0, end_required):
            if tmp_args[index] is None:
                raise Exception("Required argument '%s' has not been set." %
                                 (self.args_description[index]))


        # Sets default values for optional arguments if not provided
        for index in range(end_required, len(self.args_void_allowed)):
            if index < end_required:
                continue
            if tmp_args[index] is None:
                tmp_args[index] = self.default_values[index - end_required]
                rospy.logwarn('Setting the argument "%s" to the default value "%s"'
                          % (self.args_description[index], str(self.default_values[index - end_required])))

        return tmp_args

    def parse_arg(self, argument, desired_type):
        '''
            Converts an string into a specific type, as long as it is feasible

            Args:
                argument: string to be parsed
                desired_type: type to parse the param
            Return:
                - The parsed argument as the desired_type if OK
                - Raises an exception if ERROR
        '''
        if desired_type == bool:
            parsed_argument = BaseTypesParser.boolean(argument)
        elif desired_type == int:
            parsed_argument = BaseTypesParser.int(argument)
        elif desired_type == float:
            parsed_argument = BaseTypesParser.float(argument)
        elif desired_type == str:
            if argument == 'None':
                argument = ''
            parsed_argument = argument
        elif desired_type in list_types:
            parsed_argument = desired_type.parse(argument)
        else:
            raise Exception("Unknown type: the type " + desired_type + " to parse is not supported")

        return parsed_argument
        
    def join_lists(self, args):
        merged_args = []
        merged_arg = ""
        in_list_counter = 0
        for index in range(0, len(args)):
            argument = args[index]

            if KS_START_LIST in argument:
                in_list_counter += 1
            
            if KS_END_LIST in argument:
                in_list_counter -= 1
            
            merged_arg += " " + argument

            if in_list_counter == 0:
                merged_args.append(merged_arg.strip())
                merged_arg = ""

        return merged_args
