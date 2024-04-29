#!/usr/bin/env python

import shutil
import os
from time import sleep
from rospkg import RosPack 
import rosmsg
from threading import Thread
from collections import OrderedDict

color = {
    "purple" : '\033[95m',
    "cyan" : '\033[96m',
    "darkcyan" : '\033[36m',
    "blue" : '\033[94m',
    "green" : '\033[92m',
    "yellow" : '\033[93m',
    "red" : '\033[91m',
    "bold" : '\033[1m',
    "underline" : '\033[4m',
    "end" : '\033[0m'
}

def print_colored(text, color_name, bold):
    if bold == True:
        print(color["bold"] + color[color_name] + text + color["end"])
    else:
        print(color[color_name] + text + color["end"])

class HandlerCreator:
    def __init__(self):
        self.handler_types = {
            1: "action",
            2: "service",
            3: "procedure",
            4: "publisher",
            5: "exit"
        }

        self.handler_name_hints = {
            1: "Use snake case",
            2: "Try to use a descriptive name",
            3: "The script will append the type of the handler to create the handler file",
            4: "The script will automatically locate the handler inside the handlers folder"
        }

        self.handler_add_arguments_menu = {
            1: "Add",
            2: "No add"
        }

        self.handler_argument_type_menu = {
            1: "String",
            2: "Integer",
            3: "Float",
            4: "Boolean",
        }

        self.handler_argument_optional_menu = {
            1: "Yes",
            2: "No"
        }

        self.handler_type = None
        self.handler_name = None
        self.arguments_description = []
        self.arguments_types = []
        self.arguments_optional = []
        self.arguments_default_values = []

        self.outputs_description = []
        self.outputs_types = []

        self.add_more_arguments = True
        self.arguments_are_optional = False
        self.add_more_outputs = True

        self.msgs_ready = False

        # launch thread to execute get_msgs function
        self.msgs_thread = Thread(target=self.get_msgs)
        self.packages_with_msgs =  OrderedDict()
        self.packages_with_actions = OrderedDict()
        self.packages_with_srvs = OrderedDict()
        self.packages_with_procedures = OrderedDict()

        self.msgs_thread.setDaemon(True)
        self.msgs_thread.start()

    def get_msgs(self):
        rospack = RosPack()
        ros_packages = rospack.list()

        for  package in ros_packages:
            self.add_package_with_msgs(package)
            self.add_package_with_services(package)

            if "robot_local_control_msgs" in self.packages_with_srvs.keys():
                self.packages_with_procedures = {"robot_local_control_msgs" : self.packages_with_srvs["robot_local_control_msgs"]}

        self.msgs_ready = True

        self.packages_with_msgs = OrderedDict(sorted(self.packages_with_msgs.items()))
        self.packages_with_actions = OrderedDict(sorted(self.packages_with_actions.items()))
        self.packages_with_srvs = OrderedDict(sorted(self.packages_with_srvs.items()))

    def add_package_with_actions(self, package):    
        # get actions
        for msg in self.packages_with_msgs[package]:
            # if there is no action for current msg then skip
            if not msg.endswith('Action'):
                continue

            if self.packages_with_actions.has_key(package):
                self.packages_with_actions[package].append(msg)
            else:
                self.packages_with_actions[package] = [msg]

    def add_package_with_msgs(self, package):
        msgs_list = rosmsg.list_msgs(package)
        # if current package has no msgs then skip
        if len(msgs_list) == 0:
            return
    
        self.packages_with_msgs[package] = msgs_list
        self.add_package_with_actions(package)

    def add_package_with_services(self, package):
        srvs_list = rosmsg.list_srvs(package)
        # if current package has no srvs then skip
        if len(srvs_list) == 0:
            return
        
        self.packages_with_srvs[package] = srvs_list


    def print_menu_option(self, options):
        for key in options.keys():
            print(str(key) + ') ' + options[key])

    def handler_type_menu(self):
        text = "THIS SCRIPT HAS TO BE RUN ONCE THE WORKSPACE HAS BEEN COMPILED AND SOURCED\n"
        print_colored(text, "yellow", True)

        text = "This script will create an initial version (from a template) of the handler you want to add."
        print_colored(text, "darkcyan", True)

        text = "Please select the handler type number. It will be client of a:"
        print_colored(text, "darkcyan", True)
        self.print_menu_option(self.handler_types)
        
        try:
            option = int(input('Enter your choice: '))
            if option == 1:
                self.handler_type = "action"
                print('Handler type selected "'+self.handler_type+'"\n')
            elif option == 2:
                self.handler_type = "service"
                print('Handler type selected "'+self.handler_type+'"\n')
            elif option == 3:
                self.handler_type = "procedure"
                print('Handler type selected "'+self.handler_type+'"\n')
            elif option == 4:
                self.handler_type = "publisher"
                print('Handler type selected "'+self.handler_type+'"\n')
            elif option == 5:
                print('Thanks message before exiting')
                exit()
            else:
                text = 'Invalid option. Please enter a number between 1 and 5.'
                print_colored(text, "red", True)
                self.handler_type_menu()
        except NameError as e:
            text = 'Invalid option. Please enter a number between 1 and 5.'
            print_colored(text, "red", True)
            self.handler_type_menu()
        except KeyboardInterrupt as e:
            self.exit()
    
        
    def select_handler_name(self):
        text = "Write the name of your handler. Keep in mind:"
        print_colored(text, "darkcyan", True)    
        self.print_menu_option(self.handler_name_hints)
        try:
            self.handler_name = raw_input('Enter your handler name: ')
        except KeyboardInterrupt as e:
            self.exit()

        print("")

    def create_file(self):
        old_file = "templates/handler_name_template_type_interface.py"
        new_file = self.handler_name+"_"+self.handler_type + "_interface.py"
        shutil.copy(old_file, new_file)
    
    def configure_template(self):
        filename = self.handler_name+"_"+self.handler_type + "_interface.py"   
        
        self.replace_template_words(filename)
        self.delete_comment_block(filename)
        self.delete_comment_lines(filename)

    def replace_template_words(self, filename):
        with open(filename, 'r') as file :
            filedata = file.read()

        filedata = filedata.replace('template_type', self.handler_type)

        handler_type_cammel = self.snake_to_camel(self.handler_type)
        filedata = filedata.replace('TemplateType', handler_type_cammel)

        handler_name_cammel = self.snake_to_camel(self.handler_name)
        filedata = filedata.replace('HandlerName', handler_name_cammel)

        with open(filename, 'w') as file:
            file.write(filedata)
    
    def delete_comment_lines(self, filename):
        with open(filename, 'r') as fp:
            lines = fp.readlines()

        with open(filename, 'w') as fp:
            for line in lines:
                if '##' not in line:
                    fp.write(line)

    def delete_comment_block(self, filename):
        with open(filename, 'r') as fp:
            lines = fp.readlines()

        in_block = False
        with open(filename, 'w') as fp:
            for line in lines:
                if "##" in line and self.handler_type not in line:
                    # First line of the comment block
                    if in_block == False:
                        in_block = True
                    # Last line of the comment block
                    elif in_block == True:
                        in_block = False
                else:
                    if in_block == False:
                        fp.write(line)
                    elif in_block == True:
                        pass

    def snake_to_camel(self, text):
        # split underscore using split
        temp = text.split('_')
        
        # joining result 
        res = ''.join(ele.title() for ele in temp)
        return res
    
    def add_arguments(self):
        while self.add_more_arguments == True:
            text = "Do you want to add an argument to the handler?"
            print_colored(text, "darkcyan", True)    

            self.print_menu_option(self.handler_add_arguments_menu)
            try:
                option = int(input('Enter your choice: ')) 
                if option == 1:
                    self.add_argument()
                elif option == 2:
                    self.add_more_arguments = False
                else:
                    text = 'Invalid option. Please enter a number between 1 and 2.\n'
                    print_colored(text, "red", True)
                    self.add_arguments()
            except NameError as e:
                text = 'Invalid option. Please enter a number between 1 and 2.'
                print_colored(text, "red", True)
                self.add_arguments()
            
            except KeyboardInterrupt as e:
                self.exit()

            print("")
        
        print("")
        
        self.apply_arguments_config()
    
    def add_argument(self):
        print("")
        self.arguments_description.append(raw_input('Write the argument description (using snake_case): '))
        print("")
        self.select_argument_type()
        print("")
        self.select_argument_optional()
        print("")
    
    def select_argument_type(self):
        text = "Please select the argument type:"
        print_colored(text, "darkcyan", True)    
        
        self.print_menu_option(self.handler_argument_type_menu)
        
        option = int(input('Enter your choice: ')) 
        if option == 1:
            self.arguments_types.append("str") 
        elif option == 2:
            self.arguments_types.append("int") 
        elif option == 3:
            self.arguments_types.append("float") 
        elif option == 4:
            self.arguments_types.append("bool") 
        else:
            print('Invalid option. Please enter a number between 1 and 4.\n')
            self.select_argument_type()

    def select_argument_optional(self):
        if self.arguments_are_optional == True:
            self.arguments_optional.append(True)
            self.select_argument_default_value()
            return

        text = "Is the argument optional?"
        print_colored(text, "darkcyan", True)    
        text = "Remember that if you select an argument as optional, the following will also be optional"
        print_colored(text, "yellow", True)    
        self.print_menu_option(self.handler_argument_optional_menu)
        
        option = int(input('Enter your choice: ')) 
        if option == 1:
            self.arguments_optional.append(True)
            self.select_argument_default_value()
            self.arguments_are_optional = True 
        elif option == 2:
            self.arguments_optional.append(False)
        else:
            print('Invalid option. Please enter a number between 1 and 2.\n')
            self.select_argument_optional()

    def select_argument_default_value(self):
        # TODO: check default value type
        print("")
        print(self.arguments_types)
        default_value = None

        if self.arguments_types[-1] == "str":
            default_value = raw_input('Write default value for optional argument: ')
        elif self.arguments_types[-1] == "int" or self.arguments_types[-1] == "float":
            default_value = input('Write default value for optional argument: ')
        elif self.arguments_types[-1] == "bool":
            default_value = raw_input('Write default value for optional argument: ')
            default_value = default_value.lower().capitalize()
            if default_value == "True":
                default_value = True
            elif default_value == "False":
                default_value = False
            else:
                default_value = None
                print_colored("Invalid value, I only accept true/false", "red", True) 
        
        if default_value is None:
            self.select_argument_default_value()
        else:
            self.arguments_default_values.append(default_value)
        
    def apply_arguments_config(self):
        filename = self.handler_name+"_"+self.handler_type + "_interface.py"   

        with open(filename, 'r') as fp:
            lines = fp.readlines()

        with open(filename, 'w') as fp:
            for line in lines:
                if "args_description" in line:
                    line = line.replace("[]", str(self.arguments_description))
                elif "args_types" in line:
                    line = line.replace("[]", str(self.arguments_types))
                    line = line.replace("'", "")
                elif "args_void_allowed" in line:
                    line = line.replace("[]", str(self.arguments_optional))
                elif "default_values" in line:
                    line = line.replace("[]", str(self.arguments_default_values))
                    #line = line.replace("'", "")
                
                fp.write(line)

    def add_outputs(self):
        while self.add_more_outputs == True:
            text = "Do you want to add an output to the handler?"
            print_colored(text, "darkcyan", True)    

            self.print_menu_option(self.handler_add_arguments_menu)
            try:
                option = int(input('Enter your choice: ')) 
                if option == 1:
                    self.add_output()
                elif option == 2:
                    self.add_more_outputs = False
                else:
                    print('Invalid option. Please enter a number between 1 and 2.\n')
                    self.add_outputs()

            except NameError as e:
                text = 'Invalid option. Please enter a number between 1 and 2.'
                print_colored(text, "red", True)
                self.add_outputs()
            except KeyboardInterrupt as e:
                self.exit()

            print("")
        print("")
        
        self.apply_outputs_config()

    def add_output(self):
        print("")
        self.outputs_description.append(raw_input('Write the output description (using snake_case): '))
        print("")
        self.select_output_type()
        print("")
    
    def select_output_type(self):
        text = "Please select the output type:"
        print_colored(text, "darkcyan", True)    

        self.print_menu_option(self.handler_argument_type_menu)
        
        option = int(input('Enter your choice: ')) 
        if option == 1:
            self.outputs_types.append("str") 
        elif option == 2:
            self.outputs_types.append("int") 
        elif option == 3:
            self.outputs_types.append("float") 
        elif option == 4:
            self.outputs_types.append("bool") 
        else:
            print('Invalid option. Please enter a number between 1 and 4.\n')
            self.select_output_type()
    
    def apply_outputs_config(self):
        filename = self.handler_name+"_"+self.handler_type + "_interface.py"   

        with open(filename, 'r') as fp:
            lines = fp.readlines()

        with open(filename, 'w') as fp:
            for line in lines:
                if "output_description" in line:
                    line = line.replace("[]", str(self.outputs_description))
                elif "output_types" in line:
                    line = line.replace("[]", str(self.outputs_types))
                    line = line.replace("'", "")
                
                fp.write(line)

    def select_msg_package(self):
        while not self.msgs_ready:
            print("I'm retrieving all available messages, hold on")
            sleep(2)

        if self.handler_type == 'action':
            self.select_generic_msg_package(self.packages_with_actions)
        elif self.handler_type == 'service':
            self.select_generic_msg_package(self.packages_with_srvs)
        elif self.handler_type == 'publisher':
            self.select_generic_msg_package(self.packages_with_msgs)
        elif self.handler_type == 'procedure':
            self.select_generic_msg_package(self.packages_with_procedures)

        filename = self.handler_name+"_"+self.handler_type + "_interface.py"   

        with open(filename, 'r') as fp:
            lines = fp.readlines()

        with open(filename, 'w') as fp:
            for line in lines:
                if "action_package" in line:
                    line = line.replace("action_package", self.msg_package)
                elif "service_package" in line:
                    line = line.replace("service_package", self.msg_package)
                elif "publisher_package" in line:
                    line = line.replace("publisher_package", self.msg_package)
                
                fp.write(line) 
        print("\n")

    def select_generic_msg_package(self, packages_dict):
        # generate enum for action messages keys
        packages = {}
        
        for pair in enumerate(packages_dict.keys()):
            packages[pair[0]] = pair[1]

        text = "Please select the package where the message is located:"
        print_colored(text, "darkcyan", True)    
    
        self.print_menu_option(packages)
    
        option = int(input('Enter your choice: '))
        package = packages_dict.keys()[option]
        self.msg_package = package 
        

    def select_msg_type(self):
        if self.handler_type == 'action':
            self.select_generic_msg_type(self.packages_with_actions)
            self.msg_type = self.msg_type.replace("Action", "")
        elif self.handler_type == 'service':
            self.select_generic_msg_type(self.packages_with_srvs)
        elif self.handler_type == 'procedure':
            self.select_generic_msg_type(self.packages_with_procedures)
        elif self.handler_type == 'publisher':
            self.select_generic_msg_type(self.packages_with_msgs)


        filename = self.handler_name+"_"+self.handler_type + "_interface.py"   

        with open(filename, 'r') as fp:
            lines = fp.readlines()

        with open(filename, 'w') as fp:
            for line in lines:
                if "msg_type_goal" in line:
                    line = line.replace("msg_type_goal", self.msg_type)
                elif "msg_type_request" in line:
                    line = line.replace("msg_type_request", self.msg_type)
                elif "msg_type_publisher" in line:
                    line = line.replace("msg_type_publisher", self.msg_type)
                
                fp.write(line) 
        print("\n")

    def select_generic_msg_type(self, package_dict):
        messages = {}
            
        for pair in enumerate(package_dict[self.msg_package]):
            messages[pair[0]] = pair[1].split('/')[-1]
        
        text = "Please select the message from the package " + self.msg_package +":"
        print_colored(text, "darkcyan", True)    
        
        self.print_menu_option(messages)
        
        option = int(input('Enter your choice: '))
        message = package_dict[self.msg_package][option].split('/')[-1]
        self.msg_type = message

    def start(self):
        self.handler_type_menu()
        self.select_handler_name()
        self.create_file()
        self.configure_template()

        self.add_arguments()
        self.add_outputs()
        self.select_msg_package()
        self.select_msg_type()

    def get_values(self):
        text = "HANDLER CORRECTLY CREATED:"
        print_colored(text, "green", True)    

        print("Handler type:\t\t\t" + self.handler_type)
        print("Handler name:\t\t\t" + self.handler_name)
        print("Arguments description:\t\t" + str(self.arguments_description))
        print("Arguments types:\t\t" + str(self.arguments_types))
        print("Arguments optionals:\t\t" + str(self.arguments_optional))
        print("Arguments default values:\t" + str(self.arguments_default_values))
        print("Outputs description:\t\t" + str(self.outputs_description))
        print("Outputs type:\t\t\t" + str(self.outputs_types))
        print("Message package:\t\t" + str(self.msg_package))
        print("Message type:\t\t\t" + str(self.msg_type))

    def exit(self):
        print_colored("\nBye!", "yellow", True)
        try:
            os.remove(self.handler_name+"_"+self.handler_type + "_interface.py")
        except TypeError as e:
            exit()
        exit()

creator = HandlerCreator()
creator.start()
creator.get_values()


