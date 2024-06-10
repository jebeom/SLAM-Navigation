#!/usr/bin/env python
class BaseTypesParser:
    @staticmethod
    def boolean(param):
        '''
            Return param value provided as boolean or rises an Exception if is not capable to cast the value 
        '''
        param_lower = param.lower()
        if param_lower == 'true':
            return True
        elif param_lower == 'false':
            return False
        else:
            raise Exception('Expected input: true or false. Error parsing argument: ' + param)
    @staticmethod
    def int(param):
        '''
            Return param value provided as integer or rises an Exception if is not capable to cast the value 
        '''
        tmp = param
        if tmp.startswith("-"):
            tmp = param[1:]

        if tmp.isdigit():
            return int(param)
        else:
            raise Exception('Expected input type: int. Error parsing argument: ' + param)
    
    @staticmethod
    def float(param):
        '''
            Return param value provided as float or rises an Exception if is not capable to cast the value 
        '''
        try:
            return float(param)
        except ValueError:
            raise Exception('Expected input type: float. Error parsing argument: ' + param)
