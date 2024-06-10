#!/usr/bin/env python
from robot_simple_command_manager.parser.base_types_parser import BaseTypesParser

class GPSCoordinateList(list):
    ELEMENT_SEPARATOR = " "

    # TODO: Probably some list methods need to be reimplemented
    
    @staticmethod
    def parse(text):
        # Deletes list delimiters
        text = text[1:-1]
        elements_list = text.split(GPSCoordinateList.ELEMENT_SEPARATOR)
        
        coordinates = []
        # Parse each GPS coordinate
        for element in elements_list:
            coordinates.append(GPSCoordinate.parse(element))
        
        coordinates = GPSCoordinateList(coordinates)
        return coordinates

class GPSCoordinate(tuple):
    ELEMENT_SEPARATOR = ","
    NUM_ELEMNTS = 2

    @staticmethod
    def parse(text):
        elements = text.split(GPSCoordinate.ELEMENT_SEPARATOR)

        if len(elements) != GPSCoordinate.NUM_ELEMNTS:
            raise Exception("Expected %d arguments to create a GPSCoordinate, got %d"\
                            % (GPSCoordinate.NUM_ELEMNTS, len(elements)))

        latitude = BaseTypesParser.float(elements[0])
        longitude = BaseTypesParser.float(elements[1])

        return GPSCoordinate( (latitude, longitude))
