#!/usr/bin/env python

import rospy
import datetime


class Sequence():
    """
        A sequence is a list of simple commands
    """

    def __init__(self, id, commands):
        """
            Args:
                id: String
                commands: String[]
        """
        self.id = id
        self.commands = commands
        self.current_command_index = 0
        #
        self._creation_datetime = datetime.datetime.now()
        self._creation_rostime = rospy.Time.now()
        self._id = self._creation_datetime.strftime("S-%Y%m%d-%H%M%S")

    def reset(self):
        """
        Resets the sequence of commands, starting from the begining
        """
        self.current_command_index = 0

    def get_next_command(self):
        """
            Gets the next command in the list
        """
        if self.current_command_index >= len(self.commands):
            return None
        else:
            command = self.commands[self.current_command_index]
            self.current_command_index += 1
            return command

    def get_commands(self):
        """
            Gets the list of commands
        """
        com = []

        for i in range(len(self.commands)):
            com.append(self.commands[i])

        return com

    def get_id(self):
        """
            Gets the id of the sequence
        """
        return self.id

    def get_size(self):
        """
            Gets the id of the sequence
        """
        return len(self.commands)

    def get_index(self):
        """
            Gets the id of the sequence
        """
        return self.current_command_index