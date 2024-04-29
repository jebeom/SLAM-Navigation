#!/usr/bin/env python

import rospy
import datetime

from robot_simple_command_sequencer.sequence import Sequence

class SequenceList():
    """
        Manages the list of sequences
    """

    def __init__(self, sequences):
        """
            Args:
                - sequences: string[] with all the sequences in the list
        """
        # Array of sequences
        self.sequences_string = sequences
        self.sequences = []
        self.current_sequence_index = 0
        self.id = ' '.join([str(elem) for elem in self.sequences_string])

        #
        self.loop = False
        self.loop_cycles = 0
        self.loop_max_cycles = 0
        #
        self._creation_datetime = datetime.datetime.now()
        self._creation_rostime = rospy.Time.now()
        self._id = self._creation_datetime.strftime("SL-%Y%m%d-%H%M%S")

    def get_id(self):
        """
            Returns object id as string
        """

        return self.id

    def set_loop(self, value, max_cycles = 0):
        """
            Sets the command loop option
        """
        self.loop_max_cycles = max_cycles
        self.loop = value

    def is_loop(self):
        """
            Returns if loop is enabled for this command
        """
        if self.loop == True:
            if self.loop_max_cycles > 0:
                return self.loop_cycles < self.loop_max_cycles - 1
            else:
                return True
        
        return False

    def get_loop_cycles(self):
        """
            Returns the number of completed loops
        """
        return self.loop_cycles

    def append(self, sequence_id, commands):
        """
            Adds a new sequence

            Args:
                sequence_id: string as the sequence id
                commands: string [] as the commands that contains the sequence
        """
        self.sequences.append(Sequence(sequence_id, commands))

    def reset(self):
        """
            Resets the list of sequences
        """
        for sequence in self.sequences:
            sequence.reset()
        self.current_sequence_index = 0
        if self.loop:
            self.loop_cycles += 1

    def _get_current_sequence(self):
        """
            Gets the current sequence
        """
        if self.current_sequence_index >= len(self.sequences):
            return None
        else:
            return self.sequences[self.current_sequence_index]

    def get_next_command(self):
        """
            Gets the next command of the current sequence
            Returns:
                - The next command as string
                - None if there are no more command in the sequence list
        """
        while True:
            current_sequence = self._get_current_sequence()
            # No more sequences nor commands
            if current_sequence == None:
                return None

            command = current_sequence.get_next_command()

            if command == None:  # No more commands in the sequence
                self.current_sequence_index += 1  # increase the sequence
            else:
                return command

    def get_current_commands(self):
        """
            Gets the current sequence list of commands
        """
        current_seq = self._get_current_sequence()
        return current_seq.get_commands()

    def get_current_index(self):
        """
            Gets the current sequence index of commands
        """
        current_seq = self._get_current_sequence()

        return current_seq.get_index()

    def get_current_sequence_id(self):
        """
            Returns the current sequence id as string
            Returns None if the sequences finished
        """
        current_sequence = self._get_current_sequence()
        # No more sequences nor commands
        if current_sequence == None:
            return None
        else:
            return current_sequence.get_id()

    def get_count_remaining_commands(self):
        """
            Gets the size of the sequence
        """
        return (self._get_current_sequence().get_size() - self._get_current_sequence().get_index())
