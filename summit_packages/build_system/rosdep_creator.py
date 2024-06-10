#!/usr/bin/python

import os
import argparse
import platform
import shutil
import subprocess

from catkin_pkg.topological_order import topological_order_packages
from catkin_pkg.packages import find_packages
from utils import linux_distribution


class RosDepCreator:
    def __init__(self, args):
        self.ros_distro = args.ros_distro
        self.os_name = args.os_name

        self.base_directory = os.getcwd()
        self.source_directory = '.'
        self.output_directory = 'deb'

        self.dry_run = args.dry_run

    def run(self):
        self.print_instructions()
        if not self.dry_run:
            self.create_rosdep_source_file()
        packages = self.get_ordered_packages(self.source_directory)
        if not self.dry_run:
            self.create_rosdep_file(packages)
        print("\n***\n")
        print(self.create_rosdep_content(packages))
        print("\n***")
        self.print_instructions()

    def get_rosdep_file(self):
        return os.path.expandvars("$HOME/.ros/rosdep.yaml")

    def create_rosdep_file(self, packages):
        with open(self.get_rosdep_file(), 'wt') as rosdep:
            rosdep.write(self.create_rosdep_content(packages) + '\n')

    def create_rosdep_content(self, packages):
        content = ''
        for p in packages:
            content += "%s:\n" % p[1].name
            content += "  %s: [%s]\n" % (self.os_name,
                                         self.get_debian_name(p[1]))
        return content

    def get_rosdep_source_file(self):
        return "/etc/ros/rosdep/sources.list.d/50-my-packages.list"

    def get_rosdep_source_content(self):
        return "yaml file://" + self.get_rosdep_file()

    def create_rosdep_source_file(self):
        with open(self.get_rosdep_source_file(), 'wt') as rosdep:
            rosdep.write(self.get_rosdep_source_content() + '\n')

    def print_instructions(self):
        print("I'm running from %s" % self.base_directory)
        if self.dry_run:
            print("I'm running on dry-run: you need to execute the steps described below")
        else:
            print("I'm running on no dry-run: i'm executing the steps my self. The description is just informative")
        print("Instructions: ")
        print("1- Create file: " + self.get_rosdep_source_file())
        print("2- Add line: " + self.get_rosdep_source_content())
        print("3- Create rosdep configuration file: " + self.get_rosdep_file())
        print("4- Copy text between *** to your rosdep configuration file (the previous one)")
        print("5- Run rosdep update")

    def get_ordered_packages(self, source_directory):
        packages = find_packages(source_directory, exclude_subspaces=True)
        ordered_packages = topological_order_packages(
            packages, blacklisted=["catkin_tools_prebuild"])
        return ordered_packages

    def get_debian_name(self, package):
        # TODO: I imagine this can also be accessed from one of the methods from bloom/catkin, but for now
        # leave it like this. Will fail if building on some different from amd64
        # TODO: must check where that '-0' comes from
        return 'ros-' + self.ros_distro + '-' + package.name.replace('_', '-')


def main():
    (current_os_name, _, _) = linux_distribution()
    current_os_name = current_os_name.lower()

    parser = argparse.ArgumentParser()
    parser.add_argument('--ros-distro', required=True)
    parser.add_argument('--os-name', nargs='?', default=current_os_name)
    parser.add_argument('--dry-run', dest='dry_run', action='store_true')
    args = parser.parse_args()

    rosdep_creator = RosDepCreator(args)
    rosdep_creator.run()


if __name__ == "__main__":
    main()
