#!/usr/bin/python

import os
import argparse
import platform
import shutil
import subprocess
import yaml
import sys
import distro

from catkin_pkg.topological_order import topological_order_packages
from catkin_pkg.packages import find_packages
from utils import linux_distribution

class DebBuilder:
    # TODO: refactor init methods!
    def __init__(self, args=None):
        if args != None:
            self.__init_with_args__(args)
        else:
            self.ros_distro = 'kinetic'
            self.os_name = 'ubuntu'
            self.os_version = 'xenial'

            self.base_directory = os.getcwd()
            self.output_directory = 'deb'
            self.source_directory = '.'
            self.timestamps_file = 'timestamps'

            self.debug_build = True
            self.install_existing_packages = False
            self.clear_existing_packages = True
            self.install_built_packages = True
            self.clear_built_packages = True
            self.blacklist = []

    def __init__(self, args):
        self.ros_distro = args.ros_distro
        self.os_name = args.os_name
        self.os_version = args.os_version

        self.base_directory = os.getcwd()
        self.output_directory = 'deb'
        self.source_directory = '.'
        self.timestamps_file = 'timestamps'

        self.generate_install_debs_file = True
        self.path_install_debs_file = self.base_directory + \
            "/"+self.output_directory+"/install_debs.sh"
        self.path_remove_debs_file = self.base_directory + \
            "/"+self.output_directory+"/remove_debs.sh"

        self.debug_build = args.debug_build
        self.dry_run = args.dry_run
        self.install_existing_packages = args.install_existing_packages
        self.clear_existing_packages = True
        self.install_built_packages = True
        self.clear_built_packages = True
        self.blacklist = args.blacklist.replace(" ", "").split(",")

        self.imroot = (os.geteuid() == 0)
        if self.imroot == False:
            root_access_command = self.gain_root_access_command()
            self.execute_command(root_access_command)

    def run(self):
        print("I'm running from %s" % self.base_directory)

        self.create_output_directory(self.output_directory)
        self.blacklist.append("catkin_tools_prebuild")
        print("blacklisted packages = %s" % (str(self.blacklist)))
        packages = self.get_ordered_packages(
            self.source_directory, blacklisted=self.blacklist)

        packages_to_install = self.get_packages_to_install(packages)
        packages_to_build = self.get_packages_to_build(packages)

        if self.generate_install_debs_file:
            print("Creating installation script.")
            install_debs_file = open(self.path_install_debs_file, "w")
            install_debs_file.write("#!/bin/bash\n")
            for p in packages:
                install_debs_file.write(
                    "sudo dpkg -i "+self.get_debian_name(p[1])+"\n")
            install_debs_file.close()
            chmod_command = ['chmod', '+x', self.path_install_debs_file]
            self.execute_command(chmod_command)
            print("Creating remove packages script.")
            remove_debs_file = open(self.path_remove_debs_file, "w")
            remove_debs_file.write("#!/bin/bash\n")
            command = "sudo apt-get remove -y "
            for p in reversed(packages):
                command_aux = command + self.get_debian_name(p[1]).split('_')[0]+" "
                remove_debs_file.write(command_aux+"\n")
            remove_debs_file.close()
            chmod_command = ['chmod', '+x', self.path_remove_debs_file]
            self.execute_command(chmod_command)

        if self.install_existing_packages:
            print(
                "I'm installing the following packages that are already built, just in case: ")
            for p in packages_to_install:
                print("\t%s" % (p[1].name))

            for p in packages_to_install:
                self.install_package(p)
        else:
            print(
                "I am NOT installing the following packages that are already built, this may lead to issues.")

        # install packages
        print("I need to build the following packages: ")
        for p in packages_to_build:
            print("\t%s from folder %s" % (p[1].name, p[0]))

        print("I'm clearing old build files just in case")
        for p in packages_to_build:
            self.clear_package(p)

        # check for dependencies
        for p in packages_to_build:
            self.check_dependencies(p)

        built_packages = []
        for p in packages_to_build:
            if self.build_package(p) != 0:
                print("Error building package %s", p)
                sys.exit(-1)

            if self.move_package(p) != 0:
                print("Error building package %s", p)
                sys.exit(-1)

            if self.install_built_packages:
                print("Installing %s package" % p[1].name)
                self.install_package(p)

            if self.clear_built_packages:
                print("Clearing %s package" % p[1].name)
                self.clear_package(p)
            built_packages += p
            self.dump_timestamps_from_project(
                packages, self.get_timestamp_file())

    def clear_package(self, package):
        directories_to_delete = ['/debian', '/obj-x86_64-linux-gnu', '/.obj-x86_64-linux-gnu']
        for directory in directories_to_delete:
            shutil.rmtree(package[0] + directory, ignore_errors=True)

        print("I cannot clear for %s" % package[1].name)

    def check_dependencies(self, package):
        # def get_build_dependencies_in_project_for_package(self, package, project_packages):
        print("I cannot locate dependencies for %s" % package[1].name)

    def build_package(self, package):
        package_path = package[0]
        os.chdir(package_path)
        generate_command = self.get_generate_command(package[1])
        build_command = self.get_build_command(package[1])

        print("I'm on %s and will create package %s with the following commands \n\t%s\n\t%s" % (
            os.getcwd(), package[1].name, generate_command, build_command))

        if self.execute_command(generate_command) != 0:
            print("Error generating!")
            return -1

        if self.execute_command(build_command) != 0:
            print("Error building!")
            return -1

        os.chdir(self.base_directory)
        return 0

    def move_package(self, package):
        created_deb_file = './' + \
            os.path.dirname(package[0]) + '/' + \
            self.get_debian_name(package[1])
        destination_deb_file = self.get_destination_file(package)

        # TODO: refactor file name creation
        if self.dry_run:
            print("dry-run: I will move %s binary from %s to %s" % (
                package[1].name, created_deb_file, destination_deb_file))
            return 0

        print("I will move %s binary from %s to %s" % (package[1].name, created_deb_file, destination_deb_file))

        try:
            shutil.move(created_deb_file, destination_deb_file)
        except IOError as e:
            print(e)
            return -1
        return 0

    def install_package(self, package):
        install_command = self.get_install_command(package)
        self.execute_command(install_command)

        print("I will install %s binary with command %s" % (package[1].name, install_command))

    def execute_command(self, command):

        if self.dry_run:
            print('dry-run execution: Command %s' % command)
            return 0

        try:
            from subprocess import DEVNULL  # Python 3.
        except ImportError:
            DEVNULL = open(os.devnull, 'r+b')

        stdout = stderr = None
        if self.debug_build == False:
            stdout = stderr = DEVNULL

        # stdin=DEVNULL is to force a fail in case it tries to read from stdin
        my_env = os.environ.copy()
        p = subprocess.Popen(command, stdin=DEVNULL,
                             stdout=stdout, stderr=stderr, env=my_env)
        p.wait()

        print('Command %s resulted in' % command, p.returncode)
        return p.returncode

    def get_packages_to_build(self, packages):
        packages_to_build = []

        project_timestamps = self.get_timestamps_from_project(packages)
        deb_timestamps = self.get_timestamps_from_file(
            self.get_timestamp_file())

        for p in packages:
            destination_deb_file = self.get_destination_file(p)
            # TODO: this is a candidate to refactor
            if not os.path.exists(destination_deb_file):
                packages_to_build.append(p)
                print('package to build because deb does not exist:', p[0])
            elif p[0] not in deb_timestamps or p[0] not in project_timestamps:
                packages_to_build.append(p)
                print('package to build because timestamp does not exist:', p[0])
            elif deb_timestamps[p[0]] < project_timestamps[p[0]]:
                packages_to_build.append(p)
                print('package to build because deb is old:', p[0])

        return packages_to_build

    def get_packages_to_install(self, packages):
        packages_to_install = []
        # not sure if here we should check for
        for p in packages:
            destination_deb_file = self.get_destination_file(p)
            if os.path.exists(destination_deb_file):
                packages_to_install.append(p)
                print('package to install:', p[0])
        return packages_to_install

    def get_ordered_packages(self, source_directory, blacklisted=None):
        packages = find_packages(source_directory, exclude_subspaces=True)
        ordered_packages = topological_order_packages(
            packages, blacklisted=blacklisted)
        return ordered_packages

    def get_build_dependencies_in_project_for_package(self, package, project_packages):
        packages_name = [p[1].name for p in project_packages]
        dependencies = package[1].build_depends + package[1].build_export_depends + \
            package[1].buildtool_depends + package[1].buildtool_export_depends
        return [d for d in dependencies if d.name in packages_name]

    def get_timestamps_from_file(self, timestamps_file):
        timestamps = None
        try:
            with open(timestamps_file, 'r') as input_file:
                timestamps = yaml.load(input_file)
        except IOError as e:
            print("Timestamps file %s does not exists, cannot read timestamps" %
                  timestamps_file)
        if timestamps == None:
            timestamps = dict()
        return timestamps

    def dump_timestamps_from_project(self, packages, timestamps_file):
        timestamps = self.get_timestamps_from_project(packages)
        with open(timestamps_file, 'w') as output_file:
            yaml.dump(timestamps, output_file, default_flow_style=False)

    def get_timestamps_from_project(self, packages=None):
        # This two lines of loading packages are just for debugging purposes. Remove them when software is ready
        if packages == None:
            packages = self.get_ordered_packages(
                self.source_directory, blacklisted=["catkin_tools_prebuild"])

        timestamps = {}
        for p in packages:
            stamp = self.get_timestamp_for_package(p)
            timestamps[stamp[0]] = stamp[1]

        return timestamps

    def get_timestamp_for_package(self, package):
        timestamps = []
        for dirpath, dirnames, filenames in os.walk(package[0]):
            try:
                timestamps += [os.path.getmtime(dirpath+'/'+f)
                               for f in filenames]
            except OSError:
                pass
        return (package[0], max(timestamps))

    def create_output_directory(self, output_directory):
        if os.path.isdir(output_directory):
            print("Directory %s already exists" % output_directory)
            return True
        try:
            os.mkdir(output_directory)
        except OSError:
            print("Creation of the directory %s failed" % output_directory)
            return False
        else:
            print("Successfully created the directory %s" % output_directory)
        return True

    def get_debian_name(self, package):
        # TODO: I imagine this can also be accessed from one of the methods from bloom/catkin, but for now
        # leave it like this. Will fail if building on some different from amd64
        # TODO: must check where that '-0' comes from
        return 'ros-' + self.ros_distro + '-' + package.name.replace('_', '-') + '_' + package.version + '-0' + self.os_version + '_amd64.deb'

    def get_destination_file(self, package):
        return self.output_directory + '/' + self.get_debian_name(package[1])

    def get_timestamp_file(self):
        return self.output_directory + '/' + self.timestamps_file

    def get_generate_command(self, package):
        return ['bloom-generate', 'rosdebian', '-d', '--os-name', self.os_name, '--os-version', self.os_version, '--ros-distro', self.ros_distro]

    def get_build_command(self, package):
        return ['fakeroot', 'debian/rules', 'binary --parallel']

    def get_install_command(self, package):
        # TODO: refactor file name creation
        destination_deb_file = self.get_destination_file(package)
        root_access = self.get_root_command()
        return root_access + ['dpkg', '-i', destination_deb_file]

    def gain_root_access_command(self):
        return ['sudo', 'echo', 'i\'m root']

    def get_root_command(self):
        return [] if self.imroot else ['sudo']


def main():
    # get default args. leave ros-distro with default just to show how should be made
    # also, refactor this method someday
    (current_os_name, _, current_os_version) = linux_distribution()
    current_os_name = current_os_name.lower()

    parser = argparse.ArgumentParser()
    parser.add_argument('--ros-distro', required=True)
    parser.add_argument('--os-name', nargs='?', default=current_os_name)
    parser.add_argument('--os-version', nargs='?', default=current_os_version)
    parser.add_argument('--debug-build', dest='debug_build',
                        action='store_true')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true')
    parser.add_argument('--install-existing-packages',
                        dest='install_existing_packages', action='store_true')
    parser.add_argument('--no-install-existing-packages',
                        dest='install_existing_packages', action='store_false')
    parser.add_argument('--blacklist', nargs='?', default='')
    args = parser.parse_args()

    builder = DebBuilder(args)
    builder.run()


if __name__ == "__main__":
    main()
