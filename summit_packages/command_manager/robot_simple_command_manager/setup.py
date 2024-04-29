from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robot_simple_command_manager',
              'robot_simple_command_manager.parser',
              'robot_simple_command_manager.handlers',
              'robot_simple_command_manager.handlers.actions',
              'robot_simple_command_manager.handlers.procedures',
              'robot_simple_command_manager.handlers.publishers',
              'robot_simple_command_manager.handlers.services',
              'robot_simple_command_manager.handlers.subscribers'],
    package_dir={'': 'src'},
)

setup(**setup_args)
