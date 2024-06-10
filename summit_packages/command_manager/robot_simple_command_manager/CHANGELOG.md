# 1.7.0 (03-08-2023)
- Python 2 and Python3 compatibility

# 1.6.0 (24-05-2023)
- Restores GPSCoordinates to use blank spaces (Juan Manuel N.C)([29866af](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/29866af))
- Updates GPS separator from blank space to colon (Juan Manuel N.C)([2d8d7e8](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/2d8d7e8))
- Adds README related to list parser (Álex Arnal)([6e78847](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/6e78847))
- Deletes not tested functions from GPSCoordinatesList (Álex Arnal)([95a2ac2](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/95a2ac2))
- Updates WAIT command to work with new parser (Álex Arnal)([e606263](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/e606263))
- First working version after parser refactor (Álex Arnal)([420fb80](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/420fb80))
- Merge remote-tracking branch 'origin/devel' into fix/goto-gps (Álex Arnal)([25af6fc](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/25af6fc))
- (origin/devel) Reverts python3 compatibility, several bugs (Alejandro Arnal)([3783856](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/3783856))
- Add first version to parse lists (Álex Arnal)([550cd61](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/550cd61))
- Fixes raise exceptions in parser (Álex Arnal)([07bd166](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/07bd166))
- Moves parse_args method to parser (Álex Arnal)([f045459](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/f045459))
- Type parsing extracted to a class (Álex Arnal)([24ff49a](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/24ff49a))
- Simplifies getting index of the current argument if is tagged (Álex Arnal)([2b180c3](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/2b180c3))
- Merge pull request #29 from RobotnikAutomation/feature/set-pose-from-frame (Alejandro Arnal)([f7810c4](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/f7810c4))
- Updates package version and changelog (Álex Arnal)([93afadb](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/93afadb))
- Moves handler from scripts to src (Álex Arnal)([b01fef6](https://github.com/RobotnikAutomation/robot_simple_command_manager/commit/b01fef6))

# 1.5.0 (22-02-2023)
- Adds set_pose_from_frame handler
- Compatibility with python3
- Adds PTZ handlers
- Improves switch environment handler
- Adds handlers to reconfigure dynamic parameters
- Adds action feedback timeout as parameter

# 1.4.0 (05-10-2022)
- Allows 'None' as empty string.
- Adds service for getting all handlers info
- Adds configurable timeout to action handlers

# 1.3.0 (14-09-2022)
- Adds wait DI/DO handlers

# 1.2.0 (14-09-2022)
- Adds optional arg to RLC_CHARGE handler
- Add service to get a selective list of handlers info objects
- Add args_description attribute to GoToGpsProcedure iface

# 1.1.0 (15-08-2022)
- Adds handler type for ROS publisher

# 1.0.1 (09-08-2022)
- Fixes return value from bool subscriber (from bool to (bool, str))
- Adds vscode extension to gitignore

# 1.0.0 (21-07-2022)
- First tagged version of the project.
