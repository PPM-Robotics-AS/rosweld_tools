"""
ROSWELD
Version 0.0.1, March 2019
http://rosin-project.eu/ftp/rosweld

Copyright (c) 2019 PPM Robotics AS

This library is part of ROSWELD project,
the Focused Technical Project ROSWELD - ROS based framework
for planning, monitoring and control of multi-pass robot welding
is co-financed by the EU project ROSIN (www.rosin-project.eu)
and ROS-Industrial initiative (www.rosindustrial.eu).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import rospy

from src.rosweld.robotcontrollerhandler import RobotControllerHandler
from src.rosweld.project import Project
from src.rosweld.rosproxy import RosProxy

class RosWeld(object):
    """RosWeld main class to start the application
    """

    @staticmethod
    def main():
        """Start ROSweld main program
        Load the project and create the robot controllers
        """

        Project().load()
        RobotControllerHandler()

        RosProxy().notify("ROSweld Core - ready")

        try:
            rospy.spin()

        finally:
            RosProxy().notify("ROSweld Core - stopped")

if __name__ == "__main__":
    RosWeld.main()
