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

from geometry_msgs.msg import Pose
from rosweld_tools.msg import CalibrationPoint as _CalibrationPoint


class CalibrationPoint(_CalibrationPoint):
    """CalibrationPoint initilazitaion simplyfication
    """

    def __init__(self, model, measured=None):
        """Create a calibration point with the model point and recorded
        (measured) position and orientation

        Arguments:
            _CalibrationPoint {CalibrationPoint} -- self
            model {Point} -- the coordinates of the calibration point on model space

        Keyword Arguments:
            measured {Pose} -- the measured position and orientation in real world (default: {None})
        """

        super(CalibrationPoint, self).__init__(model=model,
                                               measured=measured if measured is not None\
                                                    else Pose())
