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

from src.rosweld.point import Point
from src.rosweld.weldingstate import WeldingState


class Modification(object):
    def __init__(
            self,
            offset=None,
            delta_r=None,
            angle=None,
            weld_params=None):
        """Initalize a Modification object

        Arguments:
            object {Modification} -- self

        Keyword Arguments:
            offset {Point} -- offset (x,y,z) in meters (default: Point(0,0,0))
            delta_r {float} -- delta r angle (default: 0)
            angle {float} -- torch angle (default: 0)
            weld_params {WeldingState} -- welding parameters (default: zero welding parameters)
        """

        if delta_r is None:
            delta_r = 0

        if offset is None:
            offset = Point(0, 0, 0)

        if angle is None:
            angle = 0

        if weld_params is None:
            weld_params = WeldingState()

        self.offset = offset
        self.delta_r = delta_r
        self.angle = angle
        self.welding_parameters = weld_params

    @staticmethod
    def is_zero(mod):
        """Check if the modification is zero

        Arguments:
            mod {Modification} -- modification to check

        Returns:
            bool -- all of the modification's params are equal to zero
        """

        if not mod.offset.is_zero():
            return False

        if not(mod.welding_parameters is None or mod.welding_parameters == WeldingState()):
            return False

        if mod.angle != 0:
            return False

        if mod.delta_r != 0:
            return False

        return True
