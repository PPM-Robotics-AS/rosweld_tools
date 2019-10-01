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

from rosweld_drivers.msg import WeldingState as _WeldingState

WELDING_MODS = {'DC': 0, 'AC': 1}

class WeldingState(_WeldingState):

    def __iadd__(self, other):
        """+= operator overload
        if the other object has a none None value, overwrite the
        self value

        Arguments:
            other {WeldingState} -- other object to add

        Returns:
            WeldingState -- the modified object
        """

        if other is None:
            return self

        if other.amperage is not None:
            self.amperage += other.amperage

        if other.voltage is not None:
            self.voltage += other.voltage

        if other.filler_speed is not None:
            self.filler_speed += other.filler_speed

        if other.default_arc_length is not None:
            self.default_arc_length += other.default_arc_length

        if other.mode is not None:
            self.mode = other.mode

        if other.speed is not None:
            self.speed += other.speed

        return self
