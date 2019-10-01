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

import math
import copy
import numpy as np

from geometry_msgs.msg import Pose

from src.rosweld.point import Point
from src.rosweld.transformation import Transformation
from src.rosweld.rosproxy import RosProxy, STATE

class Bead(object):
    """Bead class including the planned and runtime modifications
    """

    def __init__(
            self,
            parent,
            runtime_modifications,
            planned_modifications,
            angle=None,
            offset=None,
            wps_job_number=None):
        """Init a new bead with the given parameters

        Arguments:
            self {Bead} -- self
            parent {Bead} -- parent bead
            runtime_modifications {Modification} -- runtine modifications during the dry run
            planned_modifications {Modification} -- planned modifications

        Keyword Arguments:
            angle {float} -- angle (default: {None})
            offset {Point} -- offset of the bead (default: {None})
            wps_job_number {int} -- selected job number for the bead (default: {None})
        """

        if angle is None:
            angle = 0

        if offset is None:
            offset = Point(0, 0, 0)

        self.wps_job_number = wps_job_number
        self.offset = offset
        self.angle = angle
        self.parent = parent
        self.planned_modifications = planned_modifications
        self.runtime_modifications = runtime_modifications

    def add_modification(self, i, planned, mod):
        """Add a given modification to the runtime or planned list

        Arguments:
            i {int} -- step of the modification
            planned {bool} -- is planned modification
            mod {Modification} -- modification to add
        """

        arr = self.planned_modifications if planned else self.runtime_modifications

        arr[i] = mod

    def apply_modifications_to_path(self, path, angle=0, offset=Point(0, 0, 0), \
        rotation=np.zeros(3)):
        """Apply all modification to the paph

        Arguments:
            path {Pose[]} -- The path to apply the modifications

        Keyword Arguments:
            angle {int} -- Additional angle (roll) to apply (default: {0})
            offset {Point} -- Additional offset to apply (default: {Point(0, 0, 0)})
            rotation {np.array} -- Additional rotation (roll, pitch, jaw) \
                to apply (default: {np.zeros(3)})

        Returns:
            Pose[] -- Pose list with the applied mods
        """

        poses = []
        sum_offset = offset + self.offset
        sum_angle = rotation + \
            np.array([angle + self.angle, 0., 0.])

        # apply modifications
        for i, pose in enumerate(path):
            p = copy.deepcopy(pose)

            realtime_mods = self.get_modification(i, False)
            planned_mods = self.get_modification(i, True)

            if planned_mods is not None:
                sum_offset += planned_mods.offset
                sum_angle[0] += planned_mods.angle
                sum_angle[2] += planned_mods.delta_r

            if realtime_mods is not None:
                sum_offset += realtime_mods.offset
                sum_angle[0] += realtime_mods.angle
                sum_angle[2] += realtime_mods.delta_r

            Transformation.rotate(
                p, list(math.radians(x) for x in sum_angle))
            Transformation.offset(
                p, -1 * np.array([sum_offset.x, sum_offset.y, sum_offset.z, 1.]))

            poses.append(p)

        return poses


    def remove_modification(self, i, planned):
        """Remove a modification from the planned or runtime list

        Arguments:
            i {int} -- step index of the modification
            planned {bool} -- is planned modification
        """

        arr = self.planned_modifications if planned else self.runtime_modifications

        if not i in arr:
            RosProxy().notify("Invalid index", STATE.ERROR)
            return

        del arr[i]

    def get_modification(self, i, planned):
        """Get modification

        Arguments:
            i {int} -- pose index
            planned {bool} -- planned or runtime mod (true if planned)

        Returns:
            Modification -- the selected modification
        """

        arr = self.planned_modifications if planned else self.runtime_modifications

        if i in arr:
            return arr[i]

        return None
