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

from src.rosweld.rosproxy import RosProxy, STATE

class CamParser(object):
    """Parser for cam files
    """

    @staticmethod
    def convert(cam):
        """Convert a given CAM file to poses: float[6][]

        Arguments:
            cam {string} -- the read CAM file's text

        Returns:
            float[6][] -- each in the return list contains the following info:
                          - x (in meter)
                          - y (in meter)
                          - z (in meter)
                          - r
                          - p      (the RPY is a normal vector!!!!!)
                          - y
        """

        poses = []
        error = False

        for i, line in enumerate(cam.splitlines()):
            data = line.split("\t")
            pose = []

            try:
                if len(data) != 6:
                    raise IndexError()

                pose.append(float(data[0]) / 1000)
                pose.append(float(data[1]) / 1000)
                pose.append(float(data[2]) / 1000)
                pose.append(float(data[3]))
                pose.append(float(data[4]))
                pose.append(float(data[5]))
                poses.append(pose)

            except:
                RosProxy().notify("Can't parse %d. line from the path"%(i + 1), STATE.ERROR)
                error = True

        # return with empty array in case of error
        if error:
            return []

        return poses
