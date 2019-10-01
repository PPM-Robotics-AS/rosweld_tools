#pylint: skip-file
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

import uuid
from abc import ABCMeta, abstractmethod


class ITask(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, task_type, progress, desc, title):
        self.type = task_type
        self.description = desc
        self.progress = progress
        self.title = title
        self.id = str(uuid.uuid4())

    def play(self, data=None):
        pass

    def pause(self):
        pass

    def reset(self):
        pass

    def input(self, data):
        pass

    def get_param(self, name):
        if hasattr(self, name):
            return getattr(self, name)

        return None
