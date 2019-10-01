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


from src.rosweld.tasks.container import ContainerTask


class ForCycleContainerTask(ContainerTask):
    def __init__(self, type, progress, desc, title, current_task, tasks):
        super(ForCycleContainerTask, self).__init__(
            type, progress, desc, title, tasks)
        self.current_task = current_task

    def step_fwd(self):
        super(ForCycleContainerTask, self).step_fwd()

    def step_back(self):
        super(ForCycleContainerTask, self).step_back()

    def set_current_task(self, task):
        super(ForCycleContainerTask, self).set_current_task(task)

    def play(self):
        super(ForCycleContainerTask, self).play()

    def pause(self):
        super(ForCycleContainerTask, self).pause()

    def reset(self):
        super(ForCycleContainerTask, self).reset()
