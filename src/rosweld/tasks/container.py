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

import rospy

from src.rosweld.tasks.itask import ITask
from src.rosweld.rosproxy import RosProxy

class ContainerTask(ITask):
    def __init__(
            self,
            type,
            progress,
            desc,
            title,
            tasks=None,
            current_task_idx=None):
        """Init current task
        
        Arguments:
            task_type {string} -- type as string
            progress {float} -- progress
            desc {string} -- description
            title {string} -- title
            
        Keyword Arguments:
            tasks {ITask[]} -- Task list (default: {None})
            current_task_idx {int} -- Current task index (default: {None})
        """

        super(ContainerTask, self).__init__(type, progress, desc, title)

        if tasks is None:
            tasks = []

        self.tasks = tasks
        self.current_task_idx = current_task_idx

        if self.current_task_idx is None and len(self.tasks) > 0:
            self.current_task_idx = 0

    @property
    def current_task(self):
        """Get the current task based on the current_task_idx
        
        Returns:
            [type] -- [description]
        """

        if len(self.tasks) > 0 and len(self.tasks) > self.current_task_idx:
            return self.tasks[self.current_task_idx]

        return None

    def step_fwd(self):
        """Step forward in the task list
        """

        pass

    def step_back(self):
        """Step back in the task list
        """

        pass

    def set_current_task(self, idx):
        """Change the current task by idx
        
        Arguments:
            idx {[type]} -- [description]
        """

        self.current_task_idx = idx
        RosProxy().notify("Set current task id: %d" % (idx))

    def play(self, data=None):
        super(ContainerTask, self).play(data)

        if self.current_task is None:
            return

        self.current_task.play(data)

    def pause(self):
        """Pause the current task
        """

        super(ContainerTask, self).pause()

        if self.current_task is None:
            return

        self.current_task.pause()

    def reset(self):
        """Reset the current task
        """

        super(ContainerTask, self).reset()

    def input(self, data):
        """Forward the input to the current task
        
        Arguments:
            data {dict} -- input data
        """

        self.current_task.input(data)

    def get_task(self, id):
        """Get task in a container

        Arguments:
            id {str} -- Id of the task

        Returns:
            ITask -- search for task by id, None if not found
        """

        for task in self.tasks:
            if isinstance(task, ContainerTask):
                return task.get_task(id)
            elif task.id == id:
                return task

        return None
