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

class Singleton(type):
    """Singleton metaclass definition
    """

    # current instance of the class
    _instances = {}

    def __call__(cls, *args, **kwargs):
        """Getter of the class

        Returns:
            class -- the stored instance
        """

        if cls not in cls._instances:
            # if there is no instance, create one
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        #return with the instance
        return cls._instances[cls]

    def clear(cls):
        """Clear an instance
        """

        try:
            # remove the instance
            del cls._instances[cls]
        except KeyError:
            return
