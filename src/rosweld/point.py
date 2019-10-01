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

import numpy as np
from geometry_msgs.msg import Point as _Point


class Point(_Point):
    """Extended ROS geometry_msgs.msg.Point class with:
    - initialize with x,y,z coordinates
    - inline add, sub, div for Points
    - add, sub, div for Points

    Arguments:
        _Point {geometry_msgs.msg.Point} -- base class

    Returns:
        Point -- Point class with extended functionality
    """

    def __init__(self, x=0., y=0., z=0.):
        """init with x,y,z coordinates

        Keyword Arguments:
            x {float} -- x coordinate (default: {0.})
            y {float} -- y coordinate (default: {0.})
            z {float} -- z coordinate (default: {0.})
        """

        super(Point, self).__init__(x=x, y=y, z=z)

    def __iadd__(self, other):
        """inline add (+=) operator overload

        Arguments:
            other {Point} -- other Point to add

        Returns:
            Point -- the incremented self
        """

        self.x += other.x
        self.y += other.y
        self.z += other.z

        return self

    def __add__(self, other):
        """add operator overload

        Arguments:
            other {Point} -- other Point to add

        Returns:
            Point -- a new Point
        """

        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __isub__(self, other):
        """inline substract (-=) operator overload

        Arguments:
            other {Point} -- other Point to sub

        Returns:
            Point -- the decremented self
        """

        self.x -= other.x
        self.y -= other.y
        self.z -= other.z

        return self

    def __sub__(self, other):
        """sub operator overload

        Arguments:
            other {Point} -- other Point to substract

        Returns:
            Point -- a new Point
        """

        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __idiv__(self, other):
        """inline divide (/=) operator overload

        Arguments:
            other {float} -- a number to div

        Returns:
            Point -- the divided self
        """

        self.x /= other
        self.y /= other
        self.z /= other

        return self

    def __div__(self, other):
        """divide operator overload

        Arguments:
            other {float} -- a float number to divide

        Returns:
            Point -- a new Point
        """

        return Point(self.x / other, self.y / other, self.z / other)

    def is_zero(self):
        """Check if a point is near to zero

        Returns:
            bool -- True, if all coordinates are close to 0
        """

        _p = np.array([self.x, self.y, self.z])
        return np.allclose(_p, np.zeros(3))
