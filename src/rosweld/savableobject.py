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

import copy_reg
import cPickle as pickle
import os
from types import FunctionType

from src.rosweld.rosproxy import RosProxy, STATE

class SavableObject(object):
    """Parent class savable object. Makes possible to write out and load
    back the descendant object.
    """

    def __init__(self, savefile):
        """Savable object init

        Arguments:
            savefile {string} -- filename
        """

        self.savefile = savefile

    def load(self):
        """Load the save file if possible
        """

        try:
            if not os.path.isfile(self.savefile + ".rws"):
                self.load_default()
                return

            with open(self.savefile + ".rws", "rb") as f:
                dump = pickle.load(f)
                self.load_object(dump)
        except Exception, e:
            RosProxy().notify("Loading default %s failed: %s"%(self.savefile, str(e)), STATE.ERROR)

    def load_default(self):
        """Load a default, will be overwritten
        """
        pass

    def load_object(self, obj):
        """Loads a given object, will be overwritten

        Arguments:
            obj {object} -- object to load
        """
        pass

    def save(self):
        """Save calibration to a file
        """

        try:
            with open(self.savefile + ".rws", "w+") as f:
                copy_reg.pickle(
                    FunctionType,
                    stub_pickler, stub_unpickler)

                pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)
        except Exception, e:
            RosProxy().notify("Saving %s failed: %s"%(self.savefile, str(e)), STATE.ERROR)

def stub_pickler(obj):
    """a mechanisms to turn unpickable functions int

    Arguments:
        obj {object} -- ubject to pickle

    Returns:
        list -- stub pickled object
    """

    return stub_unpickler, ()

def stub_unpickler():
    """unpickle stub objects

    Returns:
        string -- "STUB" constant
    """

    return "STUB"
