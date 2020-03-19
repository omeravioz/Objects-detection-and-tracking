#!/usr/bin/env python
import rospy
from Locateble_object import Locateble_object


class Camera(Locateble_object):
    def __init__(self, id, pose):
        Locateble_object.__init__(self, id, pose)

    def __str__(self):
        return "camera" + Locateble_object.__str__(self)